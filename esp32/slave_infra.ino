#include "Wire.h"

// ============================================================
// 1. PIN & CONFIGURATION
// ============================================================

// --- I2C SLAVE (Connection to Raspberry Pi) ---
#define I2C_DEV_ADDR_S 0x55
#define I2C_SLAVE_SDA 15
#define I2C_SLAVE_SCL 13

// --- I2C MASTER (Connection to ADS7128 Sensor) ---
#define ADS_ADDR 0x14
#define ADS_OPCODE_WRITE 0x08

// ADS Registers
#define REG_GENERAL_CFG 0x01
#define REG_DATA_CFG 0x02
#define REG_PIN_CFG 0x05
#define REG_SEQUENCE_CFG 0x10
#define REG_AUTO_SEQ_SEL 0x12

const float V_REF = 3.3;
const float RESISTOR_RATIO = 2.0;
const float MAX_SAFE_DIST_CM = 7; // Safety threshold

// --- GPIO Pins ---
#define RST 4
#define DONE 16

// Motor 1 (Left)
#define M1_PWM_PIN 33
#define M1_DIR_PIN 25
#define M1_NBRK_PIN 26

// Motor 2 (Right)
#define M2_PWM_PIN 19
#define M2_DIR_PIN 18
#define M2_NBRK_PIN 17

// --- MOTOR TUNING ---
#define PWM_FREQ 25000
#define PWM_RESOLUTION 10
#define RAMP_INTERVAL_MS 5 // Time between speed steps during soft stop
#define RAMP_STEP 40       // How much speed to reduce per step
#define DIR_CHANGE_DEADTIME_US 500 // Microseconds to pause after zeroing PWM before flipping direction

// Fix 6: Minimum PWM that actually produces wheel movement (stiction threshold).
// If motors hum but don't move at low speeds, raise this value.
#define MIN_PWM 80

// Fix 5: Per-motor speed trim to compensate for winding/friction differences.
// Straight-line calibration: drive forward, observe drift direction, then
// reduce the trim on the faster side until the robot tracks straight.
// Range: 0.0 – 1.0  (1.0 = no scaling, 0.95 = 5% slower)
const float M1_TRIM = 1.0f; // Left  motor
const float M2_TRIM = 1.0f; // Right motor

// --- STATUS CODES (Sent to Raspberry Pi) ---
#define STATUS_IDLE 0
#define STATUS_MOVING 1
#define STATUS_CLIFF 2

TwoWire I2CSLAVE = TwoWire(1);

// Spinlock protecting the command buffer shared between the I2C task (core 0) and loop() (core 1)
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// 2. STATE MACHINE & VARIABLES
// ============================================================

enum RobotState
{
    STATE_IDLE,
    STATE_MOVING,
    STATE_STOPPING // Soft Stop (Ramping Down)
};

volatile RobotState currentState = STATE_IDLE;
volatile uint8_t robotStatus = STATUS_IDLE; // Status byte to send to Pi

// I2C Command Buffer
volatile bool newCommandReceived = false;
volatile int cmdCommand = 0;
volatile int cmdSpeed = 0;
volatile int cmdDuration = 0;

// Internal Timers & Flags
int m1ActiveSpeed = 0; // Per-motor tracked speeds for correct independent ramp-down
int m2ActiveSpeed = 0;
unsigned long lastStateChange = 0;
unsigned long moveEndTime = 0;
bool moveHasDuration = false;

unsigned long lastSensorRead = 0;
unsigned long lastRampTime = 0;
bool cliffDetected = false;

// ============================================================
// 3. ADS7128 SENSOR FUNCTIONS
// ============================================================

void writeRegister(byte reg, byte val)
{
    Wire.beginTransmission(ADS_ADDR);
    Wire.write(ADS_OPCODE_WRITE);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void initADS7128()
{
    Wire.beginTransmission(ADS_ADDR);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("WARNING: ADS7128 NOT FOUND. Cliff Guard Disabled.");
        return;
    }
    // Hardware Reset Requirement (Datasheet)
    writeRegister(REG_GENERAL_CFG, 0x01);
    delay(10); // Minimal required blocking delay for hardware reset (Run ONCE)
    writeRegister(REG_DATA_CFG, 0x10);
    writeRegister(REG_PIN_CFG, 0x00);
    writeRegister(REG_AUTO_SEQ_SEL, 0x01);
    writeRegister(REG_SEQUENCE_CFG, 0x11);
    Serial.println("ADS7128 Initialized.");
}

void checkCliffSensor()
{
    uint8_t bytesReceived = Wire.requestFrom(ADS_ADDR, 2);

    if (bytesReceived == 2)
    {
        byte msb = Wire.read();
        byte lsb = Wire.read();
        uint16_t rawCode = ((msb << 4) | (lsb >> 4));
        byte channelID = lsb & 0x0F;

        if (channelID == 0)
        {
            float pinV = (rawCode * V_REF) / 4096.0;
            float sensorV = pinV * RESISTOR_RATIO;
            float distCm = 99.0;
            if (sensorV > 0.1)
                distCm = 13.0 / sensorV;

            if (distCm > MAX_SAFE_DIST_CM)
            {
                if (!cliffDetected)
                {
                    Serial.printf("!!! CLIFF DETECTED (%.1f cm) !!!\n", distCm);
                    cliffDetected = true;
                }
            }
            else
            {
                cliffDetected = false;
            }
        }
    }
}

// ============================================================
// 4. MOTOR FUNCTIONS (NON-BLOCKING)
// ============================================================

void setupMotor(int pwmPin, int dirPin, int nbrkPin)
{
    pinMode(dirPin, OUTPUT);
    pinMode(nbrkPin, OUTPUT);
    digitalWrite(dirPin, LOW);
    digitalWrite(nbrkPin, HIGH);
    ledcAttach(pwmPin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pwmPin, 0);
}

// Immediate Hard Stop
void emergencyStop()
{
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, 0);
    m1ActiveSpeed = 0;
    m2ActiveSpeed = 0;
    currentState = STATE_IDLE;
}

// Trigger the Soft Stop State
void triggerSoftStop()
{
    if (m1ActiveSpeed > 0 || m2ActiveSpeed > 0)
    {
        currentState = STATE_STOPPING;
        lastRampTime = millis();
    }
    else
    {
        currentState = STATE_IDLE;
    }
}

// Fix 5+6: Apply per-motor trim and MIN_PWM deadband to a requested speed.
// A speed of 0 (intentional stop / soft-turn idle side) is preserved as 0.
static inline int applyMotorScaling(int speed, float trim)
{
    if (speed == 0)
        return 0;
    int scaled = (int)(speed * trim);
    return constrain(scaled, MIN_PWM, (1 << PWM_RESOLUTION) - 1);
}

// Fix 8: cliffBlock=true  blocks the command when a cliff is detected (forward moves).
//        cliffBlock=false lets the command through anyway    (reverse escape).
void setMotorState(int m1Dir, int m2Dir, int m1Speed, int m2Speed, bool cliffBlock = true)
{
    if (cliffBlock && cliffDetected)
    {
        triggerSoftStop();
        Serial.println("Cmd Blocked: Cliff Active");
        return;
    }

    // Fix 5+6: scale and clamp each motor independently
    int s1 = applyMotorScaling(m1Speed, M1_TRIM);
    int s2 = applyMotorScaling(m2Speed, M2_TRIM);

    // Zero PWM before changing direction to prevent shoot-through current in the DRV8353 FETs
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, 0);
    delayMicroseconds(DIR_CHANGE_DEADTIME_US);

    digitalWrite(M1_DIR_PIN, m1Dir);
    digitalWrite(M2_DIR_PIN, m2Dir);
    ledcWrite(M1_PWM_PIN, s1);
    ledcWrite(M2_PWM_PIN, s2);

    m1ActiveSpeed = s1;
    m2ActiveSpeed = s2;

    if (m1ActiveSpeed > 0 || m2ActiveSpeed > 0)
    {
        currentState = STATE_MOVING;
        lastStateChange = millis();
    }
    else
    {
        currentState = STATE_IDLE;
    }
}

// ============================================================
// 5. I2C COMMUNICATION
// ============================================================

// MODIFIED: Sends Status Byte to Raspberry Pi
void onRequest()
{
    I2CSLAVE.write(robotStatus);
}

void onReceive(int len)
{
    if (len == 0)
        return;
    int c = I2CSLAVE.read();

    // Checksum/Ping
    if (c == 1)
    {
        cmdCommand = 1;
        return;
    }

    if (c >= 20 && c <= 26)
    {
        uint16_t speed = 0;
        uint16_t duration = 0;

        if (c != 26 && I2CSLAVE.available() >= 4)
        {
            byte sL = I2CSLAVE.read();
            byte sH = I2CSLAVE.read();
            speed = (sH << 8) | sL;
            byte dL = I2CSLAVE.read();
            byte dH = I2CSLAVE.read();
            duration = (dH << 8) | dL;
        }

        portENTER_CRITICAL(&cmdMux);
        cmdCommand = c;
        cmdSpeed = speed;
        cmdDuration = duration;
        newCommandReceived = true;
        portEXIT_CRITICAL(&cmdMux);
    }
    // Flush garbage
    while (I2CSLAVE.available())
        I2CSLAVE.read();
}

void i2cSlaveTask(void *pvParameters)
{
    I2CSLAVE.begin((uint8_t)I2C_DEV_ADDR_S, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 100000);
    I2CSLAVE.onReceive(onReceive);
    I2CSLAVE.onRequest(onRequest);
    digitalWrite(DONE, LOW);

    while (1)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS); // Non-blocking yield
    }
}

// ============================================================
// 6. MAIN LOOP (STATE MACHINE)
// ============================================================

void setup()
{
    Serial.begin(115200);
    pinMode(DONE, OUTPUT);
    pinMode(RST, INPUT);

    setupMotor(M1_PWM_PIN, M1_DIR_PIN, M1_NBRK_PIN);
    setupMotor(M2_PWM_PIN, M2_DIR_PIN, M2_NBRK_PIN);

    Wire.begin();
    initADS7128();

    xTaskCreatePinnedToCore(i2cSlaveTask, "I2CSlave", 4096, NULL, 1, NULL, 0);
    Serial.println("System Ready.");
}

void loop()
{
    unsigned long currentMillis = millis();

    // ------------------------------------------
    // 1. SENSOR SAFETY CHECK (Every 50ms)
    // ------------------------------------------
    if (currentMillis - lastSensorRead > 50)
    {
        checkCliffSensor();
        lastSensorRead = currentMillis;

        // EMERGENCY STOP if moving and cliff detected
        if (cliffDetected)
        {
            if (currentState == STATE_MOVING)
            {
                Serial.println("SAFETY: Cliff! Stopping.");
                triggerSoftStop();
            }
            robotStatus = STATUS_CLIFF; // TELL RASPBERRY PI
        }
        else if (currentState == STATE_MOVING)
        {
            robotStatus = STATUS_MOVING;
        }
        else
        {
            robotStatus = STATUS_IDLE;
        }
    }

    // ------------------------------------------
    // 2. PROCESS COMMANDS
    // ------------------------------------------
    // Atomically snapshot the command buffer written by the I2C task on core 0
    portENTER_CRITICAL(&cmdMux);
    bool hasCmd = newCommandReceived;
    int snapCmd = cmdCommand;
    int snapSpeed = cmdSpeed;
    int snapDuration = cmdDuration;
    if (hasCmd)
        newCommandReceived = false;
    portEXIT_CRITICAL(&cmdMux);

    // Fix 7: Clamp speed to valid PWM range before it reaches any motor function
    if (snapSpeed > (1 << PWM_RESOLUTION) - 1)
        snapSpeed = (1 << PWM_RESOLUTION) - 1;

    if (hasCmd)
    {
        if (snapDuration > 0)
        {
            moveEndTime = currentMillis + snapDuration;
            moveHasDuration = true;
        }
        else
        {
            moveHasDuration = false;
        }

        switch (snapCmd)
        {
        case 20:
            setMotorState(LOW, HIGH, snapSpeed, snapSpeed);
            break; // Fwd
        case 21:
            setMotorState(HIGH, LOW, snapSpeed, snapSpeed, false); // Fix 8: reverse always allowed, escapes cliff
            break; // Rev
        case 22:
            setMotorState(LOW, LOW, snapSpeed, snapSpeed);
            break; // Right
        case 23:
            setMotorState(HIGH, HIGH, snapSpeed, snapSpeed);
            break; // Left
        case 24:
            setMotorState(LOW, HIGH, snapSpeed, 0);
            break; // Soft R
        case 25:
            setMotorState(LOW, HIGH, 0, snapSpeed);
            break; // Soft L
        case 26:
            triggerSoftStop();
            moveHasDuration = false;
            break;
        }
    }

    // ------------------------------------------
    // 3. STATE MACHINE HANDLER
    // ------------------------------------------
    switch (currentState)
    {
    case STATE_IDLE:
        break;

    case STATE_MOVING:
        // Duration Timeout
        if (moveHasDuration && currentMillis >= moveEndTime)
        {
            triggerSoftStop();
        }
        break;

    case STATE_STOPPING:
        // Non-blocking Ramp Down — each motor tracked independently
        if (currentMillis - lastRampTime > RAMP_INTERVAL_MS)
        {
            lastRampTime = currentMillis;

            m1ActiveSpeed -= RAMP_STEP;
            if (m1ActiveSpeed < 0)
                m1ActiveSpeed = 0;

            m2ActiveSpeed -= RAMP_STEP;
            if (m2ActiveSpeed < 0)
                m2ActiveSpeed = 0;

            ledcWrite(M1_PWM_PIN, m1ActiveSpeed);
            ledcWrite(M2_PWM_PIN, m2ActiveSpeed);

            if (m1ActiveSpeed == 0 && m2ActiveSpeed == 0)
            {
                currentState = STATE_IDLE;
            }
        }
        break;
    }

    yield();
}