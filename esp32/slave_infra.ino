#include "Wire.h"

// ============================================================
// 1. PIN & CONFIGURATION
// ============================================================

// --- I2C SLAVE (Connection to Raspberry Pi) ---
#define I2C_DEV_ADDR_S 0x55
#define I2C_SLAVE_SDA  15
#define I2C_SLAVE_SCL  13

// --- I2C MASTER (Connection to ADS7128 Cliff Sensor) ---
#define ADS_ADDR         0x14
#define ADS_OPCODE_WRITE 0x08

// ADS7128 Registers (datasheet SBAS869)
#define REG_GENERAL_CFG  0x01  // Bit 0 = software reset
#define REG_DATA_CFG     0x02  // Bit 4 = append 4-bit channel ID to result
#define REG_PIN_CFG      0x05  // 0x00 = all pins as analog input
#define REG_SEQUENCE_CFG 0x10  // Bit 4 = SEQ_START, bit 0 = auto-seq mode
#define REG_AUTO_SEQ_SEL 0x12  // Bit N = include AINn in auto-sequence

// Cliff sensor — Sharp GP2Y0A41 (or equivalent) on AIN0
// sensorV = pinV * RESISTOR_RATIO  (voltage divider: 5V sensor → 3.3V ADC)
// distCm  ≈ 13.0 / sensorV  (empirical; re-calibrate if sensor model differs)
const float V_REF          = 3.3f;
const float RESISTOR_RATIO = 2.0f;
const float MAX_SAFE_DIST_CM = 7.0f;  // Cliff edge threshold (cm)

// Set false to disable cliff detection entirely (motors ignore sensor, no blocking).
#define CLIFF_ENABLED false

// Require this many consecutive over-threshold readings before declaring a cliff,
// and this many consecutive clear readings before releasing it.
// At 50 ms polling that's 150 ms to arm, 150 ms to disarm — filters ADC spikes.
#define CLIFF_CONFIRM 3

// --- GPIO Pins ---
#define RST  4
#define DONE 16

// Motor 1 (Left)
#define M1_PWM_PIN  33
#define M1_DIR_PIN  25
#define M1_NBRK_PIN 26   // Active-LOW brake input on DRV8874/similar

// Motor 2 (Right)
#define M2_PWM_PIN  19
#define M2_DIR_PIN  18
#define M2_NBRK_PIN 17

// --- MOTOR TUNING ---
#define PWM_FREQ           25000  // Hz — above human hearing
#define PWM_RESOLUTION        10  // Bits — range 0..1023
#define PWM_MAX          ((1 << PWM_RESOLUTION) - 1)   // 1023

// Speed protocol: Pi sends 0–150. This maps onto MIN_PWM..PWM_MAX.
// MIN_PWM is the stiction floor — raise it if motors hum but don't turn.
#define SPEED_INPUT_MAX      150
#define MIN_PWM               80  // Tune per motor

// Per-motor trim: multiply PWM by this to compensate winding/friction differences.
// Drive straight, watch which side leads, lower THAT motor's trim (e.g. 0.95).
const float M1_TRIM = 1.0f;  // Left  motor  (1.0 = no correction)
const float M2_TRIM = 1.0f;  // Right motor

// Soft stop ramp
#define RAMP_INTERVAL_MS   5    // ms between ramp steps
#define RAMP_STEP         40    // PWM counts to subtract per step

// Safety: pause after zeroing PWM before flipping direction pin,
// preventing shoot-through current through the H-bridge FETs.
#define DIR_CHANGE_DEADTIME_US 500

// Watchdog: emergency-stop if no I2C command arrives within this window.
// Set to 0 to disable (not recommended for a mobile robot).
#define CMD_WATCHDOG_MS 500

// --- STATUS CODES (Sent to Raspberry Pi on I2C read) ---
#define STATUS_IDLE     0
#define STATUS_MOVING   1
#define STATUS_CLIFF    2
#define STATUS_STOPPING 3

TwoWire I2CSLAVE = TwoWire(1);

// Spinlock protecting the command buffer shared between the I2C task (core 0)
// and loop() (core 1).
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// 2. STATE MACHINE & VARIABLES
// ============================================================

enum RobotState { STATE_IDLE, STATE_MOVING, STATE_STOPPING };

volatile RobotState currentState = STATE_IDLE;
volatile uint8_t    robotStatus  = STATUS_IDLE;

// I2C command buffer — written by core 0 (I2C task), read by core 1 (loop).
volatile bool     newCommandReceived = false;
volatile int      cmdCommand  = 0;
volatile int      cmdSpeed    = 0;
volatile int      cmdDuration = 0;

// Per-motor PWM values tracked for independent ramp-down.
int m1ActiveSpeed = 0;
int m2ActiveSpeed = 0;

unsigned long lastStateChange = 0;
unsigned long moveEndTime     = 0;
bool          moveHasDuration = false;

unsigned long lastSensorRead = 0;
unsigned long lastRampTime   = 0;
unsigned long lastCmdTime    = 0;  // Watchdog timestamp

// Cliff hysteresis counters
bool    cliffDetected  = false;
uint8_t cliffArmCount  = 0;   // Counts up while distance > threshold
uint8_t cliffClearCount = 0;  // Counts up while distance ≤ threshold

// ============================================================
// 3. ADS7128 CLIFF SENSOR
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
    if (Wire.endTransmission() != 0) {
        Serial.println("WARNING: ADS7128 NOT FOUND — Cliff guard disabled.");
        return;
    }
    // Software reset (datasheet §8.5.1 — Bit 0 of GENERAL_CFG)
    writeRegister(REG_GENERAL_CFG,  0x01);
    delay(10);                           // Mandatory reset settling time

    // Append 4-bit channel ID to each conversion result (DATA_CFG Bit 4)
    writeRegister(REG_DATA_CFG,     0x10);
    // All pins as analog input
    writeRegister(REG_PIN_CFG,      0x00);
    // Auto-sequence AIN0 only (Bit 0 = AIN0 enabled)
    writeRegister(REG_AUTO_SEQ_SEL, 0x01);
    // Start auto-sequence mode (SEQ_START | SEQ_MODE=1)
    writeRegister(REG_SEQUENCE_CFG, 0x11);

    Serial.println("ADS7128 OK — cliff sensor active on AIN0.");
}

// Print a live sensor line every N sensor ticks (50 ms each → every 500 ms).
// Set to 1 to log every reading; set to 0 to silence periodic logs entirely.
#define SENSOR_LOG_EVERY 10

static uint16_t sensorLogTick = 0;

// Called every 50 ms from loop(). Uses hysteresis counters to avoid acting on
// single noisy ADC samples. CLIFF_CONFIRM consecutive readings required to
// arm OR disarm the cliff flag.
void checkCliffSensor()
{
    uint8_t n = Wire.requestFrom(ADS_ADDR, 2);
    if (n != 2) {
        Serial.println("[IR] I2C read failed — check ADS7128 wiring.");
        return;
    }

    byte     msb      = Wire.read();
    byte     lsb      = Wire.read();
    uint16_t rawCode  = (msb << 4) | (lsb >> 4);   // 12-bit result
    byte     chanID   = lsb & 0x0F;                  // Which ADS channel

    if (chanID != 0) return;  // Only process AIN0

    // --- Distance calculation ---
    float pinV    = (rawCode * V_REF) / 4096.0f;
    float sensorV = pinV * RESISTOR_RATIO;

    // Sensor range: roughly 4–30 cm. Outside that range treat as "no object = clear".
    float distCm = (sensorV > 0.1f) ? (13.0f / sensorV) : 99.0f;
    bool  overThreshold = (distCm > MAX_SAFE_DIST_CM);

    // ── Periodic terminal log ──────────────────────────────────────────────
#if SENSOR_LOG_EVERY > 0
    if (++sensorLogTick >= SENSOR_LOG_EVERY) {
        sensorLogTick = 0;
        Serial.printf("[IR] raw=%4u  pinV=%5.3fV  sensorV=%5.3fV  dist=%5.1f cm  %s\n",
                      rawCode, pinV, sensorV, distCm,
                      cliffDetected ? "*** CLIFF ***" : (overThreshold ? "(arming)" : "clear"));
    }
#endif

    // ── Hysteresis ────────────────────────────────────────────────────────
    if (overThreshold) {
        cliffClearCount = 0;
        if (cliffArmCount < CLIFF_CONFIRM) cliffArmCount++;
        if (cliffArmCount >= CLIFF_CONFIRM && !cliffDetected) {
            cliffDetected = true;
            Serial.printf("[IR] !!! CLIFF DETECTED (%.1f cm) — armed after %d readings\n",
                          distCm, CLIFF_CONFIRM);
        }
    } else {
        cliffArmCount = 0;
        if (cliffClearCount < CLIFF_CONFIRM) cliffClearCount++;
        if (cliffClearCount >= CLIFF_CONFIRM && cliffDetected) {
            cliffDetected = false;
            Serial.println("[IR] Cliff cleared.");
        }
    }
}

// ============================================================
// 4. MOTOR FUNCTIONS
// ============================================================

void setupMotor(int pwmPin, int dirPin, int nbrkPin)
{
    pinMode(dirPin,  OUTPUT);
    pinMode(nbrkPin, OUTPUT);
    digitalWrite(dirPin,  LOW);
    digitalWrite(nbrkPin, LOW);   // Brakes engaged at power-on (safe default)
    ledcAttach(pwmPin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pwmPin, 0);
}

// Engage hardware brakes on both motors (nBRK = LOW = brake).
void engageBrakes()
{
    digitalWrite(M1_NBRK_PIN, LOW);
    digitalWrite(M2_NBRK_PIN, LOW);
}

// Release brakes so motors can spin (nBRK = HIGH = run).
void releaseBrakes()
{
    digitalWrite(M1_NBRK_PIN, HIGH);
    digitalWrite(M2_NBRK_PIN, HIGH);
}

// Immediate hard stop + brake. Use for safety emergencies.
void emergencyStop()
{
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, 0);
    m1ActiveSpeed = 0;
    m2ActiveSpeed = 0;
    engageBrakes();
    currentState = STATE_IDLE;
}

// Begin a non-blocking ramp-down. Brakes engage when ramp reaches zero.
void triggerSoftStop()
{
    if (m1ActiveSpeed > 0 || m2ActiveSpeed > 0) {
        currentState = STATE_STOPPING;
        lastRampTime = millis();
    } else {
        engageBrakes();
        currentState = STATE_IDLE;
    }
}

// Map incoming speed (0–SPEED_INPUT_MAX) to the actual PWM range (MIN_PWM..PWM_MAX),
// then apply per-motor trim. A speed of 0 is always output as 0 (intentional stop).
static inline int applyMotorScaling(int speed, float trim)
{
    if (speed == 0) return 0;
    int clamped = constrain(speed, 0, SPEED_INPUT_MAX);
    // Map full input range → MIN_PWM..PWM_MAX so even speed=1 produces movement
    int pwm = map(clamped, 1, SPEED_INPUT_MAX, MIN_PWM, PWM_MAX);
    return constrain((int)(pwm * trim), MIN_PWM, PWM_MAX);
}

// Apply direction + scaled speed to both motors.
// cliffBlock=false allows the command through even if a cliff is detected (used
// for reverse escape: moving away from the edge is always safe).
void setMotorState(int m1Dir, int m2Dir, int m1Speed, int m2Speed, bool cliffBlock = true)
{
    if (CLIFF_ENABLED && cliffBlock && cliffDetected) {
        triggerSoftStop();
        Serial.println("Cmd blocked: cliff active.");
        return;
    }

    int s1 = applyMotorScaling(m1Speed, M1_TRIM);
    int s2 = applyMotorScaling(m2Speed, M2_TRIM);

    // Release brakes before applying PWM
    releaseBrakes();

    // Zero PWM briefly before flipping direction to prevent shoot-through
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, 0);
    delayMicroseconds(DIR_CHANGE_DEADTIME_US);

    digitalWrite(M1_DIR_PIN, m1Dir);
    digitalWrite(M2_DIR_PIN, m2Dir);
    ledcWrite(M1_PWM_PIN, s1);
    ledcWrite(M2_PWM_PIN, s2);

    m1ActiveSpeed = s1;
    m2ActiveSpeed = s2;

    if (s1 > 0 || s2 > 0) {
        currentState  = STATE_MOVING;
        lastStateChange = millis();
    } else {
        engageBrakes();
        currentState = STATE_IDLE;
    }
}

// ============================================================
// 5. I2C COMMUNICATION
// ============================================================

void onRequest()
{
    I2CSLAVE.write(robotStatus);
}

void onReceive(int len)
{
    if (len == 0) return;
    int c = I2CSLAVE.read();

    // Ping / connection check
    if (c == 1) {
        portENTER_CRITICAL(&cmdMux);
        cmdCommand = 1;
        newCommandReceived = true;
        portEXIT_CRITICAL(&cmdMux);
        while (I2CSLAVE.available()) I2CSLAVE.read();
        return;
    }

    if (c >= 20 && c <= 26) {
        uint16_t speed    = 0;
        uint16_t duration = 0;

        if (c != 26 && I2CSLAVE.available() >= 4) {
            byte sL = I2CSLAVE.read();
            byte sH = I2CSLAVE.read();
            speed    = (sH << 8) | sL;
            byte dL = I2CSLAVE.read();
            byte dH = I2CSLAVE.read();
            duration = (dH << 8) | dL;
        }

        portENTER_CRITICAL(&cmdMux);
        cmdCommand          = c;
        cmdSpeed            = speed;
        cmdDuration         = duration;
        newCommandReceived  = true;
        portEXIT_CRITICAL(&cmdMux);
    }

    while (I2CSLAVE.available()) I2CSLAVE.read();  // Flush unexpected bytes
}

void i2cSlaveTask(void *pvParameters)
{
    I2CSLAVE.begin((uint8_t)I2C_DEV_ADDR_S, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 100000);
    I2CSLAVE.onReceive(onReceive);
    I2CSLAVE.onRequest(onRequest);
    digitalWrite(DONE, LOW);  // Signal Pi that ESP32 is ready
    for (;;) vTaskDelay(1 / portTICK_PERIOD_MS);
}

// ============================================================
// 6. SETUP & MAIN LOOP
// ============================================================

void setup()
{
    Serial.begin(115200);
    pinMode(DONE, OUTPUT);
    pinMode(RST,  INPUT);

    setupMotor(M1_PWM_PIN, M1_DIR_PIN, M1_NBRK_PIN);
    setupMotor(M2_PWM_PIN, M2_DIR_PIN, M2_NBRK_PIN);

    Wire.begin();
    initADS7128();

    lastCmdTime = millis();
    xTaskCreatePinnedToCore(i2cSlaveTask, "I2CSlave", 4096, NULL, 1, NULL, 0);
    Serial.println("Walter ESP32 ready.");
}

void loop()
{
    unsigned long now = millis();

    // ── 1. CLIFF SENSOR (every 50 ms) ──────────────────────────────────────
    if (now - lastSensorRead >= 50) {
        lastSensorRead = now;
        checkCliffSensor();

        if (CLIFF_ENABLED && cliffDetected && currentState == STATE_MOVING) {
            Serial.println("SAFETY: cliff — soft stop.");
            triggerSoftStop();
        }

        // Update status byte for Pi
        if      (CLIFF_ENABLED && cliffDetected) robotStatus = STATUS_CLIFF;
        else if (currentState == STATE_MOVING)  robotStatus = STATUS_MOVING;
        else if (currentState == STATE_STOPPING) robotStatus = STATUS_STOPPING;
        else                                    robotStatus = STATUS_IDLE;
    }

    // ── 2. WATCHDOG ─────────────────────────────────────────────────────────
#if CMD_WATCHDOG_MS > 0
    if (currentState == STATE_MOVING && (now - lastCmdTime) > CMD_WATCHDOG_MS) {
        Serial.println("WATCHDOG: no command — emergency stop.");
        emergencyStop();
    }
#endif

    // ── 3. PROCESS INCOMING COMMAND ─────────────────────────────────────────
    portENTER_CRITICAL(&cmdMux);
    bool hasCmd      = newCommandReceived;
    int  snapCmd     = cmdCommand;
    int  snapSpeed   = cmdSpeed;
    int  snapDur     = cmdDuration;
    if (hasCmd) newCommandReceived = false;
    portEXIT_CRITICAL(&cmdMux);

    if (hasCmd) {
        lastCmdTime = now;  // Reset watchdog on every valid command

        if (snapCmd == 1) {
            // Ping — just reset watchdog, no movement
        } else {
            moveHasDuration = (snapDur > 0);
            if (moveHasDuration) moveEndTime = now + snapDur;

            switch (snapCmd) {
            case 20: setMotorState(LOW,  HIGH, snapSpeed, snapSpeed);        break; // Fwd
            case 21: setMotorState(HIGH, LOW,  snapSpeed, snapSpeed, false); break; // Rev (cliff-exempt)
            case 22: setMotorState(LOW,  LOW,  snapSpeed, snapSpeed);        break; // Turn right
            case 23: setMotorState(HIGH, HIGH, snapSpeed, snapSpeed);        break; // Turn left
            case 24: setMotorState(LOW,  HIGH, snapSpeed, 0);               break; // Pivot right (M1 only)
            case 25: setMotorState(LOW,  HIGH, 0, snapSpeed);               break; // Pivot left  (M2 only)
            case 26: triggerSoftStop(); moveHasDuration = false;             break; // Soft stop
            }
        }
    }

    // ── 4. STATE MACHINE ────────────────────────────────────────────────────
    switch (currentState) {

    case STATE_IDLE:
        break;

    case STATE_MOVING:
        if (moveHasDuration && now >= moveEndTime)
            triggerSoftStop();
        break;

    case STATE_STOPPING:
        if (now - lastRampTime >= RAMP_INTERVAL_MS) {
            lastRampTime = now;

            m1ActiveSpeed = max(0, m1ActiveSpeed - RAMP_STEP);
            m2ActiveSpeed = max(0, m2ActiveSpeed - RAMP_STEP);

            ledcWrite(M1_PWM_PIN, m1ActiveSpeed);
            ledcWrite(M2_PWM_PIN, m2ActiveSpeed);

            if (m1ActiveSpeed == 0 && m2ActiveSpeed == 0) {
                engageBrakes();      // Active brake once fully stopped
                currentState = STATE_IDLE;
                Serial.println("Stopped.");
            }
        }
        break;
    }

    yield();
}
