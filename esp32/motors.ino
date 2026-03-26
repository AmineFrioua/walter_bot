
/*
 * ═══════════════════════════════════════════════════════════════════════════
 * DRV8353F Dual-Motor Driver Test - Walter Robot
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Description:
 *   Comprehensive test of both motors with directional movements including
 *   forward, reverse, turn left, and turn right maneuvers.
 *
 * Hardware:
 *   - Motor Driver: DRV8353F
 *   - MCU: ESP32
 *   - Motors: 2x DC motors (M1 = Left, M2 = Right)
 *
 * Movement Logic:
 *   Forward:     M1=Forward,  M2=Forward
 *   Reverse:     M1=Reverse,  M2=Reverse
 *   Turn Right:  M1=Forward,  M2=Stopped/Reverse
 *   Turn Left:   M1=Stopped/Reverse, M2=Forward
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════════════════════
// Pin Definitions
// ═══════════════════════════════════════════════════════════════════════════

// Motor 1 (Left) Control Pins
#define M1_PWM_PIN 33
#define M1_DIR_PIN 25
#define M1_NBRK_PIN 26

#define M2_PWM_PIN 19
#define M2_DIR_PIN 18
#define M2_NBRK_PIN 17

const int PWM_FREQ = 25000;
const int PWM_RESOLUTION = 10;      // 10-bit resolution (0-1023)
const int TEST_DUTY_CYCLE = 200;    // ~20% duty cycle for safe testing
const int TURN_DUTY_CYCLE = 250;    // ~24% duty cycle for turns
const int MAX_DUTY_CYCLE = 1023;    // Maximum PWM value

// Timing configuration
const int RUN_TIME_MS = 2000;       // Duration for each movement test
const int TURN_TIME_MS = 1500;      // Duration for turn maneuvers
const int STOP_TIME_MS = 500;       // Pause between movements

// Direction constants
#define MOTOR_FORWARD  LOW
#define MOTOR_REVERSE  HIGH
#define BRAKE_DISABLED HIGH
#define BRAKE_ENABLED  LOW

void setupMotor(int pwmPin, int dirPin, int nbrkPin, const char *motorName)
{
    Serial.print("  → Initializing ");
    Serial.print(motorName);
    Serial.println("...");

    pinMode(dirPin, OUTPUT);
    pinMode(nbrkPin, OUTPUT);

    digitalWrite(dirPin, MOTOR_FORWARD);
    digitalWrite(nbrkPin, BRAKE_DISABLED);

    ledcAttach(pwmPin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pwmPin, 0);
}

void stopAllMotors()
{
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, 0);
}

void moveForward(int speed, int duration)
{
    Serial.print("  ▶ Moving FORWARD (");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, HIGH);
    ledcWrite(M1_PWM_PIN, speed);
    ledcWrite(M2_PWM_PIN, speed);
    delay(duration);
    stopAllMotors();
}

void moveReverse(int speed, int duration)
{
    Serial.print("  ◀ Moving REVERSE (");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, HIGH);
    digitalWrite(M2_DIR_PIN, LOW);
    ledcWrite(M1_PWM_PIN, speed);
    ledcWrite(M2_PWM_PIN, speed);
    delay(duration);
    stopAllMotors();
}

void turnRight(int speed, int duration)
{
    Serial.print("  ↻ Turning RIGHT (");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, LOW);
    ledcWrite(M1_PWM_PIN, speed);
    ledcWrite(M2_PWM_PIN, speed);
    delay(duration);
    stopAllMotors();
}

void turnLeft(int speed, int duration)
{
    Serial.print("  ↺ Turning LEFT (");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, HIGH);
    digitalWrite(M2_DIR_PIN, HIGH);
    ledcWrite(M1_PWM_PIN, speed);
    ledcWrite(M2_PWM_PIN, speed);
    delay(duration);
    stopAllMotors();
}

void turnRightSoft(int speed, int duration)
{
    Serial.print("  → Turning RIGHT (soft, ");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, HIGH);
    ledcWrite(M1_PWM_PIN, speed);
    ledcWrite(M2_PWM_PIN, 0);
    delay(duration);
    stopAllMotors();
}

void turnLeftSoft(int speed, int duration)
{
    Serial.print("  ← Turning LEFT (soft, ");
    Serial.print((speed * 100) / MAX_DUTY_CYCLE);
    Serial.print("%, ");
    Serial.print(duration);
    Serial.println("ms)");

    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, HIGH);
    ledcWrite(M1_PWM_PIN, 0);
    ledcWrite(M2_PWM_PIN, speed);
    delay(duration);
    stopAllMotors();
}

void applyBrakes()
{
    Serial.println("  ⚫ Applying brakes to both motors");
    digitalWrite(M1_NBRK_PIN, BRAKE_ENABLED);
    digitalWrite(M2_NBRK_PIN, BRAKE_ENABLED);
    delay(100);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Print header
    Serial.println();
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║   DRV8353F Dual-Motor Movement Test - Walter Robot       ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println();

    // Display configuration
    Serial.println("┌─ Configuration ──────────────────────────────────────────┐");
    Serial.print("  PWM Frequency: ");
    Serial.print(PWM_FREQ / 1000);
    Serial.println(" kHz");
    Serial.print("  Test Speed: ");
    Serial.print(TEST_DUTY_CYCLE);
    Serial.print(" (~");
    Serial.print((TEST_DUTY_CYCLE * 100) / MAX_DUTY_CYCLE);
    Serial.println("%)");
    Serial.print("  Turn Speed: ");
    Serial.print(TURN_DUTY_CYCLE);
    Serial.print(" (~");
    Serial.print((TURN_DUTY_CYCLE * 100) / MAX_DUTY_CYCLE);
    Serial.println("%)");
    Serial.println("└──────────────────────────────────────────────────────────┘");
    Serial.println();

    // Initialize motors
    Serial.println("┌─ Initialization ─────────────────────────────────────────┐");
    setupMotor(M1_PWM_PIN, M1_DIR_PIN, M1_NBRK_PIN, "Motor 1 (Left)");
    setupMotor(M2_PWM_PIN, M2_DIR_PIN, M2_NBRK_PIN, "Motor 2 (Right)");
    Serial.println("└──────────────────────────────────────────────────────────┘");
    Serial.println();

    // Test sequence
    Serial.println("┌─ Movement Test Sequence ─────────────────────────────────┐");

    // Forward movement
    moveForward(TEST_DUTY_CYCLE, RUN_TIME_MS);
    delay(STOP_TIME_MS);

    // Reverse movement
    moveReverse(TEST_DUTY_CYCLE, RUN_TIME_MS);
    delay(STOP_TIME_MS);

    // Turn right (pivot)
    turnRight(TURN_DUTY_CYCLE, TURN_TIME_MS);
    delay(STOP_TIME_MS);

    // Turn left (pivot)
    turnLeft(TURN_DUTY_CYCLE, TURN_TIME_MS);
    delay(STOP_TIME_MS);

    // Soft turn right
    turnRightSoft(TURN_DUTY_CYCLE, TURN_TIME_MS);
    delay(STOP_TIME_MS);

    // Soft turn left
    turnLeftSoft(TURN_DUTY_CYCLE, TURN_TIME_MS);
    delay(STOP_TIME_MS);

    // Forward again
    moveForward(TEST_DUTY_CYCLE, RUN_TIME_MS);
    delay(STOP_TIME_MS);

    // Reverse again
    moveReverse(TEST_DUTY_CYCLE, RUN_TIME_MS);

    Serial.println("└──────────────────────────────────────────────────────────┘");
    Serial.println();

    // Apply brakes
    applyBrakes();

    // Final status
    Serial.println("╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║                    ✓ Test Complete                       ║");
    Serial.println("║          All movement patterns tested successfully       ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Demo finished. Motors are braked and idle.");
    Serial.println("Reset the board to run the test again.");
}

void loop()
{
    // Demo runs once in setup() - no continuous operation
}