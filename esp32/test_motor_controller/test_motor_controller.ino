/*
 * test_motor_controller.ino
 * ─────────────────────────
 * Run this on an Arduino UNO/Mega (or a second ESP32) wired as I2C master
 * to the Walter ESP32 motor controller (I2C slave at 0x55).
 *
 * Arduino UNO wiring:
 *   SDA → A4    (ESP32 slave SDA = GPIO 15)
 *   SCL → A5    (ESP32 slave SCL = GPIO 13)
 *   GND → GND  (common ground REQUIRED)
 *
 * ESP32 master wiring (uncomment ESP32_MASTER below):
 *   SDA → GPIO 21    SCL → GPIO 22
 *
 * Open Serial Monitor at 115200 baud.
 * Tests run automatically in sequence; press any key to repeat.
 */

#include <Wire.h>

// ── Configuration ─────────────────────────────────────────────────────────

#define SLAVE_ADDR   0x55

// Uncomment if running on an ESP32 master instead of Arduino
// #define ESP32_MASTER
#ifdef ESP32_MASTER
  #define MASTER_SDA 21
  #define MASTER_SCL 22
#endif

// Command codes (must match slave firmware)
#define CMD_PING      1
#define CMD_FWD      20
#define CMD_REV      21
#define CMD_TURN_R   22
#define CMD_TURN_L   23
#define CMD_PIVOT_R  24
#define CMD_PIVOT_L  25
#define CMD_STOP     26

// Status codes returned by the slave
#define STATUS_IDLE     0
#define STATUS_MOVING   1
#define STATUS_CLIFF    2
#define STATUS_STOPPING 3

// Speed to use in tests (0–150; maps to full PWM range on ESP32)
#define TEST_SPEED     80

// ── Helpers ───────────────────────────────────────────────────────────────

// Send a movement command: [cmd][speedL speedH][durL durH]
void sendCmd(uint8_t cmd, uint16_t speed = 0, uint16_t durationMs = 0)
{
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(cmd);
    if (cmd != CMD_STOP) {
        Wire.write((uint8_t)(speed      & 0xFF));
        Wire.write((uint8_t)(speed      >> 8));
        Wire.write((uint8_t)(durationMs & 0xFF));
        Wire.write((uint8_t)(durationMs >> 8));
    }
    uint8_t err = Wire.endTransmission();

    Serial.print("  TX cmd="); Serial.print(cmd);
    Serial.print(" spd=");     Serial.print(speed);
    Serial.print(" dur=");     Serial.print(durationMs);
    Serial.print(" ms  →  ");
    Serial.println(err == 0 ? "ACK ✓" : "NO ACK ✗ (check wiring / address)");
}

// Read and decode the one-byte status the slave sends back on I2C read.
uint8_t readStatus()
{
    Wire.requestFrom(SLAVE_ADDR, 1);
    if (!Wire.available()) {
        Serial.println("  STATUS: <no response>");
        return 0xFF;
    }
    uint8_t s = Wire.read();
    Serial.print("  STATUS: ");
    switch (s) {
        case STATUS_IDLE:     Serial.println("IDLE");     break;
        case STATUS_MOVING:   Serial.println("MOVING");   break;
        case STATUS_CLIFF:    Serial.println("CLIFF");    break;
        case STATUS_STOPPING: Serial.println("STOPPING"); break;
        default:              Serial.print("UNKNOWN ("); Serial.print(s); Serial.println(")");
    }
    return s;
}

// Assert helper — prints PASS/FAIL.
void assertStatus(uint8_t actual, uint8_t expected, const char* testName)
{
    Serial.print("  ["); Serial.print(testName); Serial.print("] ");
    if (actual == expected) {
        Serial.println("PASS");
    } else {
        Serial.print("FAIL — expected "); Serial.print(expected);
        Serial.print(", got ");           Serial.println(actual);
    }
}

void separator(const char* title)
{
    Serial.println();
    Serial.print("── "); Serial.print(title); Serial.println(" ──────────────────────");
}

// ── Test Cases ─────────────────────────────────────────────────────────────

void testPing()
{
    separator("T1: PING / CONNECTION");
    Wire.beginTransmission(SLAVE_ADDR);
    uint8_t err = Wire.endTransmission();
    Serial.print("  I2C probe 0x55: ");
    Serial.println(err == 0 ? "FOUND ✓" : "NOT FOUND ✗");
    sendCmd(CMD_PING);
    delay(50);
    uint8_t s = readStatus();
    // After ping robot should still be idle
    assertStatus(s, STATUS_IDLE, "idle after ping");
}

void testForward()
{
    separator("T2: FORWARD 2 s");
    sendCmd(CMD_FWD, TEST_SPEED, 2000);
    delay(100);
    uint8_t s = readStatus();
    assertStatus(s, STATUS_MOVING, "moving after fwd cmd");
    Serial.println("  >>> Watch robot — should move FORWARD <<<");
    delay(2500);   // Wait for duration to expire + ramp-down
    s = readStatus();
    assertStatus(s, STATUS_IDLE, "idle after duration");
}

void testReverse()
{
    separator("T3: REVERSE 2 s");
    sendCmd(CMD_REV, TEST_SPEED, 2000);
    delay(100);
    uint8_t s = readStatus();
    assertStatus(s, STATUS_MOVING, "moving after rev cmd");
    Serial.println("  >>> Watch robot — should move BACKWARD <<<");
    delay(2500);
    s = readStatus();
    assertStatus(s, STATUS_IDLE, "idle after duration");
}

void testTurnRight()
{
    separator("T4: TURN RIGHT 1 s");
    sendCmd(CMD_TURN_R, TEST_SPEED, 1000);
    delay(100);
    Serial.println("  >>> Watch robot — should turn RIGHT (clockwise) <<<");
    delay(1500);
    readStatus();
}

void testTurnLeft()
{
    separator("T5: TURN LEFT 1 s");
    sendCmd(CMD_TURN_L, TEST_SPEED, 1000);
    delay(100);
    Serial.println("  >>> Watch robot — should turn LEFT (counter-clockwise) <<<");
    delay(1500);
    readStatus();
}

void testSoftStop()
{
    separator("T6: SOFT STOP during movement");
    sendCmd(CMD_FWD, TEST_SPEED, 0);  // dur=0 → run indefinitely
    delay(500);
    uint8_t s = readStatus();
    assertStatus(s, STATUS_MOVING, "moving before stop");
    Serial.println("  Sending STOP command...");
    sendCmd(CMD_STOP);
    delay(200);
    s = readStatus();
    // Should be STOPPING or IDLE depending on ramp speed
    bool ok = (s == STATUS_STOPPING || s == STATUS_IDLE);
    Serial.print("  [stopping/idle after stop cmd] ");
    Serial.println(ok ? "PASS" : "FAIL");
    delay(500);  // Let ramp finish
    s = readStatus();
    assertStatus(s, STATUS_IDLE, "idle after ramp");
}

void testSpeedLadder()
{
    separator("T7: SPEED LADDER (check motor responds at low speeds)");
    uint16_t speeds[] = { 20, 50, 80, 100, 150 };
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print("  Speed "); Serial.print(speeds[i]);
        Serial.print("/150 → PWM ~");
        Serial.print(map(speeds[i], 1, 150, 80, 1023));
        Serial.println("/1023");
        sendCmd(CMD_FWD, speeds[i], 0);
        delay(600);
    }
    sendCmd(CMD_STOP);
    delay(300);
    readStatus();
}

void testWatchdog()
{
    separator("T8: WATCHDOG (stop sending for >500 ms while moving)");
    sendCmd(CMD_FWD, TEST_SPEED, 0);
    delay(100);
    assertStatus(readStatus(), STATUS_MOVING, "moving before silence");
    Serial.println("  Waiting 700 ms without sending any command...");
    delay(700);   // Slave watchdog fires at 500 ms
    uint8_t s = readStatus();
    bool ok = (s == STATUS_IDLE || s == STATUS_STOPPING);
    Serial.print("  [watchdog triggered — stopped] ");
    Serial.println(ok ? "PASS" : "FAIL (watchdog may be disabled in firmware)");
    delay(300);
}

void testCliffSimulation()
{
    separator("T9: CLIFF STATUS MONITORING (30 readings over 2 s)");
    Serial.println("  Place robot near / over cliff edge to see STATUS_CLIFF.");
    Serial.println("  Leave on flat ground to see STATUS_IDLE.");
    for (int i = 0; i < 30; i++) {
        uint8_t s = readStatus();
        if (s == STATUS_CLIFF) {
            Serial.println("  *** CLIFF DETECTED — robot is protected ***");
        }
        delay(70);
    }
}

// ── Entry Points ───────────────────────────────────────────────────────────

void runAllTests()
{
    testPing();       delay(500);
    testForward();    delay(500);
    testReverse();    delay(500);
    testTurnRight();  delay(500);
    testTurnLeft();   delay(500);
    testSoftStop();   delay(500);
    testSpeedLadder(); delay(500);
    testWatchdog();   delay(500);
    testCliffSimulation();

    Serial.println();
    Serial.println("══════════════════════════════════════");
    Serial.println("All tests complete. Send any character");
    Serial.println("over Serial to run again.");
    Serial.println("══════════════════════════════════════");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);

#ifdef ESP32_MASTER
    Wire.begin(MASTER_SDA, MASTER_SCL);
#else
    Wire.begin();   // Arduino: SDA=A4, SCL=A5
#endif
    Wire.setClock(100000);  // Match slave firmware clock (100 kHz)

    delay(500);  // Give slave time to boot

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║   Walter ESP32 Motor Controller Test ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.println("Slave address: 0x55");
    Serial.println("Speed in tests: " + String(TEST_SPEED) + "/150");

    runAllTests();
}

void loop()
{
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runAllTests();
    }
}
