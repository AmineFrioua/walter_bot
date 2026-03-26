#include <Wire.h>

// --- I2C Configuration ---
const byte ADS7128_ADDR = 0x14; // Default I2C Address (ADDR pin with 100kΩ)
const byte OPCODE_WRITE = 0x08; // Opcode for "Single Register Write"

// --- Register Addresses [cite: 1517] ---
const byte REG_GENERAL_CFG = 0x01;  // For Reset
const byte REG_DATA_CFG = 0x02;     // For Appending Channel ID
const byte REG_PIN_CFG = 0x05;      // For Pin Configuration (Analog/Digital)
const byte REG_SEQUENCE_CFG = 0x10; // For Sequence Mode & Start
const byte REG_AUTO_SEQ_SEL = 0x12; // For Selecting Channels to Scan

// --- Constants ---
const float V_REF = 3.3; // Reference Voltage (AVDD)

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Wire.begin(); // Join I2C bus

    Serial.println("Initializing ADS7128...");

    // 1. Reset the Device to ensure clean state
    // Write 1 to Bit 0 (RST) of GENERAL_CFG
    writeRegister(REG_GENERAL_CFG, 0x01);
    delay(10); // Wait for reset to complete

    // 2. Configure Data Format to Append Channel ID
    // We write 0x10 (0001 0000b) to DATA_CFG.
    // Bits 5-4 = 01b enables "4-bit Channel ID" appended to data.
    // This allows us to confirm exactly which channel we are reading.
    writeRegister(REG_DATA_CFG, 0x10);

    // 3. Configure All Pins as Analog Inputs
    // Write 0x00 to PIN_CFG (0 = Analog Input, 1 = GPIO)
    writeRegister(REG_PIN_CFG, 0x00);

    // 4. Select All Channels for Auto-Sequencing
    // Write 0xFF (1111 1111b) to AUTO_SEQ_CH_SEL.
    // This enables AIN0 through AIN7 for the sequence.
    writeRegister(REG_AUTO_SEQ_SEL, 0xFF);

    // 5. Start Auto-Sequence Mode
    // Write 0x11 (0001 0001b) to SEQUENCE_CFG.
    // Bit 4 = 1 (SEQ_START: Start sequencing)
    // Bits 1-0 = 01 (SEQ_MODE: Auto-sequence mode)
    writeRegister(REG_SEQUENCE_CFG, 0x11);

    Serial.println("ADS7128 Configured. Scanning channels...");
}

void loop()
{
    Serial.println("--- Scan Start ---");

    // We loop 8 times to read all 8 enabled channels
    for (int i = 0; i < 8; i++)
    {
        readAndPrintChannel();
        delay(10); // Small delay between channel reads
    }

    Serial.println("--- Scan End ---\n");
    delay(1000); // Wait 1 second before next full scan
}

// --- Helper Functions ---

void writeRegister(byte reg, byte val)
{
    Wire.beginTransmission(ADS7128_ADDR);
    Wire.write(OPCODE_WRITE); // [cite: 1382] Opcode 0x08 required for writing
    Wire.write(reg);          // Register Address
    Wire.write(val);          // Value to write
    Wire.endTransmission();
}

void readAndPrintChannel()
{
    // Request 2 bytes: Data (12-bit) + Channel ID (4-bit)
    Wire.requestFrom(ADS7128_ADDR, 2);

    if (Wire.available() == 2)
    {
        byte msb = Wire.read();
        byte lsb = Wire.read();

        // --- Parse Data ---
        // MSB Byte: [D11 D10 D9  D8  D7  D6  D5  D4]
        // LSB Byte: [D3  D2  D1  D0  ID3 ID2 ID1 ID0]

        // 1. Extract 12-bit ADC Result
        // Shift MSB left by 4, Shift LSB right by 4, combine them.
        uint16_t rawCode = ((msb << 4) | (lsb >> 4));

        // 2. Extract Channel ID
        // Mask the lower 4 bits of the LSB byte
        byte channelID = lsb & 0x0F;

        // 3. Convert to Voltage
        float voltage = (rawCode * V_REF) / 4096.0;

        // 4. Print Result
        Serial.print("CH");
        Serial.print(channelID);
        Serial.print(": ");
        Serial.print(voltage, 3);
        Serial.print("V (Raw: ");
        Serial.print(rawCode);
        Serial.println(")");
    }
    else
    {
        Serial.println("I2C Error: No Data");
    }
}