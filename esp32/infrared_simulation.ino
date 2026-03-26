#include <Wire.h>

const byte ADS7128_ADDR = 0x14;
const byte OPCODE_WRITE = 0x08;

const byte REG_GENERAL_CFG = 0x01;
const byte REG_DATA_CFG = 0x02;
const byte REG_PIN_CFG = 0x05;
const byte REG_SEQUENCE_CFG = 0x10;
const byte REG_AUTO_SEQ_SEL = 0x12;

const float V_REF = 3.3;
const float RESISTOR_RATIO = 2.0;

const float MAX_SAFE_DIST_CM = 5.0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Wire.begin();

    Serial.println("Initializing ADS7128 for Floor Sensor...");

    writeRegister(REG_GENERAL_CFG, 0x01);
    delay(10);

    writeRegister(REG_DATA_CFG, 0x10);

    writeRegister(REG_PIN_CFG, 0x00);

    writeRegister(REG_AUTO_SEQ_SEL, 0x01);

    writeRegister(REG_SEQUENCE_CFG, 0x11);

    Serial.println("System Ready. Open Serial Monitor.");
}

void loop()
{
    // Request 2 bytes: Data (12-bit) + Channel ID (4-bit)
    Wire.requestFrom(ADS7128_ADDR, 2);

    if (Wire.available() == 2)
    {
        byte msb = Wire.read();
        byte lsb = Wire.read();

        uint16_t rawCode = ((msb << 4) | (lsb >> 4));

        byte channelID = lsb & 0x0F;

        if (channelID == 0)
        {
            checkDistance(rawCode);
        }
    }

    delay(100);
}

void checkDistance(uint16_t rawCode)
{
    float pinVoltage = (rawCode * V_REF) / 4096.0;

    float sensorVoltage = pinVoltage * RESISTOR_RATIO;

    float distanceCm = 0;

    if (sensorVoltage > 0.1)
    {
        distanceCm = 13.0 / sensorVoltage;
    }
    else
    {
        distanceCm = 99.0;
    }

    // D. Decision (Print Status)
    Serial.print("Dist: ");
    Serial.print(distanceCm, 1);
    Serial.print(" cm (");
    Serial.print(sensorVoltage, 2);
    Serial.print("V) -> ");

    if (distanceCm > MAX_SAFE_DIST_CM)
    {
        Serial.println("*** STOP *** (Cliff Detected)");
    }
    else
    {
        Serial.println("RUNNING (Ground Safe)");
    }
}

void writeRegister(byte reg, byte val)
{
    Wire.beginTransmission(ADS7128_ADDR);
    Wire.write(OPCODE_WRITE);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}