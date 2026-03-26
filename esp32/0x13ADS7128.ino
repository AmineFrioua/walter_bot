#include <Wire.h>

#define ADS7128_ADDR_1 0x13
#define REF_VOLTAGE 3.3f

// Voltage divider configurations for each channel
// Ratio = (R1 + R2) / R2
#define AIN0_RATIO 1.0 // M1_SOA_ADC - No divider
#define AIN1_RATIO ((100.0 + 10.0) / 10.0) // M2_P1_Vsense_ADC: 11.0
#define AIN2_RATIO ((100.0 + 10.0) / 10.0) // M2_P2_Vsense_ADC: 11.0
#define AIN3_RATIO ((100.0 + 10.0) / 10.0) // M2_P3_Vsense_ADC: 11.0
#define AIN4_RATIO ((100.0 + 10.0) / 10.0) // M1_P3_Vsense_ADC: 11.0
#define AIN5_RATIO 1.0 // M2_SOC_ADC - No divider
#define AIN6_RATIO 1.0 // M2_SOB_ADC - No divider
#define AIN7_RATIO 1.0 // M2_SOA_ADC - No divider

// Register definitions
#define REG_SYSTEM_STATUS 0x00   // Device status and alerts
#define REG_GENERAL_CFG 0x01     // Statistics enable, calibration settings
#define REG_DATA_CFG 0x02        // Data output format control
#define REG_OSR_CFG 0x03         // Oversampling ratio (1x to 128x)
#define REG_OPMODE_CFG 0x04      // Operating mode (manual/autonomous)
#define REG_PIN_CFG 0x05         // GPIO pin configuration
#define REG_GPIO_CFG 0x07        // GPIO configuration
#define REG_GPO_VALUE 0x09       // GPIO output values
#define REG_GPO_DRIVE_CFG 0x0B   // GPIO drive configuration
#define REG_SEQUENCE_CFG 0x10    // Auto-sequence mode control
#define REG_CHANNEL_SEL 0x11     // Manual channel selection
#define REG_AUTO_SEQ_CH_SEL 0x12 // Auto-sequence channel enable bits

#define REG_RECENT_CH0_LSB 0x20 // AIN0 most recent - lower byte
#define REG_RECENT_CH0_MSB 0x21 // AIN0 most recent - upper byte
#define REG_RECENT_CH1_LSB 0x22 // AIN1 most recent - lower byte
#define REG_RECENT_CH1_MSB 0x23 // AIN1 most recent - upper byte
#define REG_RECENT_CH2_LSB 0x24 // AIN2 most recent - lower byte
#define REG_RECENT_CH2_MSB 0x25 // AIN2 most recent - upper byte
#define REG_RECENT_CH3_LSB 0x26 // AIN3 most recent - lower byte
#define REG_RECENT_CH3_MSB 0x27 // AIN3 most recent - upper byte
#define REG_RECENT_CH4_LSB 0x28 // AIN4 most recent - lower byte
#define REG_RECENT_CH4_MSB 0x29 // AIN4 most recent - upper byte
#define REG_RECENT_CH5_LSB 0x2A // AIN5 most recent - lower byte
#define REG_RECENT_CH5_MSB 0x2B // AIN5 most recent - upper byte
#define REG_RECENT_CH6_LSB 0x2C // AIN6 most recent - lower byte
#define REG_RECENT_CH6_MSB 0x2D // AIN6 most recent - upper byte
#define REG_RECENT_CH7_LSB 0x2E // AIN7 most recent - lower byte
#define REG_RECENT_CH7_MSB 0x2F // AIN7 most recent - upper byte

#define NUM_CHANNELS 8

// Structure to store channel configuration
struct ChannelConfig
{
    const char *name;
    float ratio;
    const char *type; // "current" or "voltage"
};

// Channel configurations
ChannelConfig channels[NUM_CHANNELS] = {
    {"M1_SOA", AIN0_RATIO, "current"},     // AIN0
    {"M2_P1_Vsense", AIN1_RATIO, "voltage"},  // AIN1
    {"M2_P2_Vsense", AIN2_RATIO, "voltage"},  // AIN2
    {"M2_P3_Vsense", AIN3_RATIO, "voltage"},  // AIN3
    {"M1_P3_Vsense", AIN4_RATIO, "voltage"},  // AIN4
    {"M2_SOC", AIN5_RATIO, "current"},     // AIN5
    {"M2_SOB", AIN6_RATIO, "current"},     // AIN6
    {"M2_SOA", AIN7_RATIO, "current"}      // AIN7
};

bool setupInfoPrinted = false;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    delay(100);
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

uint16_t readChannel(uint8_t addr, uint8_t channel)
{
    // Read LSB and MSB for the specified channel
    // Each channel has 2 registers: LSB at base+0, MSB at base+1
    uint8_t lsbReg = REG_RECENT_CH0_LSB + (channel * 2);
    uint8_t msbReg = REG_RECENT_CH0_MSB + (channel * 2);

    // Read LSB first, then MSB
    uint8_t lsb = readRegister(addr, lsbReg);
    uint8_t msb = readRegister(addr, msbReg);

    // Combine into 12-bit value
    // MSB contains bits [11:4], LSB contains bits [3:0] in upper nibble
    uint16_t value = ((uint16_t)msb << 8) | lsb;
    return value & 0x0FFF; // Mask to 12 bits (0-4095)
}

float adcToVoltage(uint16_t adcValue)
{
    // Convert ADC value to voltage at ADC input
    // Formula from TI ADS7128 datasheet: V_IN = (ADC_CODE / 4095) × V_REF
    return (adcValue / 4095.0) * REF_VOLTAGE;
}

float correctForVoltageDivider(float adcVoltage, float ratio)
{
    // Correct for voltage divider to get actual input voltage
    // V_INPUT = V_ADC × (R1 + R2) / R2
    return adcVoltage * ratio;
}

float voltageToCurrentACS712(float sensorVoltage)
{
    // Convert sensor voltage to current using ACS712 formula
    // I = 20 × (Vout - 1.65)
    // This assumes ACS712-05B with 185mV/A sensitivity
    return 20.0 * (sensorVoltage - 1.65);
}

void initializeADS7128()
{
    if (!setupInfoPrinted)
    {
        Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
        Serial.println("║         ADS7128 8-Channel ADC Initialization (0x13)      ║");
        Serial.println("╚═══════════════════════════════════════════════════════════╝");

        // Configure the ADC
        Serial.println("  → Configuring ADC settings...");

        // Clear GENERAL_CFG - normal operation (no reset, no statistics)
        writeRegister(ADS7128_ADDR_1, REG_GENERAL_CFG, 0x00);
        delay(50);

        // Set OSR to 128x for better accuracy and settling
        writeRegister(ADS7128_ADDR_1, REG_OSR_CFG, 0x07);
        delay(50);

        // Enable all channels (AIN0-AIN7) for auto-sequence FIRST
        writeRegister(ADS7128_ADDR_1, REG_AUTO_SEQ_CH_SEL, 0xFF);
        delay(50);

        // Configure auto-sequence mode
        writeRegister(ADS7128_ADDR_1, REG_SEQUENCE_CFG, 0x01);
        delay(50);

        // Set auto-sequence mode (start conversions)
        writeRegister(ADS7128_ADDR_1, REG_OPMODE_CFG, 0x01);
        delay(100);

        // Wait for multiple conversion cycles to complete and stabilize
        delay(300);

        // Check system status
        uint8_t status = readRegister(ADS7128_ADDR_1, REG_SYSTEM_STATUS);
        Serial.print("ADS7128 Status: 0x");
        Serial.println(status, HEX);

        Serial.println("\n✓ ADS7128 initialized successfully");
        Serial.println("✓ All 8 channels enabled");
        Serial.println("✓ 128x oversampling for accuracy");

        Serial.println("\n┌───────────────────────────────────────────────────────────┐");
        Serial.println("│              Channel Configuration                        │");
        Serial.println("├─────┬──────────────────────┬──────────┬──────────────────┤");
        Serial.println("│ CH  │ Name                 │  Ratio   │  Type            │");
        Serial.println("├─────┼──────────────────────┼──────────┼──────────────────┤");

        Serial.println("│ AIN0│ M1_SOA               │  1.00x   │  Current sensor  │");
        Serial.println("│ AIN1│ M2_P1_Vsense         │ 11.00x   │  Voltage sense   │");
        Serial.println("│ AIN2│ M2_P2_Vsense         │ 11.00x   │  Voltage sense   │");
        Serial.println("│ AIN3│ M2_P3_Vsense         │ 11.00x   │  Voltage sense   │");
        Serial.println("│ AIN4│ M1_P3_Vsense         │ 11.00x   │  Voltage sense   │");
        Serial.println("│ AIN5│ M2_SOC               │  1.00x   │  Current sensor  │");
        Serial.println("│ AIN6│ M2_SOB               │  1.00x   │  Current sensor  │");
        Serial.println("│ AIN7│ M2_SOA               │  1.00x   │  Current sensor  │");
        Serial.println("└─────┴──────────────────────┴──────────┴──────────────────┘\n");

        setupInfoPrinted = true;
    }
}

void printChannelReading(uint8_t channel, uint16_t adcValue, float adcVoltage, float actualValue)
{
    Serial.print("│ AIN");
    Serial.print(channel);
    Serial.print(" │ ");

    // Print channel name (20 chars wide)
    String name = String(channels[channel].name);
    Serial.print(name);
    for (int i = name.length(); i < 20; i++)
    {
        Serial.print(" ");
    }

    // Print ADC raw value
    char buffer[10];
    sprintf(buffer, " %4d │ ", adcValue);
    Serial.print(buffer);

    // Print ADC voltage (at ADC input)
    Serial.print(adcVoltage, 3);
    Serial.print("V │ ");

    // Print actual measurement based on type
    if (strcmp(channels[channel].type, "current") == 0)
    {
        // Current sensor reading
        Serial.print(actualValue, 3);
        Serial.print("A       │");
    }
    else
    {
        // Voltage sensor reading
        Serial.print(actualValue, 3);
        Serial.print("V       │");
    }
    Serial.println();
}

void loop()
{
    // Initialize ADC on first loop
    initializeADS7128();

    Serial.println("\n╔════════════════════════════════════════════════════════════════════╗");
    Serial.println("║              ADS7128 All Channels Reading (0x13)                  ║");
    Serial.println("╚════════════════════════════════════════════════════════════════════╝");
    Serial.println("┌──────┬──────────────────────┬───────┬──────────┬─────────────┐");
    Serial.println("│ CH   │ Name                 │  ADC  │ ADC_V    │ Measurement │");
    Serial.println("├──────┼──────────────────────┼───────┼──────────┼─────────────┤");

    // Read all 8 channels
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
    {
        // Read ADC value
        uint16_t adcValue = readChannel(ADS7128_ADDR_1, ch);

        // Convert to ADC input voltage
        float adcVoltage = adcToVoltage(adcValue);

        // Calculate actual measurement based on channel type
        float actualValue;
        if (strcmp(channels[ch].type, "current") == 0)
        {
            // Current sensor - no divider correction needed
            actualValue = voltageToCurrentACS712(adcVoltage);
        }
        else
        {
            // Voltage sensor - apply divider correction
            actualValue = correctForVoltageDivider(adcVoltage, channels[ch].ratio);
        }

        // Print reading
        printChannelReading(ch, adcValue, adcVoltage, actualValue);
    }

    Serial.println("└──────┴──────────────────────┴───────┴──────────┴─────────────┘");

    delay(10000);
}
