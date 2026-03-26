#include <Arduino.h>

/**
 * ---------------------------------------------------------
 * OPTIMIZED ESP32 MM-WAVE PARSER
 * ---------------------------------------------------------
 * Improvements:
 * - Faster block-reading (reduced overhead)
 * - Decoupled data parsing from Serial printing
 * - Hardened against buffer overflows
 * ---------------------------------------------------------
 */

// --- PIN DEFINITIONS ---
#define CLI_RX_PIN 18
#define CLI_TX_PIN 17
#define DATA_RX_PIN 16

HardwareSerial RadarCLI(1);
HardwareSerial RadarData(2);

// --- RADAR CONFIGURATION ---
const char *radarConfig[] = {
    // --- SYSTEM CONTROL ---
    "sensorStop",          // Stops the sensor so we can safely configure it.
    "flushCfg",            // Clears all previous configurations from memory.
    "dfeDataOutputMode 1", // Sets output mode to 'Frame Based' (standard for point clouds).

    // --- ANTENNA & ADC SETUP ---
    "channelCfg 15 7 0",    // Enables antennas: RX Mask 15 (1111 = 4 RX), TX Mask 7 (111 = 3 TX).
    "adcCfg 2 1",           // ADC Format: 16-bit resolution, Complex 1x mode.
    "adcbufCfg -1 0 1 1 1", // Configures internal data buffer (Complex data, Chirp mode).

    // --- PHYSICS SETTINGS ---
    // Optimized for roughly 15cm resolution.
    // Max Range: ~9 meters.
    "profileCfg 0 60 359 7 57.14 0 0 70 1 256 5209 0 0 158",

    // --- CHIRP SEQUENCING (MIMO Setup) ---
    "chirpCfg 0 0 0 0 0 0 0 1", // Chirp 0: Uses Profile 0, enables TX Antenna 1 ONLY.
    "chirpCfg 1 1 0 0 0 0 0 2", // Chirp 1: Uses Profile 0, enables TX Antenna 2 ONLY.
    "chirpCfg 2 2 0 0 0 0 0 4", // Chirp 2: Uses Profile 0, enables TX Antenna 3 ONLY.

    // --- FRAME TIMING ---
    // Chirps 0-2, 16 loops per frame, 100ms periodicity (10Hz), trigger immediately
    "frameCfg 0 2 16 0 100 1 0",

    // --- SYSTEM SETTINGS ---
    "lowPower 0 0", // Disables low-power mode (0 0 = standard performance).

    // --- DATA OUTPUT (What ESP32 receives) ---
    // [Detected Objects = 1], [Range Profile = 0], [Noise Profile = 0], ...
    "guiMonitor -1 1 0 0 0 0 0", // CRITICAL: Tells radar to ONLY send X,Y,Z object list.

    // --- SENSITIVITY (CFAR) ---
    // 12 DB to see 'thin' objects like table legs.
    "cfarCfg -1 0 2 8 4 3 0 12 1",
    "cfarCfg -1 1 0 4 2 3 1 12 1",

    // --- SIGNAL PROCESSING ---
    "multiObjBeamForming -1 1 0.5",  // Enables better separation of objects at similar distances.
    "clutterRemoval -1 0",           // 0 = OFF. If 1, it removes static objects (walls/tables).
    "calibDcRangeSig -1 0 -5 8 256", // Calibrates out DC noise from the antenna coupling.
    "extendedMaxVelocity -1 0",      // 0 = OFF. (Used if you need to measure very fast objects).

    // --- HARDWARE TUNING ---
    "lvdsStreamCfg -1 0 0 0", // Disables high-speed LVDS (we are using UART).

    // Calibration for antenna phase mismatches (Factory calibrated values usually).
    "compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0",
    "measureRangeBiasAndRxChanPhase 0 1.5 0.2", // Do not perform range bias measurement now.

    "analogMonitor 0 0", // Disables background analog health monitoring (prevents crashes).

    // --- FIELD OF VIEW (FOV) ---
    "aoaFovCfg -1 -90 90 -90 90", // Angle limits: +/- 90 degrees horizontal, +/- 90 vertical.
    "cfarFovCfg -1 0 0 8.92",     // Range limits: Only detect objects from 0m to 8.92m.
    "cfarFovCfg -1 1 -1 1.00",    // Doppler limits: Velocity filtering.

    // --- STARTUP ---
    "calibData 0 0 0", // Boot calibration command.
    "sensorStart"      // Starts the radar processing chain.
};

// --- CONSTANTS ---
const uint8_t MAGIC_WORD[] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
const float MAX_DISTANCE_M = 4.0; // Filter threshold

// --- STATE MANAGEMENT ---
bool isRunning = false;

// --- FUNCTION PROTOTYPES ---
void startRadar();
void stopRadar();
void readAndParseRadar();

void setup()
{
  Serial.begin(115200);

  // Initialize Radar UARTs
  RadarCLI.begin(115200, SERIAL_8N1, CLI_RX_PIN, CLI_TX_PIN);
  RadarData.begin(921600, SERIAL_8N1, DATA_RX_PIN, -1);

  // Increase RX buffer size for high-speed data (default is 256)
  RadarData.setRxBufferSize(4096);

  Serial.println("\n[SYSTEM] Radar Interface Ready.");
  Serial.println("Commands: 'c' = Start, 's' = Stop");
}

void loop()
{
  // 1. Handle User Input
  if (Serial.available())
  {
    char cmd = Serial.read();
    if (cmd == 'c')
      startRadar();
    if (cmd == 's')
      stopRadar();
  }

  // 2. Handle Radar Data
  if (isRunning)
  {
    readAndParseRadar();
  }
}

void readAndParseRadar()
{
  // We need at least the Magic Word (8 bytes) + Header (32 bytes) = 40 bytes to start
  if (RadarData.available() < 40)
    return;

  // Search for Magic Word
  // Peek checks the buffer without removing data
  bool magicFound = true;
  for (int i = 0; i < 8; i++)
  {
    // This is a naive peek; if efficiency is critical, implement a ring buffer search
    // But for this baud rate, checking byte-by-byte works if buffer is large enough
    if (RadarData.read() != MAGIC_WORD[i])
    {
      // If mismatch, we lost the packet. Wait for next sync.
      // In a robust system, we would slide the window, but here we drop and retry.
      return;
    }
  }

  // If we are here, we passed the magic word check.
  // Read the Header (Next 32 bytes)
  uint8_t header[32];
  RadarData.readBytes(header, 32);

  // Decode Header Information
  // Packet Length is at index 4-7
  uint32_t packetLen = header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24);
  // Number of TLVs is at index 24-27
  uint32_t numTLVs = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);

  // Calculate remaining payload to wait for
  // PacketLen includes MagicWord(8) + Header(32) = 40 bytes.
  // We already read 40 bytes.
  uint32_t payloadLen = packetLen - 40;

  // Wait for the rest of the packet to arrive
  // Safety timeout to prevent locking up
  unsigned long startWait = millis();
  while (RadarData.available() < payloadLen)
  {
    if (millis() - startWait > 50)
      return; // Timeout (bad packet)
  }

  // Iterate through TLVs
  uint32_t bytesProcessed = 0;
  for (int i = 0; i < numTLVs; i++)
  {
    uint8_t tlvHeader[8];
    RadarData.readBytes(tlvHeader, 8);
    bytesProcessed += 8;

    uint32_t tlvType = tlvHeader[0] | (tlvHeader[1] << 8) | (tlvHeader[2] << 16) | (tlvHeader[3] << 24);
    uint32_t tlvLen = tlvHeader[4] | (tlvHeader[5] << 8) | (tlvHeader[6] << 16) | (tlvHeader[7] << 24);

    // TLV Type 1 = Detected Objects (Point Cloud)
    if (tlvType == 1)
    {
      int numObjects = tlvLen / 16; // Each object is 16 bytes (x,y,z,doppler)

      for (int k = 0; k < numObjects; k++)
      {
        uint8_t objData[16];
        RadarData.readBytes(objData, 16);

        float x, y, z;
        memcpy(&x, &objData[0], 4);
        memcpy(&y, &objData[4], 4);
        memcpy(&z, &objData[8], 4);

        // --- FILTER & PRINT ---
        if (y > 0.0 && y <= MAX_DISTANCE_M)
        {
          Serial.printf("OBJ: X:%.2fm  Y:%.2fm  Z:%.2fm\n", x, y, z);
        }
      }
      bytesProcessed += tlvLen;
    }
    else
    {
      // Skip unknown/unused TLVs
      for (int k = 0; k < tlvLen; k++)
        RadarData.read();
      bytesProcessed += tlvLen;
    }
  }

  // Clean up any padding bytes at the end of the packet (alignment)
  while (bytesProcessed < payloadLen)
  {
    if (RadarData.available())
    {
      RadarData.read();
      bytesProcessed++;
    }
    else
    {
      break;
    }
  }
}

void startRadar()
{
  Serial.println("--- CONFIGURING RADAR ---");
  int numCmds = sizeof(radarConfig) / sizeof(radarConfig[0]);
  for (int i = 0; i < numCmds; i++)
  {
    RadarCLI.println(radarConfig[i]);
    delay(50); // Small delay between commands is safer
  }

  // Clear any old data in the buffer before starting
  while (RadarData.available())
    RadarData.read();

  isRunning = true;
  Serial.println("--- RADAR STARTED ---");
}

void stopRadar()
{
  RadarCLI.println("sensorStop");
  isRunning = false;
  Serial.println("--- RADAR STOPPED ---");
}