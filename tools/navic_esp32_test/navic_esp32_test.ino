#include <HardwareSerial.h>

static const int RX2_PIN = 16;
static const int TX2_PIN = 17;

HardwareSerial navic(2);

// SkyTraq command: Configure Output Message Format -> NMEA (Message ID 0x09)
// 0xA0, 0xA1, Length(0x00, 0x03), MsgID(0x09), Mode(0x01=NMEA), Attribute(0x01=Update SRAM&Flash), Checksum(0x09), 0x0D, 0x0A
const uint8_t CMD_ENABLE_NMEA[] = {0xA0, 0xA1, 0x00, 0x03, 0x09, 0x01, 0x01, 0x09, 0x0D, 0x0A};

// SkyTraq command: Query Software Version (Message ID 0x02)
const uint8_t CMD_QUERY_VERSION[] = {0xA0, 0xA1, 0x00, 0x02, 0x02, 0x01, 0x03, 0x0D, 0x0A};

char lineBuf[128];
int lineIdx = 0;
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n==========================================");
  Serial.println("   SkyTraq NavIC Wake-up & NMEA Enabler   ");
  Serial.println("==========================================");
  Serial.println("Configuring UART2 on GPIO 16 (RX) and GPIO 17 (TX) at 115200 baud...");

  navic.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // Send Wake-up / NMEA commands to SkyTraq
  Serial.println("Sending SkyTraq NMEA activation commands...");
  navic.write(CMD_ENABLE_NMEA, sizeof(CMD_ENABLE_NMEA));
  delay(100);
  navic.write(CMD_QUERY_VERSION, sizeof(CMD_QUERY_VERSION));
  delay(100);
  Serial.println("Listening for response...\n");
}

double convertToDecimalDegrees(const char* raw, char direction) {
  double val = atof(raw);
  int degrees = (int)(val / 100);
  double minutes = val - (degrees * 100);
  double dec = degrees + (minutes / 60.0);
  if (direction == 'S' || direction == 'W') dec = -dec;
  return dec;
}

void parseGGA(char* sentence) {
  char* tokens[15];
  int tokenCount = 0;
  char* ptr = sentence;

  while (*ptr && tokenCount < 15) {
    tokens[tokenCount++] = ptr;
    while (*ptr && *ptr != ',') ptr++;
    if (*ptr == ',') {
      *ptr = '\0';
      ptr++;
    }
  }

  if (tokenCount >= 10) {
    int fixQuality = atoi(tokens[6]);
    int satellites = atoi(tokens[7]);

    if (fixQuality > 0 && strlen(tokens[2]) > 0 && strlen(tokens[4]) > 0) {
      double lat = convertToDecimalDegrees(tokens[2], tokens[3][0]);
      double lon = convertToDecimalDegrees(tokens[4], tokens[5][0]);
      float altitude = atof(tokens[9]);

      Serial.println("\n******************************************");
      Serial.println(">>>     [3D SATELLITE FIX ACQUIRED]    <<<");
      Serial.println("******************************************");
      Serial.printf("Satellites  : %d\n", satellites);
      Serial.printf("Latitude    : %.6f deg\n", lat);
      Serial.printf("Longitude   : %.6f deg\n", lon);
      Serial.printf("Altitude    : %.2f m\n", altitude);
      Serial.printf("Google Maps : https://maps.google.com/?q=%.6f,%.6f\n", lat, lon);
      Serial.println("******************************************\n");
    } else {
      static unsigned long lastSearchPrint = 0;
      if (millis() - lastSearchPrint > 2000) {
        lastSearchPrint = millis();
        Serial.printf("[SkyTraq Searching] Satellites: %d\n", satellites);
      }
    }
  }
}

void loop() {
  while (navic.available() > 0) {
    uint8_t b = navic.read();

    // If ASCII printable character
    if (b >= 32 && b <= 126) {
      if (lineIdx < (int)(sizeof(lineBuf) - 1)) {
        lineBuf[lineIdx++] = (char)b;
      }
    } else if (b == '\n' || b == '\r') {
      if (lineIdx > 0) {
        lineBuf[lineIdx] = '\0';
        Serial.print("[NMEA] ");
        Serial.println(lineBuf);
        if (strstr(lineBuf, "GGA") != NULL) {
          parseGGA(lineBuf);
        }
        lineIdx = 0;
      }
    } else {
      // Print raw hex if binary packet
      Serial.printf("[HEX: 0x%02X] ", b);
    }
  }

  // Resend NMEA command every 3 seconds if no data
  if (millis() - lastSend > 3000) {
    lastSend = millis();
    Serial.println("[Heartbeat] Sending NMEA output enable command to SkyTraq...");
    navic.write(CMD_ENABLE_NMEA, sizeof(CMD_ENABLE_NMEA));
  }
}
