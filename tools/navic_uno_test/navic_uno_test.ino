#include <SoftwareSerial.h>

// NavIC TX -> Arduino Pin 4
// NavIC RX -> Arduino Pin 3
static const int RX_PIN = 4;
static const int TX_PIN = 3;

// Test baud rate at 115200 (common for N-GS-01 and high-speed NavIC modules)
static const uint32_t GPS_BAUD = 115200;

SoftwareSerial navic(RX_PIN, TX_PIN);

char nmeaBuffer[128];
int bufferIndex = 0;
unsigned long lastCharTime = 0;
unsigned long lastStatusTime = 0;
unsigned long totalCharsReceived = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { ; }
  
  navic.begin(GPS_BAUD);

  Serial.println(F("\n========================================"));
  Serial.println(F("   NavIC / GNSS Real-Time Monitor       "));
  Serial.println(F("========================================"));
  Serial.println(F("Arduino Pin 4 (RX) <- NavIC TX"));
  Serial.println(F("Arduino Pin 3 (TX) -> NavIC RX"));
  Serial.print(F("Baud Rate: "));
  Serial.println(GPS_BAUD);
  Serial.println(F("Listening for NMEA sentences...\n"));
}

double convertToDecimalDegrees(const char* raw, char direction) {
  double val = atof(raw);
  int degrees = (int)(val / 100);
  double minutes = val - (degrees * 100);
  double dec = degrees + (minutes / 60.0);
  if (direction == 'S' || direction == 'W') {
    dec = -dec;
  }
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

      Serial.println(F("\n>>> [3D FIX ACQUIRED] <<<"));
      Serial.print(F("Fix Mode    : "));
      if (fixQuality == 1) Serial.println(F("GPS Fix"));
      else if (fixQuality == 2) Serial.println(F("DGPS Fix"));
      else if (fixQuality == 4) Serial.println(F("NavIC / RTK Fix"));
      else { Serial.print(F("Fix Type ")); Serial.println(fixQuality); }

      Serial.print(F("Satellites  : ")); Serial.println(satellites);
      Serial.print(F("Latitude    : ")); Serial.print(lat, 6); Serial.println(F(" deg"));
      Serial.print(F("Longitude   : ")); Serial.print(lon, 6); Serial.println(F(" deg"));
      Serial.print(F("Altitude    : ")); Serial.print(altitude, 2); Serial.println(F(" m"));
      Serial.print(F("Google Maps : https://maps.google.com/?q="));
      Serial.print(lat, 6); Serial.print(F(",")); Serial.println(lon, 6);
      Serial.println(F("----------------------------------------\n"));
    } else {
      Serial.print(F("[Searching Satellites] In fix: "));
      Serial.println(satellites);
    }
  }
}

void loop() {
  while (navic.available() > 0) {
    char c = navic.read();
    totalCharsReceived++;
    lastCharTime = millis();

    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        nmeaBuffer[bufferIndex] = '\0';
        
        // Print raw NMEA line for verification
        Serial.print(F("[RAW] "));
        Serial.println(nmeaBuffer);

        if (strstr(nmeaBuffer, "GGA") != NULL) {
          parseGGA(nmeaBuffer);
        }
        bufferIndex = 0;
      }
    } else {
      if (bufferIndex < (int)(sizeof(nmeaBuffer) - 1)) {
        nmeaBuffer[bufferIndex++] = c;
      }
    }
  }

  // Periodic heartbeat if no characters received
  if (millis() - lastStatusTime > 4000) {
    lastStatusTime = millis();
    if (totalCharsReceived == 0) {
      Serial.println(F("[WAITING] No data received yet. Checking connection on Pin 4..."));
    }
  }
}
