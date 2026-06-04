// ======================================================================
//  SRAL IDF RTK Rover - T-Display S3 + ZED-F9P
//  Version stable RTK Float - Interface terrain finale
// ======================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>
#include "NTRIPClient.h"
#include "Secret.h"
#include "pin_config.h"

#define UART_RX_PIN  11
#define UART_TX_PIN  10
const unsigned long GNSS_BAUD = 38400;

const int   NTRIP_PORT   = 2101;
const char* NTRIP_MNTPNT = "NEAR";

WiFiUDP udp;
const uint16_t UDP_PORT = 10110;

HardwareSerial GNSS(1);
TFT_eSPI tft = TFT_eSPI();
NTRIPClient ntrip;

int    satsUsed     = 0;
float  hdop         = 99.9f;
String rtkStatus    = "Attente RTK";
int    batteryPct   = 0;
float  batteryVolt  = 0.0f;
int    rtcmCount    = 0;
unsigned long lastRTCMPrint = 0;
bool   blinkSync    = false;

unsigned long lastGGA      = 0;
unsigned long lastDisplay  = 0;
unsigned long lastBlink    = 0;

String lastGGAString = "";   // ← Déclaration ajoutée ici

const unsigned long GGA_INTERVAL    = 8000;
const unsigned long DISPLAY_REFRESH = 1000;

// Batterie
int getBatteryPercent(float v) {
  int mv = (int)(v * 1000);
  if (mv >= 4200) return 100;
  else if (mv >= 4100) return 92;
  else if (mv >= 4000) return 80;
  else if (mv >= 3900) return 70;
  else if (mv >= 3800) return 60;
  else if (mv >= 3750) return 50;
  else if (mv >= 3650) return 35;
  else if (mv >= 3550) return 20;
  else if (mv >= 3400) return 10;
  return 0;
}

// ====================== CONFIG UART1 ======================
const uint8_t ubx_prt[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF9,0x2E};

const uint8_t ubx_rate[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};

const uint8_t ubx_nmea_gga[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x2A};
const uint8_t ubx_nmea_gsa[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x2C};
const uint8_t ubx_nmea_gsv[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x2D};
const uint8_t ubx_nmea_rmc[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x2E};
const uint8_t ubx_nmea_vtg[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x2F};

const uint8_t ubx_rtk_rover[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x00,0x01,0x00,0x03,0x00,0x01,0x00,0x01,0x00,0x2C,0xD2};
const uint8_t ubx_sbas_on[]   = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x2A,0xD0};

const uint8_t ubx_save[]       = {0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x17,0x79};
const uint8_t ubx_cold_reset[] = {0xB5,0x62,0x06,0x04,0x04,0x00,0xFF,0xFF,0x01,0x00,0x0F,0x4A};

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(0x0841);

  drawBootScreen();

  GNSS.begin(38400, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(1200);
  while(GNSS.available()) GNSS.read();

  Serial.println("Envoi config UART1...");
  GNSS.write(ubx_prt, sizeof(ubx_prt));     delay(300);
  GNSS.write(ubx_rate, sizeof(ubx_rate));   delay(200);

  GNSS.write(ubx_nmea_gga, sizeof(ubx_nmea_gga)); delay(150);
  GNSS.write(ubx_nmea_gsa, sizeof(ubx_nmea_gsa)); delay(150);
  GNSS.write(ubx_nmea_gsv, sizeof(ubx_nmea_gsv)); delay(150);
  GNSS.write(ubx_nmea_rmc, sizeof(ubx_nmea_rmc)); delay(150);
  GNSS.write(ubx_nmea_vtg, sizeof(ubx_nmea_vtg)); delay(150);

  GNSS.write(ubx_rtk_rover, sizeof(ubx_rtk_rover)); delay(200);
  GNSS.write(ubx_sbas_on, sizeof(ubx_sbas_on));     delay(200);

  GNSS.write(ubx_save, sizeof(ubx_save));       delay(600);
  GNSS.write(ubx_cold_reset, sizeof(ubx_cold_reset)); delay(2500);

  GNSS.end();
  delay(500);
  GNSS.begin(GNSS_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(3000);
  while(GNSS.available()) GNSS.read();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) {
    delay(300); tries++; yield();
  }

  tft.fillRect(0, 160, tft.width(), 100, TFT_BLACK);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(10, 170);
    tft.print("WiFi OK ");
    tft.print(WiFi.localIP());
  }

  tft.setCursor(10, 190);
  tft.setTextColor(TFT_YELLOW);
  tft.print("NTRIP...");
  if (ntrip.reqRaw((char*)NTRIP_SERVER_HOST, (int&)NTRIP_PORT, (char*)NTRIP_MNTPNT,
                   (char*)NTRIP_USER, (char*)NTRIP_PASSWORD)) {
    tft.setTextColor(TFT_GREEN);
    tft.print(" OK");
  } else {
    tft.setTextColor(TFT_RED);
    tft.print(" KO");
  }

  udp.begin(UDP_PORT);
}

// ====================== BOOT SCREEN ======================
void drawBootScreen() {
  tft.fillScreen(0x0841);

  // Liseret tricolore
  tft.fillRect(0, 0, tft.width(), 10, 0x001F);
  tft.fillRect(tft.width()/3, 0, tft.width()/3, 10, TFT_WHITE);
  tft.fillRect(2*tft.width()/3, 0, tft.width()/3, 10, 0xF800);

  // Infos QField sur 4 lignes
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(25, 115);
  tft.print("QFIELD NMEA UDP");
  tft.setCursor(25, 130);
  tft.print("255.255.255.255:10110");
  tft.setCursor(25, 145);
  tft.print("SSID : "); tft.print(WIFI_SSID);
  tft.setCursor(25, 160);
  tft.print("PASS : "); tft.print(WIFI_PASSWORD);
}

// ====================== LOOP ======================
void loop() {
  yield();

  if (millis() - lastGGA >= GGA_INTERVAL) {
    lastGGA = millis();
    String gga = "";
    unsigned long t = millis();
    while (millis() - t < 3000) {
      if (GNSS.available()) {
        char c = GNSS.read();
        gga += c;
        if (c == '\n') {
          if (gga.startsWith("$GNGGA") || gga.startsWith("$GPGGA")) {
            lastGGAString = gga.substring(0, 75);
            Serial.println("GGA reçue : " + gga);

            int q = 0, sats = 0;
            float hd = 99.9;
            int idx = 0, start = 0;
            String parts[15];
            for (int i = 0; i < gga.length() && idx < 15; i++) {
              if (gga[i] == ',') {
                parts[idx++] = gga.substring(start, i);
                start = i + 1;
              }
            }
            if (idx >= 6) q = parts[6].toInt();
            if (idx >= 7) sats = parts[7].toInt();
            if (idx >= 8) hd = parts[8].toFloat();

            if (q >= 1) {
              ntrip.sendGGA((char*)gga.c_str(), (char*)NTRIP_SERVER_HOST, NTRIP_PORT,
                            (char*)NTRIP_USER, (char*)NTRIP_PASSWORD, (char*)NTRIP_MNTPNT);

              satsUsed = sats;
              hdop = hd;

              if      (q == 4) rtkStatus = "RTK Fixe";
              else if (q == 5) rtkStatus = "RTK Float";
              else if (q == 2) rtkStatus = "DGPS";
              else if (q == 1) rtkStatus = "GPS seul";
              else             rtkStatus = "No Fix";
            }
          }
          gga = "";
        }
      }
      yield();
    }
  }

  while (ntrip.available()) {
    char c = ntrip.read();
    GNSS.write(c);
    if (c == 0xD3) {
      rtcmCount++;
      blinkSync = true;
    }
  }

  if (millis() - lastBlink > 350) {
    lastBlink = millis();
    blinkSync = !blinkSync;
  }

  static String buffer = "";
  while (GNSS.available()) {
    char c = GNSS.read();
    buffer += c;
    if (c == '\n') {
      if (buffer.startsWith("$")) {
        udp.beginPacket("255.255.255.255", UDP_PORT);
        udp.print(buffer);
        udp.endPacket();
      }
      buffer = "";
    }
    yield();
  }

  if (millis() - lastDisplay >= DISPLAY_REFRESH) {
    lastDisplay = millis();

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    uint16_t adc = analogRead(PIN_BAT_VOLT);
    batteryVolt = (adc / 4095.0f) * 3.3f * 2.0f;
    batteryPct = getBatteryPercent(batteryVolt);

    tft.fillScreen(0x0841);

    // Liseret tricolore
    tft.fillRect(0, 0, tft.width(), 8, 0x001F);
    tft.fillRect(tft.width()/3, 0, tft.width()/3, 8, TFT_WHITE);
    tft.fillRect(2*tft.width()/3, 0, tft.width()/3, 8, 0xF800);

    int y = 25;

    uint16_t col = (rtkStatus == "RTK Fixe") ? TFT_GREEN : 
                   (rtkStatus == "RTK Float" ? TFT_YELLOW : TFT_RED);
    tft.setTextColor(col);
    tft.setTextSize(2);
    tft.setCursor(35, y);
    tft.print(rtkStatus);

    y += 100;

    // Sats et HDOP
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(40, y);
    tft.print("SATS : "); tft.print(satsUsed);
    y += 28;
    tft.setCursor(40, y);
    tft.print("HDOP : "); tft.print(hdop,1);

    y += 100;

    // Cadre batterie en footer
    tft.drawRoundRect(8, y, tft.width()-16, 32, 8, TFT_DARKGREY);

    int bx = 18;
    tft.drawRect(bx, y+8, 36, 18, TFT_LIGHTGREY);
    tft.fillRect(bx+36, y+12, 5, 10, TFT_LIGHTGREY);
    int fill = map(batteryPct, 0, 100, 0, 32);
    tft.fillRect(bx+2, y+10, fill, 14, batteryPct > 25 ? TFT_GREEN : TFT_RED);

    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(70, y+10);
    tft.print("Batterie ");
    tft.print(batteryPct);
    tft.print("%   ");
    //tft.print(batteryVolt,2);
    //tft.print("V");
  }

  yield();
  delay(5);
}