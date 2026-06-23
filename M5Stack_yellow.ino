#include <M5Unified.h>
#include <WiFi.h>
#include <Wire.h>
#include "M5UnitENV.h"

SHT3X   sht3x;
QMP6988 qmp6988;

// 0=Nivel  1=WiFi  2=Sistema  3=Módulos  4=Sonido
int currentScreen = 0;
const int TOTAL_SCREENS = 5;

// ─── Colores ──────────────────────────────────────────────────
#define C_BG     BLACK
#define C_HDR    0xFEA0
#define C_ACCENT TFT_CYAN
#define C_OK     TFT_GREEN
#define C_WARN   TFT_YELLOW
#define C_BAD    TFT_RED
#define C_GREY   0x8410
#define C_DARK   0x1082

// ─── Botón B: largo(1s)=siguiente, corto=refrescar ────────────
uint32_t btnBDownAt    = 0;
bool     btnBLongFired = false;

// ─── IMU ──────────────────────────────────────────────────────
float ax, ay, az;

// ════ PANTALLA 0: NIVEL ══════════════════════════════════════
#define LVL_CX 65
#define LVL_CY 74
#define LVL_R  48
#define BUB_R   9
float sAx = 0, sAy = 0, sAz = 0;
const float LVL_ALPHA = 0.30f;
int prevBX = -1, prevBY = -1;

// ════ PANTALLA 1: WIFI ═══════════════════════════════════════
int  wifiCount        = 0;
bool wifiScanned      = false;
int  wifiScrollOffset = 0;
const int WIFI_PER_PAGE = 6;

// ════ PANTALLA 2: SISTEMA ════════════════════════════════════
int sistemaView = 0;  // 0=info general  1=detalles chip

// ════ PANTALLA 3: MÓDULOS I2C ════════════════════════════════
// M5StickC Plus2 HAT port: SDA=G0, SCL=G26
#define GROVE_SDA 0
#define GROVE_SCL 26

struct KnownI2C { uint8_t addr; const char* label; };
const KnownI2C KNOWN[] = {
  { 0x44, "SHT3X   ENV.III Temp+Hum" },
  { 0x56, "QMP6988 ENV.III Presion"  },
  { 0x38, "AHT20   ENV.IV  Temp+Hum" },
  { 0x76, "BMP280  Presion/Temp"     },
  { 0x51, "BM8563  RTC"              },
  { 0x28, "Earth   Humedad suelo"    },
  { 0x45, "NCIR    Temp IR"          },
  { 0x68, "MPU6050 IMU"              },
};
const int KNOWN_COUNT = sizeof(KNOWN) / sizeof(KNOWN[0]);

bool    envReady = false, qmpReady = false;
float   envTemp  = 0, envHum  = 0;
float   qmpPres  = 0, qmpAlt  = 0;
uint8_t foundAddrs[16];
char    foundLabels[16][28];
int     foundCount = 0;
int     moduleView = 0;  // 0=Temp+Hum  1=Presion+Altitud

// ════ PANTALLA 4: NIVEL SONORO ═══════════════════════════════
#define MIC_SAMPLES 256
#define HIST_W      230
int16_t micBuf[MIC_SAMPLES];
int     soundLevel = 0;
int     soundPeak  = 0;
bool    peakHold   = false;
uint8_t soundHist[HIST_W];

// ════════════════════════════════════════════════════════════
//  UTILIDADES
// ════════════════════════════════════════════════════════════

void drawHeader(const char* title) {
  M5.Lcd.fillScreen(C_BG);
  M5.Lcd.fillRect(0, 0, 240, 18, C_HDR);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, 5);   M5.Lcd.print(title);
  M5.Lcd.setCursor(206, 5); M5.Lcd.printf("%d/%d", currentScreen + 1, TOTAL_SCREENS);
}

void drawFooter(const char* a, const char* b) {
  M5.Lcd.fillRect(0, 122, 240, 13, 0x2104);
  M5.Lcd.setTextColor(WHITE); M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4,   125); M5.Lcd.printf("[A]%s", a);
  M5.Lcd.setCursor(130, 125); M5.Lcd.printf("[B]%s", b);
}

// ════════════════════════════════════════════════════════════
//  PANTALLA 0 – NIVEL DE BURBUJA
// ════════════════════════════════════════════════════════════

void drawLevelStatic() {
  M5.Lcd.drawCircle(LVL_CX, LVL_CY, LVL_R,         C_GREY);
  M5.Lcd.drawCircle(LVL_CX, LVL_CY, LVL_R * 2 / 3, C_GREY);
  M5.Lcd.drawCircle(LVL_CX, LVL_CY, LVL_R / 3,     C_GREY);
  M5.Lcd.drawFastHLine(LVL_CX - LVL_R, LVL_CY, LVL_R * 2, C_GREY);
  M5.Lcd.drawFastVLine(LVL_CX, LVL_CY - LVL_R, LVL_R * 2, C_GREY);
  M5.Lcd.setTextColor(C_GREY); M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(LVL_CX - LVL_R - 6, LVL_CY - 3); M5.Lcd.print("-");
  M5.Lcd.setCursor(LVL_CX + LVL_R + 2, LVL_CY - 3); M5.Lcd.print("+");
  M5.Lcd.setCursor(138, 24); M5.Lcd.print("Eje X (pitch)");
  M5.Lcd.setCursor(138, 64); M5.Lcd.print("Eje Y (roll)");
}

void drawLevelScreen() {
  drawHeader("  NIVEL");
  drawLevelStatic();
  drawFooter("---", "1s=Sig / tap=Refr");
  prevBX = -1; prevBY = -1;
  M5.Imu.getAccel(&sAx, &sAy, &sAz);
}

void updateLevelValues() {
  M5.Imu.getAccel(&ax, &ay, &az);
  sAx = LVL_ALPHA * ax + (1.0f - LVL_ALPHA) * sAx;
  sAy = LVL_ALPHA * ay + (1.0f - LVL_ALPHA) * sAy;
  sAz = LVL_ALPHA * az + (1.0f - LVL_ALPHA) * sAz;

  float angleX = atan2f(sAx, sAz) * 180.0f / M_PI;
  float angleY = atan2f(sAy, sAz) * 180.0f / M_PI;
  float total  = sqrtf(angleX * angleX + angleY * angleY);
  uint16_t col = (total < 3.0f) ? C_OK : (total < 12.0f) ? C_WARN : C_BAD;

  int bx = LVL_CX + constrain((int)(angleY / 45.0f * (LVL_R - BUB_R)), -(LVL_R - BUB_R), (LVL_R - BUB_R));
  int by = LVL_CY + constrain((int)(angleX / 45.0f * (LVL_R - BUB_R)), -(LVL_R - BUB_R), (LVL_R - BUB_R));

  if (prevBX >= 0) { M5.Lcd.fillCircle(prevBX, prevBY, BUB_R, C_BG); drawLevelStatic(); }
  M5.Lcd.fillCircle(bx, by, BUB_R, col);
  M5.Lcd.drawCircle(bx, by, BUB_R, WHITE);
  prevBX = bx; prevBY = by;

  M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(col, C_BG);
  M5.Lcd.setCursor(138, 34); M5.Lcd.printf("%+6.1f", angleX);
  M5.Lcd.setTextSize(1);     M5.Lcd.setCursor(222, 40); M5.Lcd.print(" o");

  M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(col, C_BG);
  M5.Lcd.setCursor(138, 74); M5.Lcd.printf("%+6.1f", angleY);
  M5.Lcd.setTextSize(1);     M5.Lcd.setCursor(222, 80); M5.Lcd.print(" o");

  M5.Lcd.setTextSize(1); M5.Lcd.setCursor(138, 108);
  if (total < 3.0f) { M5.Lcd.setTextColor(C_OK,  C_BG); M5.Lcd.print(" NIVELADO   "); }
  else              { M5.Lcd.setTextColor(C_BAD, C_BG); M5.Lcd.printf(" %.1f gr    ", total); }
}

// ════════════════════════════════════════════════════════════
//  PANTALLA 1 – WIFI SCANNER
// ════════════════════════════════════════════════════════════

void doWifiScan() {
  drawHeader(" WIFI SCANNER");
  M5.Lcd.setTextColor(C_WARN); M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, 34); M5.Lcd.println("Escaneando redes WiFi...");
  M5.Lcd.setCursor(4, 50); M5.Lcd.println("(3-5 segundos)");
  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(200);
  wifiCount = WiFi.scanNetworks();
  wifiScanned = true; wifiScrollOffset = 0;
}

void drawWifiScreen() {
  drawHeader(" WIFI SCANNER");
  M5.Lcd.setTextSize(1);

  if (!wifiScanned) {
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(4, 40); M5.Lcd.println("Pulsa [B] para escanear.");
    M5.Lcd.setTextColor(C_GREY);
    M5.Lcd.setCursor(4, 58); M5.Lcd.println("[A] desplaza la lista.");
    drawFooter("Scroll", "Escanear / 1s=Sig");
    return;
  }

  M5.Lcd.setTextColor(C_ACCENT);
  M5.Lcd.setCursor(4, 21); M5.Lcd.printf("Redes: %d", wifiCount);
  if (wifiCount > WIFI_PER_PAGE) {
    M5.Lcd.setTextColor(C_GREY); M5.Lcd.setCursor(74, 21);
    M5.Lcd.printf("  %d-%d de %d  [A]=bajar",
      wifiScrollOffset + 1, min(wifiScrollOffset + WIFI_PER_PAGE, wifiCount), wifiCount);
  }
  M5.Lcd.drawLine(0, 31, 240, 31, C_GREY);
  M5.Lcd.setTextColor(C_GREY); M5.Lcd.setCursor(4, 33);
  M5.Lcd.print("SSID               RSSI  CH ENC");

  int end = min(wifiScrollOffset + WIFI_PER_PAGE, wifiCount);
  for (int i = wifiScrollOffset; i < end; i++) {
    int rssi = WiFi.RSSI(i);
    uint16_t col = (rssi > -65) ? C_OK : (rssi > -80) ? C_WARN : C_BAD;
    M5.Lcd.setTextColor(col);
    M5.Lcd.setCursor(4, 43 + (i - wifiScrollOffset) * 12);
    String ssid = WiFi.SSID(i);
    if (ssid.length() > 19) ssid = ssid.substring(0, 18) + "~";
    bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    M5.Lcd.printf("%-19s %4d %2d %s", ssid.c_str(), rssi, WiFi.channel(i), open ? "---" : "WPA");
  }
  drawFooter("Scroll", "Reescanear / 1s=Sig");
}

// ════════════════════════════════════════════════════════════
//  PANTALLA 2 – SISTEMA
// ════════════════════════════════════════════════════════════

void drawSystemScreen() {
  drawHeader(" SISTEMA");
  M5.Lcd.setTextSize(1);

  if (sistemaView == 0) {
    // ── Info de uso ──
    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 22);   M5.Lcd.print("CPU:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(32, 22);  M5.Lcd.printf("%d MHz", ESP.getCpuFreqMHz());
    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(108, 22); M5.Lcd.print("Flash:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(148, 22); M5.Lcd.printf("%d KB", ESP.getFlashChipSize() / 1024);
    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 34);   M5.Lcd.print("Heap:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(38, 34);  M5.Lcd.printf("%d B libres", ESP.getFreeHeap());
    M5.Lcd.drawLine(0, 44, 240, 44, C_GREY);

    // Batería
    int  batt     = M5.Power.getBatteryLevel();
    bool charging = M5.Power.isCharging();
    uint16_t bCol = (batt > 50) ? C_OK : (batt > 20) ? C_WARN : C_BAD;

    M5.Lcd.setTextColor(C_HDR); M5.Lcd.setCursor(4, 50); M5.Lcd.print("Bateria:");
    M5.Lcd.setTextColor(bCol);  M5.Lcd.setTextSize(2); M5.Lcd.setCursor(4, 62);
    M5.Lcd.printf("%3d%%", batt);

    // Barra
    M5.Lcd.drawRect(55, 64, 108, 14, WHITE);
    M5.Lcd.fillRect(56, 65, (int)(106.0f * constrain(batt, 0, 100) / 100), 12, bCol);
    M5.Lcd.fillRect(163, 67, 3, 8, WHITE);

    M5.Lcd.setTextSize(1); M5.Lcd.setCursor(170, 69);
    if (charging) {
      M5.Lcd.setTextColor(C_WARN, C_BG); M5.Lcd.print("USB/Carga");
    } else {
      M5.Lcd.setTextColor(C_GREY, C_BG); M5.Lcd.print("Bateria  ");
    }

    // Nota si USB conectado
    if (charging) {
      M5.Lcd.setTextColor(C_GREY); M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(4, 82); M5.Lcd.print("(% puede ser impreciso con USB)");
    }

    M5.Lcd.drawLine(0, 94, 240, 94, C_GREY);
    M5.Lcd.setTextColor(C_HDR); M5.Lcd.setCursor(4, 100); M5.Lcd.print("Uptime:");
    unsigned long s = millis() / 1000;
    M5.Lcd.setTextColor(C_ACCENT); M5.Lcd.setCursor(56, 100);
    M5.Lcd.printf("%02luh %02lum %02lus", s / 3600, (s % 3600) / 60, s % 60);

    drawFooter("Chip info", "Refrescar / 1s=Sig");

  } else {
    // ── Detalles del chip ──
    uint64_t chipId = ESP.getEfuseMac();
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
      (uint8_t)(chipId >> 40), (uint8_t)(chipId >> 32),
      (uint8_t)(chipId >> 24), (uint8_t)(chipId >> 16),
      (uint8_t)(chipId >> 8),  (uint8_t)(chipId));

    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 22); M5.Lcd.print("Chip ID:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(4, 34);
    M5.Lcd.printf("%04X%08X", (uint16_t)(chipId >> 32), (uint32_t)chipId);

    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 50); M5.Lcd.print("MAC WiFi:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(4, 62); M5.Lcd.print(mac);

    M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 78); M5.Lcd.print("Sketch:");
    M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(4, 90);
    M5.Lcd.printf("%d KB usados / %d KB libres",
      ESP.getSketchSize() / 1024, ESP.getFreeSketchSpace() / 1024);

    M5.Lcd.setTextColor(C_GREY); M5.Lcd.setCursor(4, 106);
    M5.Lcd.printf("SDK: %s", ESP.getSdkVersion());

    drawFooter("Info general", "Refrescar / 1s=Sig");
  }
}

// ════════════════════════════════════════════════════════════
//  PANTALLA 3 – MÓDULOS I2C
// ════════════════════════════════════════════════════════════

void doModuleScan() {
  foundCount = 0; envReady = false; qmpReady = false;

  // Escaneo genérico para detectar qué hay conectado
  Wire.begin(GROVE_SDA, GROVE_SCL, 100000);
  delay(50);
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0 && foundCount < 16) {
      foundAddrs[foundCount] = addr;
      bool named = false;
      for (int k = 0; k < KNOWN_COUNT; k++) {
        if (KNOWN[k].addr == addr) { strncpy(foundLabels[foundCount], KNOWN[k].label, 27); named = true; break; }
      }
      if (!named) snprintf(foundLabels[foundCount], 27, "Desconocido  0x%02X", addr);
      foundCount++;
    }
  }

  // Inicializar sensores ENV.III con la librería M5UnitENV
  if (sht3x.begin(&Wire, SHT3X_I2C_ADDR, GROVE_SDA, GROVE_SCL, 400000U))
    envReady = true;

  if (qmp6988.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, GROVE_SDA, GROVE_SCL, 400000U))
    qmpReady = true;

  moduleView = 0;
}

void drawModuleScreen() {
  drawHeader(" MODULOS I2C");
  M5.Lcd.setTextSize(1);

  if (foundCount == 0) {
    M5.Lcd.setTextColor(C_GREY);
    M5.Lcd.setCursor(4, 30); M5.Lcd.println("No se detecto ningun modulo.");
    M5.Lcd.setCursor(4, 46); M5.Lcd.println("Conecta un modulo al puerto");
    M5.Lcd.setCursor(4, 58); M5.Lcd.println("HY2.0  SDA=G0  SCL=G26");
    drawFooter("---", "Escanear / 1s=Sig");
    return;
  }

  if (envReady || qmpReady) {
    M5.Lcd.setTextColor(C_OK); M5.Lcd.setCursor(4, 22);
    M5.Lcd.printf("ENV.III  SHT3X%s", qmpReady ? " + QMP6988" : "");
    M5.Lcd.drawLine(0, 32, 240, 32, C_GREY);
    M5.Lcd.setTextColor(C_GREY);
    if (moduleView == 0) {
      // Temperatura + Humedad
      M5.Lcd.setCursor(4, 38);  M5.Lcd.print("Temperatura:");
      M5.Lcd.setCursor(4, 82);  M5.Lcd.print("Humedad:");
    } else {
      // Presion + Altitud
      M5.Lcd.setCursor(4, 38);  M5.Lcd.print("Presion:");
      M5.Lcd.setCursor(4, 82);  M5.Lcd.print("Altitud aprox:");
    }
    drawFooter(qmpReady ? "T+H / Pres+Alt" : "---", "Reescanear / 1s=Sig");
  } else {
    M5.Lcd.setTextColor(C_ACCENT);
    M5.Lcd.setCursor(4, 22); M5.Lcd.printf("%d dispositivo(s) I2C:", foundCount);
    M5.Lcd.drawLine(0, 32, 240, 32, C_GREY);
    for (int i = 0; i < min(foundCount, 5); i++) {
      M5.Lcd.setTextColor(C_WARN); M5.Lcd.setCursor(4, 36 + i * 14);
      M5.Lcd.printf("0x%02X %s", foundAddrs[i], foundLabels[i]);
    }
    drawFooter("---", "Reescanear / 1s=Sig");
  }
}

void updateModuleValues() {
  if (moduleView == 0) {
    // ── Vista Temperatura + Humedad (SHT3X) ──
    if (!envReady) return;
    if (!sht3x.update()) return;
    envTemp = sht3x.cTemp;
    envHum  = sht3x.humidity;

    uint16_t tCol = (envTemp < 20) ? C_ACCENT : (envTemp < 30) ? C_OK : (envTemp < 38) ? C_WARN : C_BAD;
    M5.Lcd.setTextColor(tCol, C_BG); M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(4, 48); M5.Lcd.printf("%.1f", envTemp);
    M5.Lcd.setTextSize(1); M5.Lcd.print(" C  ");

    M5.Lcd.setTextColor(C_ACCENT, C_BG); M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(4, 90); M5.Lcd.printf("%.1f %%  ", envHum);

  } else {
    // ── Vista Presion + Altitud (QMP6988) ──
    if (!qmpReady) return;
    if (!qmp6988.update()) return;
    qmpPres = qmp6988.pressure / 100.0f;  // Pa → hPa
    qmpAlt  = qmp6988.altitude;

    M5.Lcd.setTextColor(C_ACCENT, C_BG); M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(4, 48); M5.Lcd.printf("%.1f", qmpPres);
    M5.Lcd.setTextSize(1); M5.Lcd.print(" hPa");

    M5.Lcd.setTextColor(C_OK, C_BG); M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(4, 90); M5.Lcd.printf("%.1f m  ", qmpAlt);
  }
}

// ════════════════════════════════════════════════════════════
//  PANTALLA 4 – NIVEL SONORO
// ════════════════════════════════════════════════════════════

void drawSoundScreen() {
  drawHeader(" NIVEL SONORO");
  M5.Lcd.setTextSize(1); M5.Lcd.setTextColor(C_GREY);
  M5.Lcd.setCursor(4, 22); M5.Lcd.print("Nivel actual:");
  M5.Lcd.drawRect(4, 32, 232, 20, C_GREY);
  // Marcas 25/50/75%
  for (int p = 25; p <= 75; p += 25) {
    M5.Lcd.drawFastVLine(4 + p * 232 / 100, 52, 3, C_GREY);
  }
  M5.Lcd.setCursor(4, 78); M5.Lcd.print("Historial:");
  M5.Lcd.drawRect(4, 87, 232, 32, C_GREY);
  memset(soundHist, 0, sizeof(soundHist));
  soundPeak = 0; soundLevel = 0;
  drawFooter(peakHold ? "Peak:ON" : "Peak hold", "1s=Sig / tap=Refr");
}

void updateSoundLevel() {
  if (!M5.Mic.isEnabled()) {
    M5.Lcd.setTextColor(C_BAD, C_BG); M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(4, 55); M5.Lcd.print("Microfono no disponible");
    return;
  }
  if (!M5.Mic.record(micBuf, MIC_SAMPLES, 16000)) return;

  // RMS
  int64_t sum = 0;
  for (int i = 0; i < MIC_SAMPLES; i++) sum += (int64_t)micBuf[i] * micBuf[i];
  soundLevel = constrain((int)(sqrtf(sum / (float)MIC_SAMPLES) / 80.0f), 0, 100);

  if (!peakHold) {
    if (soundLevel > soundPeak) soundPeak = soundLevel;
    else if (soundPeak > 0)     soundPeak--;
  }

  // ── Barra VU ──
  uint16_t col = (soundLevel < 40) ? C_OK : (soundLevel < 70) ? C_WARN : C_BAD;
  int barW = soundLevel * 230 / 100;
  M5.Lcd.fillRect(5, 33, barW,       18, col);
  M5.Lcd.fillRect(5 + barW, 33, 230 - barW, 18, C_DARK);
  if (soundPeak > 0)
    M5.Lcd.drawFastVLine(4 + soundPeak * 232 / 100, 33, 18, WHITE);

  // Porcentaje + etiqueta
  M5.Lcd.setTextColor(col, C_BG); M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(4, 58); M5.Lcd.printf("%3d%%", soundLevel);
  M5.Lcd.setTextSize(1); M5.Lcd.setCursor(44, 62);
  if      (soundLevel < 20) { M5.Lcd.setTextColor(C_OK,   C_BG); M5.Lcd.print("SILENCIO  "); }
  else if (soundLevel < 45) { M5.Lcd.setTextColor(C_OK,   C_BG); M5.Lcd.print("NORMAL    "); }
  else if (soundLevel < 70) { M5.Lcd.setTextColor(C_WARN, C_BG); M5.Lcd.print("ALTO      "); }
  else                      { M5.Lcd.setTextColor(C_BAD,  C_BG); M5.Lcd.print("MUY ALTO!!"); }

  // ── Historial ──
  for (int i = 0; i < HIST_W - 1; i++) soundHist[i] = soundHist[i + 1];
  soundHist[HIST_W - 1] = (uint8_t)soundLevel;
  M5.Lcd.fillRect(5, 88, 230, 30, C_BG);
  for (int i = 0; i < HIST_W; i++) {
    int h = soundHist[i] * 29 / 100;
    if (h > 0) {
      uint16_t c = (soundHist[i] < 40) ? C_OK : (soundHist[i] < 70) ? C_WARN : C_BAD;
      M5.Lcd.drawFastVLine(5 + i, 117 - h, h, c);
    }
  }
}

// ════════════════════════════════════════════════════════════
//  NAVEGACIÓN
// ════════════════════════════════════════════════════════════

void renderScreen() {
  switch (currentScreen) {
    case 0: drawLevelScreen();  break;
    case 1: drawWifiScreen();   break;
    case 2: drawSystemScreen(); break;
    case 3: drawModuleScreen(); break;
    case 4: drawSoundScreen();  break;
  }
}

// B corto = refrescar / acción secundaria
void doRefresh() {
  switch (currentScreen) {
    case 1: doWifiScan(); drawWifiScreen(); break;
    case 2: drawSystemScreen(); break;
    case 3: doModuleScan(); drawModuleScreen(); break;
    case 4:
      peakHold = !peakHold;
      if (!peakHold) soundPeak = 0;
      drawFooter(peakHold ? "Peak:ON" : "Peak hold", "1s=Sig / tap=Refr");
      break;
    default: renderScreen(); break;
  }
}

// A = interacción con la pantalla
void doInteract() {
  switch (currentScreen) {
    case 1: // WiFi: scroll
      if (wifiScanned && wifiCount > WIFI_PER_PAGE) {
        wifiScrollOffset = (wifiScrollOffset + WIFI_PER_PAGE) % wifiCount;
        drawWifiScreen();
      }
      break;
    case 2: // Sistema: toggle vista
      sistemaView = 1 - sistemaView;
      drawSystemScreen();
      break;
    case 3: // Módulos: toggle Temp+Hum / Presion+Altitud
      if (envReady || qmpReady) { moduleView = 1 - moduleView; drawModuleScreen(); }
      break;
    default: break;
  }
}

// ════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════════════════════════

void setup() {
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.setBrightness(120);
  M5.Mic.begin();
  delay(100);
  doModuleScan();
  renderScreen();
}

unsigned long lastLiveUpdate   = 0;
unsigned long lastModuleUpdate = 0;

void loop() {
  M5.update();

  // ── Botón A: interacción ──
  if (M5.BtnA.wasPressed()) doInteract();

  // ── Botón B: largo=siguiente pantalla, corto=refrescar ──
  if (M5.BtnB.wasPressed()) { btnBDownAt = millis(); btnBLongFired = false; }
  if (M5.BtnB.isPressed() && !btnBLongFired && (millis() - btnBDownAt >= 1000)) {
    currentScreen = (currentScreen + 1) % TOTAL_SCREENS;
    renderScreen();
    btnBLongFired = true;
  }
  if (M5.BtnB.wasReleased() && !btnBLongFired) doRefresh();

  // ── Actualizaciones en vivo ──
  if (currentScreen == 0 && millis() - lastLiveUpdate > 50) {
    updateLevelValues();
    lastLiveUpdate = millis();
  }
  if (currentScreen == 4) {
    updateSoundLevel();  // record() ya limita a ~16ms por bloque
  }
  if (currentScreen == 3 && envReady && millis() - lastModuleUpdate > 2000) {
    updateModuleValues();
    lastModuleUpdate = millis();
  }
}
