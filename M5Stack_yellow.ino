#include <M5Unified.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include "M5UnitENV.h"
#include "secrets.h"

SHT3X       sht3x;
QMP6988     qmp6988;
M5Canvas    carousel(&M5.Lcd);   // sprite para el panel del carrusel (sin tearing)
M5Canvas    sysList(&M5.Lcd);    // sprite para deslizar la barra de la lista de Sistema
M5Canvas    iconCv(&M5.Lcd);     // sprite del icono HOME (x=0..87, y=22..134): animacion sin parpadeo
Preferences prefs;               // NVS: persiste ajustes de Sistema entre reinicios

// ─── Colores ──────────────────────────────────────────────────
#define C_BG     BLACK
#define C_OK     TFT_GREEN
#define C_WARN   TFT_YELLOW
#define C_BAD    TFT_RED
#define C_GREY   0x8410
#define C_DARK   0x1082
#define C_MENU   0x3000

// C_HDR (primario/cabeceras) y C_ACCENT (secundario) son variables: cambian con el tema.
uint16_t C_HDR    = 0xD700;
uint16_t C_ACCENT = 0xFDA0;

// ─── Temas de color ───────────────────────────────────────────
struct Theme { const char* name; uint16_t hdr; uint16_t accent; };
const Theme THEMES[] = {
  { "Carmesi", 0xD800, 0xFD20 },   // rojo + ambar (original)
  { "Oceano",  0x015F, 0x07FF },   // azul + cian
  { "Bosque",  0x05E0, 0xAFE5 },   // verde
  { "Violeta", 0x780F, 0xFC1F },   // morado + magenta
  { "Solar",   0xFC00, 0xFFE0 },   // naranja + amarillo
};
const int THEME_COUNT = sizeof(THEMES) / sizeof(THEMES[0]);
int theme = 0;

void applyTheme() {
  C_HDR    = THEMES[theme].hdr;
  C_ACCENT = THEMES[theme].accent;
}

// ─── Estado de navegación ─────────────────────────────────────
bool inHome      = true;
int  selectedMenu = 0;   // 0=Sistema 1=Nivel 2=WiFi 3=Sensores
const int TOTAL_MENUS = 4;

const char* MENU_NAMES[] = { "Sistema", "Nivel", "WiFi", "Sensores" };

// ─── Botones: timers para long press ─────────────────────────
const uint32_t LONG_PRESS_MS = 400;   // umbral de pulsacion larga
const int LED_PIN = 19;               // LED rojo del M5StickC Plus2 (activo en HIGH)
uint32_t btnADownAt    = 0;
bool     btnALongFired = false;
uint32_t btnBDownAt    = 0;
bool     btnBLongFired = false;
bool     chordFired    = false;   // A+B simultáneo (ya disparado en esta pulsación)

// ─── Ajustes Sistema ──────────────────────────────────────────
int  brightness    = 120;
bool clickSound    = false;
int  cpuFreq       = 240;     // MHz de CPU; menor = menos consumo
bool sistemaInList = true;    // true: lista de opciones; false: dentro de una opcion
int  sistemaSel    = 0;       // 0=Info 1=Brillo 2=Sonido 3=Frecuencia

// ─── IMU ──────────────────────────────────────────────────────
#define BUB_R   9
float ax, ay, az;
float sAx = 0, sAy = 0, sAz = 0;
const float LVL_ALPHA = 0.25f;   // suavizado del acelerómetro
float levelPitch = 0, levelRoll = 0, levelTotal = 0;   // últimos ángulos del nivel (para enviar por MQTT)

// ─── WiFi ─────────────────────────────────────────────────────
int  wifiCount        = 0;
bool wifiScanned      = false;
int  wifiScrollOffset = 0;
int  wifiSelected     = 0;       // red resaltada en la lista
bool wifiInDetail     = false;   // true: viendo detalle de la red seleccionada
bool wifiInKeyboard   = false;   // true: tecleando la contraseña
const int WIFI_PER_PAGE = 6;
bool wifiSaved[32];              // por red visible: ¿tiene clave guardada en NVS?

// Teclado de contraseña (2 botones): cursor que recorre una rejilla de teclas.
String    pwBuf;                 // contraseña en construcción
int       kbSel       = 0;       // tecla resaltada
String    connSsid;              // SSID objetivo de la conexión / teclado
int       connResult  = 0;       // 0=sin intento, 1=conectado, 2=fallo (de connResultSsid)
String    connResultSsid;
const int WIFI_SAVED_MAX = 24;   // máximo de redes recordadas en NVS

// ─── Jarvis / MQTT ────────────────────────────────────────────
// Envía la telemetría del ENV.III al servidor Jarvis (broker Mosquitto).
// Credenciales en secrets.h (fuera de git). El UID del topic = usuario MQTT (lo exige el ACL).
WiFiClient   mqttNet;
PubSubClient mqtt(mqttNet);
const char* MQTT_HOST      = SECRET_MQTT_HOST;
const int   MQTT_PORT      = SECRET_MQTT_PORT;
const char* MQTT_USER      = SECRET_MQTT_USER;
const char* MQTT_PASS      = SECRET_MQTT_PASS;
const char* DEVICE_UID     = "m5stack";                  // = usuario MQTT
const char* MQTT_CLIENT_ID = "m5stack-plus2";
const char* TOPIC_TELEMETRY = "jarvis/m5stack/telemetry";
const char* TOPIC_STATUS    = "jarvis/m5stack/status";   // online/offline (Last Will)
const uint32_t MQTT_INTERVAL = 10000;   // periodo de publicación (ms) — solo en pantalla Sensores
const uint32_t MQTT_LEVEL_INTERVAL = 50;  // periodo del nivel (ms) = 20 Hz — casi en vivo, solo en pantalla Nivel
const uint32_t MQTT_RETRY_MS = 5000;    // reintento de (re)conexión
uint32_t lastMqttPublish = 0;
uint32_t lastMqttRetry   = 0;

// ─── Módulos I2C ─────────────────────────────────────────────
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

// ─── Timers live update ───────────────────────────────────────
unsigned long lastLiveUpdate   = 0;
unsigned long lastModuleUpdate = 0;
unsigned long lastIconAnim     = 0;   // animacion constante de iconos en HOME

// ─── Animación carrusel HOME ──────────────────────────────────
float    homeAnimOffset = 0.0f;
bool     homeAnimating  = false;
uint32_t homeAnimStart  = 0;
const uint32_t ANIM_MS  = 200;

// ════════════════════════════════════════════════════════════
//  SONIDO DE CLICK
// ════════════════════════════════════════════════════════════

void playClick() {
  if (!clickSound) return;
  M5.Speaker.tone(700, 60);
}

// ════════════════════════════════════════════════════════════
//  ICONOS HOME — dibujados en el sprite iconCv (88×113, volcado a 0,22)
//  Centro local del icono: (44, 56). `yOff` desplaza vertical (carrusel),
//  `ph` es la fase de animación en segundos (movimiento constante).
// ════════════════════════════════════════════════════════════

// Rectángulo rotado (para los dientes del engranaje).
void fillRotRect(float cx, float cy, float w, float h, float ang, uint16_t col) {
  float c = cosf(ang), s = sinf(ang), hw = w / 2, hh = h / 2;
  float dx[4] = { -hw, hw, hw, -hw }, dy[4] = { -hh, -hh, hh, hh };
  float xs[4], ys[4];
  for (int i = 0; i < 4; i++) { xs[i] = cx + dx[i] * c - dy[i] * s; ys[i] = cy + dx[i] * s + dy[i] * c; }
  iconCv.fillTriangle(xs[0], ys[0], xs[1], ys[1], xs[2], ys[2], col);
  iconCv.fillTriangle(xs[0], ys[0], xs[2], ys[2], xs[3], ys[3], col);
}

// Sistema → engranaje que gira.
void drawIconSistema(int yOff, float ph) {
  float cx = 44, cy = 56 + yOff;
  float ang = ph * 1.2f;
  const int teeth = 8;
  for (int i = 0; i < teeth; i++) {
    float a = ang + i * (2.0f * PI / teeth);
    fillRotRect(cx + cosf(a) * 20, cy + sinf(a) * 20, 9, 11, a, C_HDR);
  }
  iconCv.fillCircle(cx, cy, 18, C_HDR);
  iconCv.fillCircle(cx, cy, 12, C_DARK);
  iconCv.fillCircle(cx, cy,  5, C_ACCENT);
}

// Nivel → vial con burbuja que deriva continuamente.
void drawIconNivel(int yOff, float ph) {
  int cx = 44, cy = 56 + yOff;
  iconCv.drawCircle(cx, cy, 22, C_GREY);
  iconCv.drawCircle(cx, cy, 21, C_DARK);
  iconCv.drawCircle(cx, cy, 13, C_GREY);
  iconCv.drawFastHLine(cx - 22, cy, 45, C_GREY);
  iconCv.drawFastVLine(cx, cy - 22, 45, C_GREY);
  int bx = cx + (int)(8.0f * sinf(ph * 1.3f));
  int by = cy + (int)(6.0f * cosf(ph * 0.9f));
  iconCv.fillCircle(bx, by, 7, C_OK);
  iconCv.fillCircle(bx - 2, by - 2, 2, WHITE);   // brillo
}

// WiFi → arcos que emiten señal hacia arriba en bucle.
// Color según estado de conexión: verde conectado, rojo desconectado.
void drawIconWifi(int yOff, float ph) {
  int cx = 44, cy = 70 + yOff;
  uint16_t col = (WiFi.status() == WL_CONNECTED) ? C_OK : C_BAD;
  int active = (int)fmodf(ph * 2.0f, 4.0f);   // 0..3
  for (int k = 0; k < 3; k++) {
    int r = 10 + k * 8;
    iconCv.drawArc(cx, cy, r, r + 2, 215, 325, (active > k) ? col : C_DARK);
  }
  iconCv.fillCircle(cx, cy, 4, col);
}

// Sensores → termómetro con mercurio que sube y baja.
void drawIconSensores(int yOff, float ph) {
  int cx = 44, cy = 56 + yOff;
  iconCv.fillCircle(cx, cy + 18, 9, C_GREY);
  iconCv.fillRoundRect(cx - 4, cy - 20, 8, 38, 4, C_GREY);
  iconCv.fillCircle(cx, cy + 18, 6, C_DARK);
  iconCv.fillRoundRect(cx - 2, cy - 18, 4, 36, 2, C_DARK);
  float lvl = 0.5f + 0.4f * sinf(ph * 1.4f);     // 0.1..0.9
  int top   = (cy + 16) - (int)(30.0f * lvl);
  iconCv.fillCircle(cx, cy + 18, 6, C_BAD);
  iconCv.fillRoundRect(cx - 2, top, 4, (cy + 18) - top, 2, C_BAD);
  for (int i = 0; i < 5; i++) iconCv.drawFastHLine(cx + 7, cy - 14 + i * 7, 4, C_GREY);
}

void drawIconForMenu(int idx, int yOff, float ph) {
  switch (idx % TOTAL_MENUS) {
    case 0: drawIconSistema(yOff, ph);  break;
    case 1: drawIconNivel(yOff, ph);    break;
    case 2: drawIconWifi(yOff, ph);     break;
    case 3: drawIconSensores(yOff, ph); break;
  }
}

// Vuelca el icono activo (animado) al área HOME.
void renderHomeIcon(float ph) {
  iconCv.fillSprite(C_BG);
  drawIconForMenu(selectedMenu, 0, ph);
  iconCv.pushSprite(0, 22);
}

// Vuelca el frame del carrusel: icono actual sale arriba, siguiente entra abajo.
void renderHomeSlide(int nextMenu, float eased, float ph) {
  iconCv.fillSprite(C_BG);
  drawIconForMenu(selectedMenu, -(int)(eased * 113.0f), ph);
  drawIconForMenu(nextMenu,    113 - (int)(eased * 113.0f), ph);
  iconCv.pushSprite(0, 22);
}

// ════════════════════════════════════════════════════════════
//  CARRUSEL MENÚ (panel derecho x=90..239)
// ════════════════════════════════════════════════════════════

// Interpolación lineal entre dos colores RGB565
static inline uint16_t lerpColor565(uint16_t c1, uint16_t c2, float t) {
  int r1=(c1>>11)&0x1F, g1=(c1>>5)&0x3F, b1=c1&0x1F;
  int r2=(c2>>11)&0x1F, g2=(c2>>5)&0x3F, b2=c2&0x1F;
  return (uint16_t)(((r1+(int)((r2-r1)*t))<<11)|((g1+(int)((g2-g1)*t))<<5)|(b1+(int)((b2-b1)*t)));
}

void drawCarouselPanel(float off) {
  // Sprite: 150×113, mapea pantalla x=90..239 y=22..134
  const int PW = 150, CY = 56, SP = 30;

  carousel.fillSprite(C_BG);
  carousel.fillRoundRect(1, CY - 11, PW - 2, 22, 3, C_HDR);

  for (int i = 0; i < TOTAL_MENUS; i++) {
    float rel = (float)(i - selectedMenu) + off;
    if (rel >  TOTAL_MENUS * 0.5f) rel -= TOTAL_MENUS;
    if (rel < -TOTAL_MENUS * 0.5f) rel += TOTAL_MENUS;
    float fy = CY + rel * SP;
    if (fy < 6.0f || fy > 107.0f) continue;
    int   iy   = (int)(fy + 0.5f);
    float dist = fabsf(rel);

    // Color: interpolación continua → sin "pop" al cruzar el centro
    uint16_t col;
    if (dist < 1.0f) {
      // De gris (dist=1) a negro (dist=0). Fuera de la caja: invisible. Dentro: visible.
      col = lerpColor565(C_GREY, 0x0000, 1.0f - dist);
    } else {
      float fade = constrain((dist - 1.0f) / 0.6f, 0.0f, 1.0f);
      col = lerpColor565(C_GREY, C_DARK, fade);
    }
    carousel.setTextSize(2);
    carousel.setTextColor(col);
    int tw = strlen(MENU_NAMES[i]) * 12;
    carousel.setCursor((PW - tw) / 2, iy - 8);
    carousel.print(MENU_NAMES[i]);
  }

  carousel.pushSprite(90, 22);
}

// ════════════════════════════════════════════════════════════
//  HOME SCREEN
// ════════════════════════════════════════════════════════════

void drawHomeScreen() {
  M5.Lcd.fillScreen(C_BG);

  // Barra superior (22 px). Texto centrado verticalmente: (22-16)/2 = 3
  M5.Lcd.fillRect(0, 0, 240, 22, C_HDR);
  M5.Lcd.setTextColor(BLACK); M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(8, 3); M5.Lcd.print("HOME");

  // Batería + porcentaje, alineados a la derecha y a la misma altura que "HOME"
  int  batt     = M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging();
  char pBuf[6]; snprintf(pBuf, sizeof(pBuf), "%d%%", batt);
  int  tw    = (int)strlen(pBuf) * 12;
  int  pX    = 236 - tw;          // texto a la derecha
  int  iconX = pX - 45;           // batería (36 cuerpo + 3 terminal + 6 gap)

  uint16_t bcol = (batt > 50) ? C_OK : (batt > 20) ? C_WARN : C_BAD;
  M5.Lcd.fillRoundRect(iconX,      4, 36, 14, 3, BLACK);   // marco
  M5.Lcd.fillRect    (iconX + 36, 8,  3,  6,    BLACK);   // terminal
  M5.Lcd.fillRoundRect(iconX + 2,  6, 32, 10, 2, C_DARK);  // hueco
  int fw = (int)(32.0f * constrain(batt, 0, 100) / 100);
  if (fw > 0) M5.Lcd.fillRoundRect(iconX + 2, 6, fw, 10, 2, bcol);   // nivel coloreado
  if (charging) {                                         // rayo de carga
    int cxb = iconX + 18;
    M5.Lcd.fillTriangle(cxb + 2, 6,  cxb - 4, 12, cxb, 12, BLACK);
    M5.Lcd.fillTriangle(cxb - 2, 16, cxb + 4, 10, cxb, 10, BLACK);
  }
  M5.Lcd.setTextColor(BLACK); M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(pX, 3); M5.Lcd.print(pBuf);

  // Divisor vertical y contenido
  M5.Lcd.drawFastVLine(88, 22, 113, C_GREY);

  renderHomeIcon(millis() / 1000.0f);
  drawCarouselPanel(0.0f);
}

// ════════════════════════════════════════════════════════════
//  UTILIDADES PANTALLAS
// ════════════════════════════════════════════════════════════

void drawHeader(const char* title) {
  M5.Lcd.fillScreen(C_BG);
  M5.Lcd.fillRect(0, 0, 240, 20, C_HDR);
  M5.Lcd.setTextColor(BLACK); M5.Lcd.setTextSize(2);
  int tw = (int)strlen(title) * 12;   // textSize 2 ≈ 12 px/char
  M5.Lcd.setCursor((240 - tw) / 2, 2);
  M5.Lcd.print(title);
}

// ════════════════════════════════════════════════════════════
//  MENÚ 0 – SISTEMA
// ════════════════════════════════════════════════════════════

// Pasos de brillo: 30, 60, 90, 120, 160, 200, 255
const int BRIGHT_STEPS[] = { 30, 60, 90, 120, 160, 200, 255 };
const int BRIGHT_COUNT   = 7;

// Pasos de frecuencia de CPU (MHz): menor = menos consumo
const int CPU_STEPS[] = { 240, 160, 80 };
const int CPU_COUNT   = 3;

const char* SISTEMA_ITEMS[] = { "Info", "Brillo", "Sonido", "Frecuencia", "Color" };
const int   SISTEMA_COUNT   = 5;

int getBrightIndex() {
  for (int i = 0; i < BRIGHT_COUNT - 1; i++)
    if (brightness <= BRIGHT_STEPS[i]) return i;
  return BRIGHT_COUNT - 1;
}

int getCpuIndex() {
  for (int i = 0; i < CPU_COUNT; i++)
    if (cpuFreq == CPU_STEPS[i]) return i;
  return 0;
}

// Dibuja la lista en el sprite con la barra en una posicion fraccional `sel`
// (sel entre 0 y SISTEMA_COUNT-1) y vuelca a pantalla. Sprite mapeado a y=20..134.
void renderSystemList(float sel) {
  sysList.fillSprite(C_BG);
  int barY = (int)(4 + sel * 22 + 0.5f);
  sysList.fillRoundRect(4, barY, 232, 20, 3, C_HDR);
  sysList.setTextSize(2);
  int active = (int)(sel + 0.5f);
  for (int i = 0; i < SISTEMA_COUNT; i++) {
    sysList.setTextColor(i == active ? BLACK : WHITE);
    sysList.setCursor(14, 4 + i * 22 + 3);
    sysList.print(SISTEMA_ITEMS[i]);
  }
  sysList.pushSprite(0, 20);
}

void drawSystemList() { renderSystemList((float)sistemaSel); }

// Desliza suavemente la barra de fromSel a toSel (sin fundido).
// Basado en tiempo: renderiza tantos frames como permita el hardware en DUR ms,
// asi la velocidad es constante y no se ve a saltos.
void animateSystemList(int fromSel, int toSel) {
  const uint32_t DUR = 160;   // duracion del deslizamiento en ms
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    float eased = t * t * (3.0f - 2.0f * t);
    renderSystemList(fromSel + (toSel - fromSel) * eased);
  }
  renderSystemList((float)toSel);   // frame final exacto
}

// Todas las categorías de Sistema se dibujan en el sprite `sysList` (mapeado a y=20..134,
// coords locales = globales-20) y se vuelcan con pushSprite(0,20). Sin parpadeo.

// ── Info ── (battShown permite animar la barra de batería)
void renderSystemInfo(int battShown) {
  bool charging = M5.Power.isCharging();
  uint16_t bCol = (battShown > 50) ? C_OK : (battShown > 20) ? C_WARN : C_BAD;

  sysList.fillSprite(C_BG);
  sysList.setTextSize(1);
  sysList.setTextColor(C_HDR); sysList.setCursor(4, 4); sysList.print("Bateria:");
  sysList.setTextColor(bCol);  sysList.setTextSize(2); sysList.setCursor(4, 14);
  sysList.printf("%3d%%", battShown);
  sysList.drawRect(70, 16, 108, 14, WHITE);
  sysList.fillRect(71, 17, (int)(106.0f * constrain(battShown, 0, 100) / 100), 12, bCol);
  sysList.fillRect(178, 19, 3, 8, WHITE);
  sysList.setTextSize(1); sysList.setTextColor(charging ? C_WARN : C_GREY);
  sysList.setCursor(4, 34); sysList.print(charging ? "Cargando (USB)" : "En bateria");

  sysList.drawLine(0, 46, 240, 46, C_GREY);
  sysList.setTextColor(C_HDR); sysList.setCursor(4, 50);   sysList.print("CPU:");
  sysList.setTextColor(WHITE); sysList.setCursor(40, 50);  sysList.printf("%d MHz", ESP.getCpuFreqMHz());
  sysList.setTextColor(C_HDR); sysList.setCursor(120, 50); sysList.print("Flash:");
  sysList.setTextColor(WHITE); sysList.setCursor(162, 50); sysList.printf("%dKB", ESP.getFlashChipSize() / 1024);
  sysList.setTextColor(C_HDR); sysList.setCursor(4, 64);   sysList.print("Heap:");
  sysList.setTextColor(WHITE); sysList.setCursor(40, 64);  sysList.printf("%d B", ESP.getFreeHeap());

  uint64_t chipId = ESP.getEfuseMac();
  sysList.setTextColor(C_HDR); sysList.setCursor(4, 78);  sysList.print("MAC:");
  sysList.setTextColor(WHITE); sysList.setCursor(40, 78);
  sysList.printf("%02X:%02X:%02X:%02X:%02X:%02X",
    (uint8_t)(chipId >> 40), (uint8_t)(chipId >> 32), (uint8_t)(chipId >> 24),
    (uint8_t)(chipId >> 16), (uint8_t)(chipId >> 8),  (uint8_t)(chipId));

  sysList.setTextColor(C_HDR); sysList.setCursor(4, 92); sysList.print("Uptime:");
  unsigned long s = millis() / 1000;
  sysList.setTextColor(C_ACCENT); sysList.setCursor(56, 92);
  sysList.printf("%02luh %02lum %02lus", s / 3600, (s % 3600) / 60, s % 60);
  sysList.pushSprite(0, 20);
}

void drawSystemInfo() { renderSystemInfo(M5.Power.getBatteryLevel()); }

// [A] en Info: barra de batería sube de 0% al valor real en ~2 s.
void animateSystemInfo() {
  int target = M5.Power.getBatteryLevel();
  const uint32_t DUR = 2000;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    renderSystemInfo((int)(target * t + 0.5f));
  }
  renderSystemInfo(target);
}

// ── Brillo ── (blocks = nº fraccional de bloques llenos, para el efecto de llenado)
void renderSystemBrillo(float blocks) {
  sysList.fillSprite(C_BG);
  sysList.setTextSize(2); sysList.setTextColor(C_ACCENT);
  sysList.setCursor(4, 8); sysList.print("Brillo");
  for (int i = 0; i < BRIGHT_COUNT; i++) {
    int x = 4 + i * 32;
    sysList.drawRect(x, 38, 28, 28, C_DARK);
    float fill = constrain(blocks - i, 0.0f, 1.0f);
    if (fill > 0.0f) {
      int h = (int)(28 * fill + 0.5f);
      sysList.fillRect(x, 38 + (28 - h), 28, h, C_HDR);   // se llena de abajo hacia arriba
    }
  }
  sysList.setTextSize(2); sysList.setTextColor(WHITE);
  sysList.setCursor(4, 80); sysList.printf("%d / 255", brightness);
  sysList.pushSprite(0, 20);
}

void drawSystemBrillo() { renderSystemBrillo((float)(getBrightIndex() + 1)); }

void animateSystemBrillo(int fromIdx, int toIdx) {
  const uint32_t DUR = 220;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    float eased = t * t * (3.0f - 2.0f * t);
    renderSystemBrillo((fromIdx + 1) + ((toIdx - fromIdx) * eased));
  }
  renderSystemBrillo((float)(toIdx + 1));
}

// ── Sonido ── (pos = 0 OFF, 1 ON; toggle deslizante tipo interruptor)
void renderSystemSonido(float pos) {
  sysList.fillSprite(C_BG);
  sysList.setTextSize(2); sysList.setTextColor(C_ACCENT);
  sysList.setCursor(4, 8); sysList.print("Sonido");
  const int tx = 4, ty = 44, tw = 110, th = 44, r = th / 2;
  sysList.fillRoundRect(tx, ty, tw, th, r, lerpColor565(C_DARK, C_OK, constrain(pos, 0.0f, 1.0f)));
  int kx = tx + r + (int)((tw - 2 * r) * constrain(pos, 0.0f, 1.0f));
  sysList.fillCircle(kx, ty + r, r - 5, WHITE);
  sysList.setTextSize(2); sysList.setTextColor(WHITE);
  sysList.setCursor(130, ty + 14); sysList.print(pos > 0.5f ? "ON" : "OFF");
  sysList.pushSprite(0, 20);
}

void drawSystemSonido() { renderSystemSonido(clickSound ? 1.0f : 0.0f); }

void animateSystemSonido(bool from, bool to) {
  const uint32_t DUR = 220;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    float eased = t * t * (3.0f - 2.0f * t);
    renderSystemSonido((from ? 1.0f : 0.0f) + ((to ? 1.0f : 0.0f) - (from ? 1.0f : 0.0f)) * eased);
  }
  renderSystemSonido(to ? 1.0f : 0.0f);
}

// ── Frecuencia ── (shownFreq permite el efecto contador tipo odómetro)
void renderSystemFreq(int shownFreq) {
  sysList.fillSprite(C_BG);
  sysList.setTextSize(2); sysList.setTextColor(C_ACCENT);
  sysList.setCursor(4, 8); sysList.print("Frecuencia");
  sysList.setTextSize(3); sysList.setTextColor(WHITE);
  sysList.setCursor(4, 44); sysList.printf("%d MHz", shownFreq);
  sysList.setTextSize(1); sysList.setTextColor(C_GREY);
  sysList.setCursor(4, 84); sysList.print("Menor = menos consumo");
  sysList.pushSprite(0, 20);
}

void drawSystemFreq() { renderSystemFreq(cpuFreq); }

// ── Color ── lista de temas con muestras; el tema activo va resaltado.
void drawSystemColor() {
  sysList.fillSprite(C_BG);
  sysList.setTextSize(2); sysList.setTextColor(C_ACCENT);
  sysList.setCursor(4, 6); sysList.print("Color");
  for (int i = 0; i < THEME_COUNT; i++) {
    int  y   = 32 + i * 16;
    bool sel = (i == theme);
    if (sel) sysList.fillRoundRect(2, y - 2, 236, 15, 2, C_HDR);
    sysList.fillRoundRect(10, y, 14, 11, 2, THEMES[i].hdr);
    sysList.fillRoundRect(26, y, 14, 11, 2, THEMES[i].accent);
    sysList.setTextSize(1); sysList.setTextColor(sel ? BLACK : WHITE);
    sysList.setCursor(48, y + 2); sysList.print(THEMES[i].name);
  }
  sysList.pushSprite(0, 20);
}

// [A] en Frecuencia: los números corren rápido (odómetro) hasta la nueva frecuencia.
void animateSystemFreq(int from, int to) {
  const uint32_t DUR = 500;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    float eased = 1.0f - (1.0f - t) * (1.0f - t);   // ease-out: desacelera al llegar
    renderSystemFreq(from + (int)((to - from) * eased));
  }
  renderSystemFreq(to);
}

void drawSystemScreen() {
  drawHeader("SISTEMA");
  if (sistemaInList) { drawSystemList(); return; }
  switch (sistemaSel) {
    case 0: drawSystemInfo();   break;
    case 1: drawSystemBrillo(); break;
    case 2: drawSystemSonido(); break;
    case 3: drawSystemFreq();   break;
    case 4: drawSystemColor();  break;
  }
}

// ════════════════════════════════════════════════════════════
//  MENÚ 1 – NIVEL DE BURBUJA
// ════════════════════════════════════════════════════════════

// Render completo del nivel en el sprite `sysList` (y=20..134, local = global-20).
// Un único pushSprite por frame → sin parpadeo en la burbuja.
void renderLevel(float angleX, float angleY, float total, uint16_t col) {
  const int CX = 62, CY = 56, R = 50;   // gauge (coords locales)
  sysList.fillSprite(C_BG);

  // Anillos concéntricos
  sysList.drawCircle(CX, CY, R,       C_GREY);
  sysList.drawCircle(CX, CY, R - 1,   C_DARK);
  sysList.drawCircle(CX, CY, R * 2/3, C_DARK);
  sysList.drawCircle(CX, CY, R / 3,   C_DARK);

  // Marcas cada 45°
  for (int i = 0; i < 8; i++) {
    float a = i * (PI / 4.0f);
    sysList.drawLine(CX + cosf(a) * (R - 1), CY + sinf(a) * (R - 1),
                     CX + cosf(a) * (R - 6), CY + sinf(a) * (R - 6), C_GREY);
  }

  // Cruceta
  sysList.drawFastHLine(CX - R, CY, 2 * R, C_DARK);
  sysList.drawFastVLine(CX, CY - R, 2 * R, C_DARK);

  // Zona objetivo: anillo central que se ilumina al nivelar
  sysList.drawCircle(CX, CY, BUB_R + 3, (total < 3.0f) ? C_OK : C_GREY);

  // Burbuja
  int bx = CX + constrain((int)(angleY / 45.0f * (R - BUB_R)), -(R - BUB_R), (R - BUB_R));
  int by = CY + constrain((int)(angleX / 45.0f * (R - BUB_R)), -(R - BUB_R), (R - BUB_R));
  sysList.fillCircle(bx, by, BUB_R, col);
  sysList.drawCircle(bx, by, BUB_R, WHITE);
  sysList.fillCircle(bx - 3, by - 3, 2, WHITE);   // reflejo "cristal"

  // Panel derecho
  const int px = 126;
  sysList.setTextSize(1); sysList.setTextColor(C_GREY);
  sysList.setCursor(px, 6); sysList.print("PITCH (X)");
  sysList.setTextSize(2); sysList.setTextColor(col);
  sysList.setCursor(px, 18); sysList.printf("%+5.1f", angleX);
  sysList.setTextSize(1); sysList.print(" o");

  sysList.setTextSize(1); sysList.setTextColor(C_GREY);
  sysList.setCursor(px, 46); sysList.print("ROLL (Y)");
  sysList.setTextSize(2); sysList.setTextColor(col);
  sysList.setCursor(px, 58); sysList.printf("%+5.1f", angleY);
  sysList.setTextSize(1); sysList.print(" o");

  // Estado
  if (total < 3.0f) {
    sysList.fillRoundRect(px, 88, 108, 24, 4, C_OK);
    sysList.setTextColor(BLACK); sysList.setTextSize(2);
    sysList.setCursor(px + 12, 92); sysList.print("NIVEL");
  } else {
    sysList.fillRoundRect(px, 88, 108, 24, 4, C_DARK);
    sysList.setTextColor(col); sysList.setTextSize(2);
    sysList.setCursor(px + 6, 92); sysList.printf("%.1f", total);
    sysList.setTextSize(1); sysList.print(" gr");
  }

  sysList.pushSprite(0, 20);
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

  // Guarda los últimos valores para que mqttPublishLevel() los envíe
  levelPitch = angleX; levelRoll = angleY; levelTotal = total;

  renderLevel(angleX, angleY, total, col);
}

void drawLevelScreen() {
  drawHeader("NIVEL");
  M5.Imu.getAccel(&sAx, &sAy, &sAz);   // semilla del suavizado
  updateLevelValues();                 // primer frame
}

// ════════════════════════════════════════════════════════════
//  MENÚ 2 – WIFI SCANNER
// ════════════════════════════════════════════════════════════

const char* encStr(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN:            return "Abierta";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Ent";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "?";
  }
}

// RSSI (dBm) → calidad aproximada en %
int wifiQuality(int rssi) { return constrain(2 * (rssi + 100), 0, 100); }

void drawWifiScanning() {
  drawHeader("WIFI");
  M5.Lcd.setTextColor(C_WARN); M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(28, 58); M5.Lcd.print("Escaneando...");
}

// Dibuja una fila de red en el sprite, desplazada `dx` px en horizontal (para la transición).
void drawWifiRow(int i, int row, int dx) {
  int y = 16 + row * 16;
  bool sel = (i == wifiSelected);
  if (sel) sysList.fillRoundRect(2 + dx, y - 2, 236, 15, 2, C_HDR);
  int rssi = WiFi.RSSI(i);
  uint16_t col = sel ? WHITE : (rssi > -65) ? C_OK : (rssi > -80) ? C_WARN : C_BAD;
  sysList.setTextSize(1); sysList.setTextColor(col); sysList.setCursor(6 + dx, y);
  String ssid = WiFi.SSID(i);
  if (ssid.length() == 0) ssid = "(oculta)";
  if (ssid.length() > 17) ssid = ssid.substring(0, 16) + "~";
  bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
  sysList.printf("%-17s %4ddBm %s", ssid.c_str(), rssi, open ? " " : "*");
  if (i < 32 && wifiSaved[i]) sysList.fillCircle(232 + dx, y + 5, 3, C_OK);  // clave guardada
}

void drawWifiHeaderLine() {
  sysList.setTextSize(1);
  sysList.setTextColor(C_ACCENT); sysList.setCursor(4, 2);   sysList.printf("Redes: %d", wifiCount);
  sysList.setTextColor(C_GREY);   sysList.setCursor(190, 2); sysList.printf("%d/%d", wifiSelected + 1, wifiCount);
}

// Cuerpo de la lista en el sprite `sysList` (sin parpadeo al mover la selección).
void renderWifiList() {
  sysList.fillSprite(C_BG);
  if (!wifiScanned) { sysList.pushSprite(0, 20); return; }
  if (wifiCount == 0) {
    sysList.setTextSize(2); sysList.setTextColor(C_GREY);
    sysList.setCursor(4, 40); sysList.print("Sin redes");
    sysList.pushSprite(0, 20); return;
  }
  // auto-scroll para mantener visible la selección
  if (wifiSelected < wifiScrollOffset) wifiScrollOffset = wifiSelected;
  if (wifiSelected >= wifiScrollOffset + WIFI_PER_PAGE) wifiScrollOffset = wifiSelected - WIFI_PER_PAGE + 1;

  drawWifiHeaderLine();
  int end = min(wifiScrollOffset + WIFI_PER_PAGE, wifiCount);
  for (int i = wifiScrollOffset; i < end; i++) drawWifiRow(i, i - wifiScrollOffset, 0);
  sysList.pushSprite(0, 20);
}

// Reveal: las filas entran deslizándose desde la derecha, en cascada.
void renderWifiReveal(float t) {
  sysList.fillSprite(C_BG);
  drawWifiHeaderLine();
  int end = min(wifiScrollOffset + WIFI_PER_PAGE, wifiCount);
  for (int i = wifiScrollOffset; i < end; i++) {
    int row = i - wifiScrollOffset;
    float rt = constrain((t - row * 0.10f) / 0.55f, 0.0f, 1.0f);
    float e  = rt * rt * (3.0f - 2.0f * rt);
    drawWifiRow(i, row, (int)((1.0f - e) * 240.0f));
  }
  sysList.pushSprite(0, 20);
}

void revealWifiList() {
  drawHeader("WIFI");
  if (wifiCount == 0) { renderWifiList(); return; }
  const uint32_t DUR = 500;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;
    renderWifiReveal(t);
  }
  renderWifiList();   // estado final exacto
}

void drawWifiList() {
  drawHeader("WIFI");
  renderWifiList();
}

void drawWifiDetail() {
  drawHeader("RED");
  int i = wifiSelected;
  int rssi = WiFi.RSSI(i);
  int q    = wifiQuality(rssi);
  uint16_t qcol = (q > 60) ? C_OK : (q > 30) ? C_WARN : C_BAD;

  String ssid = WiFi.SSID(i);
  if (ssid.length() == 0) ssid = "(oculta)";
  if (ssid.length() > 19) ssid = ssid.substring(0, 18) + "~";
  M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(C_ACCENT);
  M5.Lcd.setCursor(4, 24); M5.Lcd.print(ssid);

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 48);  M5.Lcd.print("Senal:");
  M5.Lcd.setTextColor(qcol);   M5.Lcd.setCursor(58, 48); M5.Lcd.printf("%d%%  (%d dBm)", q, rssi);
  M5.Lcd.drawRect(4, 60, 150, 8, C_GREY);
  M5.Lcd.fillRect(5, 61, (int)(148.0f * q / 100.0f), 6, qcol);

  M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 76);  M5.Lcd.print("Canal:");
  M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(58, 76); M5.Lcd.printf("%d", WiFi.channel(i));

  M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 90);  M5.Lcd.print("Cifrado:");
  M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(58, 90); M5.Lcd.print(encStr(WiFi.encryptionType(i)));

  uint8_t* b = WiFi.BSSID(i);
  M5.Lcd.setTextColor(C_HDR);  M5.Lcd.setCursor(4, 104); M5.Lcd.print("BSSID:");
  M5.Lcd.setTextColor(WHITE);  M5.Lcd.setCursor(58, 104);
  if (b) M5.Lcd.printf("%02X:%02X:%02X:%02X:%02X:%02X", b[0], b[1], b[2], b[3], b[4], b[5]);

  // Línea de estado / acción (parte inferior). Usa el SSID completo (ssid de arriba va truncado).
  String fullSsid = WiFi.SSID(i);
  bool conn = (WiFi.status() == WL_CONNECTED && fullSsid.length() && WiFi.SSID() == fullSsid);
  M5.Lcd.setCursor(4, 120);
  if (conn) {
    M5.Lcd.setTextColor(C_OK);
    M5.Lcd.printf("Conectado  %s", WiFi.localIP().toString().c_str());
  } else if (connResult == 2 && connResultSsid == fullSsid) {
    M5.Lcd.setTextColor(C_BAD); M5.Lcd.print("Fallo - [A] reintentar");
  } else if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) {
    M5.Lcd.setTextColor(C_ACCENT); M5.Lcd.print("[A] conectar (abierta)");
  } else if (hasWifiPass(fullSsid)) {
    M5.Lcd.setTextColor(C_ACCENT); M5.Lcd.print("[A] conectar  (hold A: editar)");
  } else {
    M5.Lcd.setTextColor(C_ACCENT); M5.Lcd.print("[A] introducir clave");
  }
}

void drawWifiScreen() {
  if (wifiInKeyboard) { drawWifiKeyboard(); return; }
  if (wifiInDetail)   { drawWifiDetail();   return; }
  drawWifiList();
}

// Escanea (asíncrono) con el LED parpadeando; al terminar revela la lista con transición.
void scanWifiWithFeedback() {
  fadeToBlack();
  drawWifiScanning();
  fadeFromBlack();

  WiFi.mode(WIFI_STA);
  // Mantener la conexión: solo limpiar si NO estamos conectados (se puede escanear estando asociado).
  if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); delay(150); }
  WiFi.scanNetworks(true);   // asíncrono: no bloquea

  uint32_t lastBlink = 0; bool led = false;
  while (WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
    if (millis() - lastBlink > 120) {
      led = !led; digitalWrite(LED_PIN, led ? HIGH : LOW);
      lastBlink = millis();
    }
    delay(10);
  }
  digitalWrite(LED_PIN, LOW);   // LED apagado al terminar

  int res = WiFi.scanComplete();
  wifiCount = (res > 0) ? res : 0;
  wifiScanned = true;
  wifiSelected = 0; wifiScrollOffset = 0;
  connResult = 0;
  for (int i = 0; i < wifiCount && i < 32; i++) wifiSaved[i] = hasWifiPass(WiFi.SSID(i));

  revealWifiList();   // transición chula
}

// ─── Persistencia de contraseñas WiFi (NVS namespace "wifi") ──────
// Las claves de NVS están limitadas a 15 chars, así que no se puede usar el SSID
// como clave directa: se guardan en ranuras indexadas s0/p0, s1/p1, ... con un
// contador "cnt", y se busca el SSID linealmente.
String loadWifiPass(const String& ssid) {
  if (ssid.length() == 0) return "";
  prefs.begin("wifi", true);
  int n = prefs.getInt("cnt", 0);
  String res = "";
  char k[8];
  for (int i = 0; i < n; i++) {
    snprintf(k, sizeof(k), "s%d", i);
    if (prefs.getString(k, "") == ssid) {
      snprintf(k, sizeof(k), "p%d", i);
      res = prefs.getString(k, "");
      break;
    }
  }
  prefs.end();
  return res;
}

bool hasWifiPass(const String& ssid) { return loadWifiPass(ssid).length() > 0; }

void saveWifiPass(const String& ssid, const String& pass) {
  if (ssid.length() == 0) return;
  prefs.begin("wifi", false);
  int n = prefs.getInt("cnt", 0);
  char k[8]; int slot = -1;
  for (int i = 0; i < n; i++) {
    snprintf(k, sizeof(k), "s%d", i);
    if (prefs.getString(k, "") == ssid) { slot = i; break; }
  }
  if (slot < 0) {
    if (n < WIFI_SAVED_MAX) { slot = n; prefs.putInt("cnt", ++n); }
    else slot = 0;   // lleno: sobrescribe la ranura más antigua
  }
  snprintf(k, sizeof(k), "s%d", slot); prefs.putString(k, ssid);
  snprintf(k, sizeof(k), "p%d", slot); prefs.putString(k, pass);
  prefs.end();
}

// ─── Teclado de contraseña (2 botones) ───────────────────────────
// Rejilla de teclas; [A] tap mueve el cursor, [A] hold pulsa la tecla.
// Índices 0..76 = caracteres; 77=SP (espacio), 78=< (borrar), 79=OK.
const char* KB_CHARS =
  "abcdefghijklmnopqrstuvwxyz"
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "0123456789"
  "@#$%&*-_.+!?=/:";          // total: 77 caracteres (índices 0..76)
const int KB_COLS  = 16;
const int KEY_COUNT = 80;     // 77 caracteres + SP + < + OK

const char* keyLabel(int idx, char* buf) {
  if (idx < 77) { buf[0] = KB_CHARS[idx]; buf[1] = 0; return buf; }
  if (idx == 77) return "SP";
  if (idx == 78) return "<";
  return "OK";
}

void renderWifiKeyboard() {
  sysList.fillSprite(C_BG);
  sysList.setTextSize(1);

  // Vista previa de la clave (se muestra en claro para poder verificarla)
  String shown = pwBuf;
  if (shown.length() > 32) shown = "~" + shown.substring(shown.length() - 31);
  sysList.setTextColor(C_ACCENT);
  sysList.setCursor(4, 4);
  sysList.printf("Clave: %s_", shown.c_str());

  char buf[3];
  for (int i = 0; i < KEY_COUNT; i++) {
    int col = i % KB_COLS, row = i / KB_COLS;
    int x = col * 15, y = 18 + row * 19;
    bool sel = (i == kbSel);
    const char* lbl = keyLabel(i, buf);
    if (sel) sysList.fillRoundRect(x + 1, y, 13, 17, 2, C_HDR);
    sysList.setTextColor(sel ? BLACK : (i >= 77 ? C_ACCENT : WHITE));
    int lw = (int)strlen(lbl) * 6;
    sysList.setCursor(x + (15 - lw) / 2 + 1, y + 5);
    sysList.print(lbl);
  }
  sysList.pushSprite(0, 20);
}

void drawWifiKeyboard() {
  drawHeader("CLAVE");
  renderWifiKeyboard();
}

// Conecta (bloqueante, ~12 s máx) con el LED parpadeando; guarda la clave si va bien.
void wifiConnect(const String& ssid, const String& pass) {
  fadeToBlack();
  drawHeader("WIFI");
  M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(C_WARN);
  M5.Lcd.setCursor(20, 50); M5.Lcd.print("Conectando...");
  M5.Lcd.setTextSize(1); M5.Lcd.setTextColor(C_GREY);
  String s = ssid; if (s.length() > 28) s = s.substring(0, 27) + "~";
  M5.Lcd.setCursor(20, 82); M5.Lcd.print(s);
  fadeFromBlack();

  WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : (const char*)nullptr);
  uint32_t start = millis(), lb = 0; bool led = false;
  while (millis() - start < 12000) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED || st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) break;
    if (millis() - lb > 150) { led = !led; digitalWrite(LED_PIN, led ? HIGH : LOW); lb = millis(); }
    delay(20);
  }
  digitalWrite(LED_PIN, LOW);

  bool ok = (WiFi.status() == WL_CONNECTED);
  connResult = ok ? 1 : 2; connResultSsid = ssid;
  if (ok && pass.length()) {
    saveWifiPass(ssid, pass);
    if (wifiSelected < 32) wifiSaved[wifiSelected] = true;
  }
  wifiInKeyboard = false;
  wifiInDetail = true;
  transition();   // vuelve al detalle, que muestra el estado de conexión
}

// Abre el teclado para el SSID dado (precarga la clave guardada si existe).
void startKeyboard(const String& ssid) {
  connSsid = ssid;
  pwBuf    = loadWifiPass(ssid);
  kbSel    = 0;
  wifiInKeyboard = true;
  transition();
}

// [A] tap en el detalle: conecta directo si es abierta o tiene clave; si no, teclado.
void wifiConnectAction() {
  int i = wifiSelected;
  String ssid = WiFi.SSID(i);
  if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) { wifiConnect(ssid, ""); return; }
  String saved = loadWifiPass(ssid);
  if (saved.length()) wifiConnect(ssid, saved);
  else                startKeyboard(ssid);
}

// [A] hold en el teclado: ejecuta la tecla resaltada.
void wifiKeyCommit() {
  if (kbSel < 77) {
    if (pwBuf.length() < 63) pwBuf += KB_CHARS[kbSel];
    renderWifiKeyboard();
  } else if (kbSel == 77) {                       // SP
    if (pwBuf.length() < 63) pwBuf += ' ';
    renderWifiKeyboard();
  } else if (kbSel == 78) {                       // borrar
    if (pwBuf.length()) pwBuf.remove(pwBuf.length() - 1);
    renderWifiKeyboard();
  } else {                                        // OK → conectar
    wifiConnect(connSsid, pwBuf);
  }
}

// ════════════════════════════════════════════════════════════
//  MENÚ 3 – SENSORES I2C
// ════════════════════════════════════════════════════════════

void doModuleScan() {
  foundCount = 0; envReady = false; qmpReady = false;
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
  if (sht3x.begin(&Wire, SHT3X_I2C_ADDR, GROVE_SDA, GROVE_SCL, 400000U))   envReady = true;
  if (qmp6988.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, GROVE_SDA, GROVE_SCL, 400000U)) qmpReady = true;
}

// Tarjeta de métrica (116×46) en el sprite sysList.
void sensorCard(int x, int y, const char* label, uint16_t vcol, const char* valstr, const char* unit) {
  const int w = 116, h = 46;
  sysList.fillRoundRect(x, y, w, h, 4, C_DARK);
  sysList.drawRoundRect(x, y, w, h, 4, C_GREY);
  sysList.setTextSize(1); sysList.setTextColor(C_ACCENT);
  sysList.setCursor(x + 8, y + 6); sysList.print(label);
  sysList.setTextSize(3); sysList.setTextColor(vcol);
  sysList.setCursor(x + 8, y + 18); sysList.print(valstr);
  sysList.setTextSize(1); sysList.setTextColor(C_GREY);
  sysList.setCursor(x + w - 26, y + h - 11); sysList.print(unit);
}

// Render completo de Sensores en sysList (todo en una pantalla, sin parpadeo).
void renderModule() {
  sysList.fillSprite(C_BG);

  if (foundCount == 0) {
    sysList.setTextSize(2); sysList.setTextColor(C_GREY);
    sysList.setCursor(4, 14); sysList.print("Sin modulo");
    sysList.setTextSize(1);
    sysList.setCursor(4, 44); sysList.print("Conecta al puerto Grove HY2.0");
    sysList.setCursor(4, 58); sysList.print("SDA=G0  SCL=G26");
    sysList.pushSprite(0, 20); return;
  }

  if (envReady || qmpReady) {
    sysList.setTextSize(1); sysList.setTextColor(C_OK);
    sysList.setCursor(4, 2);
    sysList.printf("ENV.III  %s%s%s", envReady ? "SHT3X" : "",
                   (envReady && qmpReady) ? " + " : "", qmpReady ? "QMP6988" : "");
    char buf[12];

    uint16_t tcol = (envTemp < 20) ? C_ACCENT : (envTemp < 30) ? C_OK : (envTemp < 38) ? C_WARN : C_BAD;
    if (envReady) snprintf(buf, sizeof(buf), "%.1f", envTemp); else strcpy(buf, "--");
    sensorCard(2,   14, "TEMP",     tcol,     buf, "C");
    if (envReady) snprintf(buf, sizeof(buf), "%.0f", envHum);  else strcpy(buf, "--");
    sensorCard(122, 14, "HUMEDAD",  C_OK,     buf, "%");
    if (qmpReady) snprintf(buf, sizeof(buf), "%.0f", qmpPres); else strcpy(buf, "--");
    sensorCard(2,   64, "PRESION",  C_ACCENT, buf, "hPa");
    if (qmpReady) snprintf(buf, sizeof(buf), "%.0f", qmpAlt);  else strcpy(buf, "--");
    sensorCard(122, 64, "ALTITUD",  C_WARN,   buf, "m");
  } else {
    sysList.setTextSize(1); sysList.setTextColor(C_ACCENT);
    sysList.setCursor(4, 2); sysList.printf("%d dispositivo(s) I2C:", foundCount);
    for (int i = 0; i < min(foundCount, 6); i++) {
      sysList.setTextColor(C_WARN); sysList.setCursor(4, 16 + i * 16);
      sysList.printf("0x%02X %s", foundAddrs[i], foundLabels[i]);
    }
  }
  sysList.pushSprite(0, 20);
}

// Solo lee los sensores a los globales (sin tocar la pantalla).
void readSensors() {
  if (envReady && sht3x.update())   { envTemp = sht3x.cTemp; envHum = sht3x.humidity; }
  if (qmpReady && qmp6988.update()) { qmpPres = qmp6988.pressure / 100.0f; qmpAlt = qmp6988.altitude; }
}

void updateModuleValues() {
  readSensors();
  renderModule();
}

void drawModuleScreen() {
  drawHeader("SENSORES");
  if (envReady || qmpReady) updateModuleValues();   // lee y renderiza al instante
  else                      renderModule();
}

// ════════════════════════════════════════════════════════════
//  ENVÍO A JARVIS (MQTT)
// ════════════════════════════════════════════════════════════

// Lee los sensores y publica la telemetría como JSON en el topic del dispositivo.
void mqttPublishSensors() {
  if (!envReady && !qmpReady) return;            // sin sensores no hay nada que mandar
  readSensors();

  char payload[96];
  int n = snprintf(payload, sizeof(payload), "{");
  bool first = true;
  if (envReady) {
    n += snprintf(payload + n, sizeof(payload) - n,
                  "\"temperatura\":%.1f,\"humedad\":%.1f", envTemp, envHum);
    first = false;
  }
  if (qmpReady) {
    n += snprintf(payload + n, sizeof(payload) - n,
                  "%s\"presion\":%.1f,\"altitud\":%.1f", first ? "" : ",", qmpPres, qmpAlt);
  }
  snprintf(payload + n, sizeof(payload) - n, "}");

  mqtt.publish(TOPIC_TELEMETRY, payload);
}

// Publica el nivel (inclinación del IMU) como JSON. Solo se usa en la pantalla Nivel.
// Los valores los calcula updateLevelValues() (que corre a ~30 fps mientras estás en Nivel).
void mqttPublishLevel() {
  char payload[80];
  snprintf(payload, sizeof(payload),
           "{\"pitch\":%.1f,\"roll\":%.1f,\"inclinacion\":%.1f}",
           levelPitch, levelRoll, levelTotal);
  mqtt.publish(TOPIC_TELEMETRY, payload);
}

// Mantiene la conexión MQTT y publica cada `interval` ms llamando a `publishFn`.
// Se llama desde loop() SOLO en las pantallas que retransmiten (Sensores o Nivel).
// Solo actúa con WiFi asociado; el (re)conectar se limita a MQTT_RETRY_MS para no
// bloquear la UI si el broker no responde. Al salir de esas pantallas deja de llamarse:
// la conexión queda inactiva y el broker acaba marcando "offline" (Last Will) por keepalive.
void mqttService(uint32_t interval, void (*publishFn)()) {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!mqtt.connected()) {
    if (millis() - lastMqttRetry < MQTT_RETRY_MS) return;
    lastMqttRetry = millis();
    // Last Will: el broker marca "offline" si nos caemos de golpe.
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                     TOPIC_STATUS, 0, true, "offline")) {
      mqtt.publish(TOPIC_STATUS, "online", true);
      lastMqttPublish = millis() - interval;   // fuerza un envío inmediato al conectar
    } else {
      return;
    }
  }

  mqtt.loop();
  if (millis() - lastMqttPublish >= interval) {
    lastMqttPublish = millis();
    publishFn();
  }
}

// ════════════════════════════════════════════════════════════
//  TRANSICIONES DE PANTALLA
// ════════════════════════════════════════════════════════════

// Apaga el backlight en 10 pasos (~130 ms). La pantalla queda a oscuras.
void fadeToBlack() {
  for (int i = 10; i >= 0; i--) {
    M5.Lcd.setBrightness(brightness * i / 10);
    delay(13);
  }
}

// Enciende el backlight en 10 pasos (~130 ms). Revela el contenido ya dibujado.
void fadeFromBlack() {
  for (int i = 0; i <= 10; i++) {
    M5.Lcd.setBrightness(brightness * i / 10);
    delay(13);
  }
}

// Redibuja la pantalla activa según el estado de navegación.
void drawCurrentScreen() {
  if (inHome) { drawHomeScreen(); return; }
  switch (selectedMenu) {
    case 0: drawSystemScreen(); break;
    case 1: drawLevelScreen();  break;
    case 2: drawWifiScreen();   break;
    case 3: drawModuleScreen(); break;
  }
}

// Transición universal: oscurece, redibuja con la pantalla apagada y revela.
// Unifica el efecto en toda la app y oculta el parpadeo del refresco.
void transition() {
  fadeToBlack();
  drawCurrentScreen();
  fadeFromBlack();
}

// ════════════════════════════════════════════════════════════
//  NAVEGACIÓN
// ════════════════════════════════════════════════════════════

void enterMenu() {
  inHome = false;
  if (selectedMenu == 0) { sistemaInList = true; sistemaSel = 0; }
  if (selectedMenu == 2) {
    wifiInDetail = false; wifiInKeyboard = false;
    // Si ya estamos conectados, no reescanear ni desconectar: mostrar la lista ya escaneada.
    if (WiFi.status() == WL_CONNECTED && wifiScanned) transition();
    else                                              scanWifiWithFeedback();
    return;
  }
  transition();
}

void goHome() {
  inHome = true;
  transition();
}

// Acción [A] dentro de un menú
void doMenuInteract() {
  switch (selectedMenu) {
    case 0: // Sistema: en lista [A] avanza cursor; dentro de opcion [A] modifica (con animacion propia)
      if (sistemaInList) {
        int from = sistemaSel;
        sistemaSel = (sistemaSel + 1) % SISTEMA_COUNT;
        animateSystemList(from, sistemaSel);
      } else if (sistemaSel == 0) {
        animateSystemInfo();
      } else if (sistemaSel == 1) {
        int oldIdx = getBrightIndex();
        brightness = BRIGHT_STEPS[(oldIdx + 1) % BRIGHT_COUNT];
        M5.Lcd.setBrightness(brightness);
        saveSettings();
        animateSystemBrillo(oldIdx, getBrightIndex());
      } else if (sistemaSel == 2) {
        bool old = clickSound;
        clickSound = !clickSound;
        saveSettings();
        animateSystemSonido(old, clickSound);
      } else if (sistemaSel == 3) {
        int oldFreq = cpuFreq;
        cpuFreq = CPU_STEPS[(getCpuIndex() + 1) % CPU_COUNT];
        setCpuFrequencyMhz(cpuFreq);
        saveSettings();
        animateSystemFreq(oldFreq, cpuFreq);
      } else if (sistemaSel == 4) {
        theme = (theme + 1) % THEME_COUNT;
        applyTheme();
        saveSettings();
        transition();   // recolorea toda la UI con un fundido elegante
      }
      break;
    case 2: // WiFi
      if (wifiInKeyboard) {                 // teclado: [A] mueve el cursor
        kbSel = (kbSel + 1) % KEY_COUNT;
        renderWifiKeyboard();
      } else if (wifiInDetail) {            // detalle: [A] conecta
        wifiConnectAction();
      } else if (wifiScanned && wifiCount > 0) {  // lista: [A] baja la seleccion
        wifiSelected = (wifiSelected + 1) % wifiCount;
        renderWifiList();
      }
      break;
    default: break;   // Sensores (3): todo en una pantalla, [A] sin acción
  }
}

// Acción [B] corto dentro de un menú
void doMenuRefresh() {
  switch (selectedMenu) {
    case 0:
      if (sistemaInList) {            // en lista: [B] retrocede la seleccion
        int from = sistemaSel;
        sistemaSel = (sistemaSel - 1 + SISTEMA_COUNT) % SISTEMA_COUNT;
        animateSystemList(from, sistemaSel);
      } else {                        // dentro de opcion: [B] vuelve a la lista
        sistemaInList = true;
        transition();
      }
      break;
    case 2: // WiFi
      if (wifiInKeyboard) {                          // teclado: [B] retrocede el cursor
        kbSel = (kbSel - 1 + KEY_COUNT) % KEY_COUNT;
        renderWifiKeyboard();
      } else if (wifiInDetail) {                      // detalle: [B] vuelve a la lista
        wifiInDetail = false; connResult = 0; transition();
      } else if (wifiScanned && wifiCount > 0) {      // lista: [B] retrocede la seleccion
        wifiSelected = (wifiSelected - 1 + wifiCount) % wifiCount;
        renderWifiList();
      }
      break;
    case 3: doModuleScan(); transition(); break;
    default: break;
  }
}

// ════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════════════════════════

void loadSettings() {
  prefs.begin("sistema", true);   // solo lectura
  brightness = prefs.getInt ("bri", brightness);
  clickSound = prefs.getBool("snd", clickSound);
  cpuFreq    = prefs.getInt ("cpu", cpuFreq);
  theme      = prefs.getInt ("thm", theme);
  prefs.end();
}

void saveSettings() {
  prefs.begin("sistema", false);
  prefs.putInt ("bri", brightness);
  prefs.putBool("snd", clickSound);
  prefs.putInt ("cpu", cpuFreq);
  prefs.putInt ("thm", theme);
  prefs.end();
}

// Pantalla de arranque: "Hello!" + barra de 0 a 100 (~3.5 s).
void bootAnimation() {
  M5.Lcd.fillScreen(C_BG);
  const int bx = 20, by = 90, bw = 200, bh = 18;
  M5.Lcd.drawRoundRect(bx, by, bw, bh, 4, C_GREY);

  // Curva de carga (tiempo → progreso): tramos rápidos, lentos y pausas.
  const float KT[] = { 0.00f, 0.12f, 0.22f, 0.35f, 0.48f, 0.60f, 0.74f, 0.88f, 1.00f };
  const float KP[] = { 0.00f, 0.34f, 0.38f, 0.38f, 0.70f, 0.74f, 0.74f, 0.95f, 1.00f };
  const int   KN   = sizeof(KT) / sizeof(KT[0]);

  const uint32_t DUR = 4000;
  uint32_t start = millis();
  for (;;) {
    float t = (float)(millis() - start) / (float)DUR;
    if (t >= 1.0f) break;

    // Progreso interpolado según la curva (no lineal: acelera, se para, acelera)
    float p = 1.0f;
    for (int k = 1; k < KN; k++) {
      if (t <= KT[k]) {
        float f = (t - KT[k - 1]) / (KT[k] - KT[k - 1]);
        p = KP[k - 1] + (KP[k] - KP[k - 1]) * f;
        break;
      }
    }

    // "Hello!" aparece con fundido al principio
    float ha = constrain(t / 0.25f, 0.0f, 1.0f);
    M5.Lcd.setTextSize(5); M5.Lcd.setTextColor(lerpColor565(C_BG, C_HDR, ha));
    M5.Lcd.setCursor(30, 24); M5.Lcd.print("Hello!");

    // Barra: relleno proporcional + brillo que se desliza
    int w = (int)((bw - 4) * p);
    M5.Lcd.fillRect(bx + 2, by + 2, w, bh - 4, C_HDR);
    if (w > 6) M5.Lcd.fillRect(bx + 2 + w - 6, by + 2, 6, bh - 4, C_ACCENT);  // punta brillante

    // Porcentaje: ancho y posición FIJOS + fondo opaco → sobrescribe sin parpadeo
    char b[6]; snprintf(b, sizeof(b), "%3d%%", (int)(p * 100));   // "  0%".."100%" (4 chars)
    M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(C_ACCENT, C_BG);
    M5.Lcd.setCursor(96, by + bh + 6); M5.Lcd.print(b);
  }

  // Completar al 100%
  M5.Lcd.fillRect(bx + 2, by + 2, bw - 4, bh - 4, C_OK);
  M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(C_ACCENT, C_BG);
  M5.Lcd.setCursor(96, by + bh + 6); M5.Lcd.print("100%");
  delay(350);
  fadeToBlack();
}

void setup() {
  M5.begin();
  M5.Lcd.setRotation(1);
  loadSettings();                    // recupera ultimos valores antes de aplicarlos
  applyTheme();                      // aplica el tema guardado
  M5.Lcd.setBrightness(brightness);
  setCpuFrequencyMhz(cpuFreq);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  WiFi.setAutoReconnect(false);      // sin reconexión automática: el usuario reconecta a mano
  WiFi.setSleep(false);              // desactiva el modem-sleep del WiFi: MUCHA menos latencia en MQTT (clave para el nivel en vivo)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setSocketTimeout(4);          // no bloquear la UI demasiado si el broker no responde
  carousel.createSprite(150, 113);   // panel derecho: x=90..239, y=22..134
  sysList.createSprite(240, 115);    // lista de Sistema: y=20..134 (barra deslizante)
  iconCv.createSprite(88, 113);      // icono HOME: x=0..87, y=22..134 (animacion constante)
  bootAnimation();                   // splash "Hello!"
  doModuleScan();
  drawHomeScreen();
  fadeFromBlack();                   // revela HOME tras el splash
}

void loop() {
  M5.update();

  // ── Botón A: long press en HOME = entrar menú; tap en HOME = ciclar; dentro = interactuar ──
  if (M5.BtnA.wasPressed()) {
    btnADownAt = millis(); btnALongFired = false;
  }
  if (M5.BtnA.isPressed() && !btnALongFired && (millis() - btnADownAt >= LONG_PRESS_MS)) {
    btnALongFired = true;
    playClick();
    if (inHome) {
      enterMenu();
    } else if (selectedMenu == 0 && sistemaInList) {
      // Sistema: [A] mantenido abre la opcion seleccionada
      sistemaInList = false;
      transition();
    } else if (selectedMenu == 2 && wifiInKeyboard) {
      // WiFi: [A] mantenido pulsa la tecla resaltada
      wifiKeyCommit();
    } else if (selectedMenu == 2 && wifiScanned && !wifiInDetail && wifiCount > 0) {
      // WiFi: [A] mantenido abre el detalle de la red seleccionada
      wifiInDetail = true;
      transition();
    } else if (selectedMenu == 2 && wifiInDetail &&
               WiFi.encryptionType(wifiSelected) != WIFI_AUTH_OPEN) {
      // WiFi: [A] mantenido en el detalle edita/reescribe la clave
      startKeyboard(WiFi.SSID(wifiSelected));
    }
  }
  if (M5.BtnA.wasReleased() && !btnALongFired) {
    playClick();
    if (inHome) {
      if (!homeAnimating) {
        homeAnimating = true;
        homeAnimStart = millis();
      }
    } else {
      doMenuInteract();
    }
  }

  // ── Botón B: long press en menú = HOME; corto en menú = refrescar ──
  if (M5.BtnB.wasPressed()) {
    btnBDownAt = millis(); btnBLongFired = false;
  }
  if (M5.BtnB.isPressed() && !btnBLongFired && (millis() - btnBDownAt >= LONG_PRESS_MS)) {
    btnBLongFired = true;
    playClick();
    if (!inHome) {
      if (selectedMenu == 2 && wifiInKeyboard) {   // teclado: [B] hold cancela hacia el detalle
        wifiInKeyboard = false; wifiInDetail = true; transition();
      } else {
        goHome();
      }
    }
  }
  if (M5.BtnB.wasReleased() && !btnBLongFired) {
    if (!inHome) {
      playClick();
      doMenuRefresh();
    }
  }

  // ── Chord A+B en la lista WiFi = reescanear (desconecta; sin reconexión automática) ──
  // Se evalúa tras los bloques de cada botón: al disparar, anula su tap/long para que no
  // muevan la selección al soltar. chordFired se rearma cuando ambos botones se sueltan.
  if (!chordFired && M5.BtnA.isPressed() && M5.BtnB.isPressed() &&
      !inHome && selectedMenu == 2 && !wifiInDetail && !wifiInKeyboard) {
    chordFired = true;
    btnALongFired = true; btnBLongFired = true;
    playClick();
    WiFi.disconnect();              // autoReconnect ya está en false: no se reconecta solo
    scanWifiWithFeedback();
  }
  if (!M5.BtnA.isPressed() && !M5.BtnB.isPressed()) chordFired = false;

  // ── Animación carrusel HOME (cambio de menú) ──
  if (inHome && homeAnimating) {
    float t      = constrain((float)(millis() - homeAnimStart) / (float)ANIM_MS, 0.0f, 1.0f);
    float eased  = t * t * (3.0f - 2.0f * t);
    int   nextMenu = (selectedMenu + 1) % TOTAL_MENUS;

    renderHomeSlide(nextMenu, eased, millis() / 1000.0f);
    drawCarouselPanel(-eased);

    if (t >= 1.0f) {
      homeAnimating = false;
      selectedMenu  = nextMenu;
      drawCarouselPanel(0.0f);
    }
  }

  // ── Animación constante del icono en reposo (HOME, ~25 fps) ──
  if (inHome && !homeAnimating && millis() - lastIconAnim > 40) {
    renderHomeIcon(millis() / 1000.0f);
    lastIconAnim = millis();
  }

  // ── Actualizaciones en vivo ──
  if (!inHome && selectedMenu == 1 && millis() - lastLiveUpdate > 33) {
    updateLevelValues();
    lastLiveUpdate = millis();
  }
  if (!inHome && selectedMenu == 3 && (envReady || qmpReady) && millis() - lastModuleUpdate > 2000) {
    updateModuleValues();
    lastModuleUpdate = millis();
  }

  // ── Envío de telemetría a Jarvis (MQTT) ──
  // Sensores: temperatura/humedad/presión cada 10 s. Nivel: pitch/roll cada 200 ms (casi en vivo).
  if (!inHome && selectedMenu == 3)      mqttService(MQTT_INTERVAL,       mqttPublishSensors);
  else if (!inHome && selectedMenu == 1) mqttService(MQTT_LEVEL_INTERVAL, mqttPublishLevel);
}
