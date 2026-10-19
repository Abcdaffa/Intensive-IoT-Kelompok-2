#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <math.h>
#include "esp_wifi.h"

// =====================================================
// PASTE - Anti-Tremor (TANPA FILTER)
// ESP32-C3 Mini + MPU6050 + 2x Servo + OLED + PUSH BUTTON
// - Servo 1 (D3) = KIRI-KANAN (roll)  -> MG90S (500-2500 us, full 180°)
// - Servo 2 (D2) = ATAS-BAWAH (pitch) -> SG90S (500-2500 us, full 180°) [INVERT]
// - MPU DIPUTAR 180 DERAJAT
// - Accelerometer LANGSUNG (tanpa filter, tanpa gyro)
// - Baterai: random awal 80-95%, turun 1% tiap 5 menit
// - Servo HANYA aktif saat tombol ditekan (momentary)
// - Sudut awal servo FIX di 90 derajat
// - FIX WiFi (SOLUSI 4)
// - Status getaran 3 level: NORMAL / SEDANG / BAHAYA
// =====================================================

// ===============================
// WIFI
// ===============================
const char* WIFI_SSID     = "Redmi1121";
const char* WIFI_PASSWORD = "daoa123456";

#define WIFI_SOLUSI 4

const char* FIREBASE_URL =
  "https://iot-intensive-2026-default-rtdb.firebaseio.com/PASTE.json";

const char* SHEETS_URL =
  "https://script.google.com/macros/s/AKfycbzycgcv6rzm9AdC6Z7x44CEiyQzHsnffbfDhKqTPrNkB4vzJxjTcKYqjNA3wLwq2ifD/exec";

// ===============================
// PIN
// ===============================
#define SDA_PIN     8
#define SCL_PIN     9
#define SERVO_PIN   3   // Servo 1 -> D3 -> MG90S -> KIRI-KANAN (roll)
#define SERVO2_PIN  2   // Servo 2 -> D2 -> SG90S -> ATAS-BAWAH (pitch)
#define TTP_PIN     1   // PUSH BUTTON -> D1 (ke GND)

// ===============================
// SUDUT AWAL SERVO (FIX)
// ===============================
const int SUDUT_AWAL_SERVO = 90;

// ===============================
// PULSE WIDTH PER SERVO
// ===============================
const int SERVO_ROLL_MIN_US = 500;
const int SERVO_ROLL_MAX_US = 2500;

const int SERVO_PITCH_MIN_US = 500;
const int SERVO_PITCH_MAX_US = 2500;

// ===============================
// MAPPING SERVO (dua-duanya full 0-180)
// ===============================
const int SERVO_ROLL_MAP_MIN = 0;
const int SERVO_ROLL_MAP_MAX = 180;

const int SERVO_PITCH_MAP_MIN = 0;
const int SERVO_PITCH_MAP_MAX = 180;

// ===============================
// PARAMETER TANPA FILTER (accelerometer langsung)
// ===============================
const float ROLL_DEADZONE   = 2.0;
const float PITCH_DEADZONE  = 2.0;
const float MAX_TILT_DEG    = 30.0;

// ===============================
// THRESHOLD STATUS GETARAN
// ===============================
const float THRESHOLD_NORMAL = 5.0;
const float THRESHOLD_SEDANG = 15.0;

// ===============================
// SPEED LIMIT SERVO
// ===============================
const int MAX_STEP = 5;

// ===============================
// BATERAI (random + turun 1% tiap 5 menit)
// ===============================
int batteryPercentage = 85;

// ===============================
// OLED
// ===============================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===============================
// SERVO & MPU
// ===============================
Servo myServoRoll;    // D3 = MG90S (kiri-kanan / roll)
Servo myServoPitch;   // D2 = SG90S (atas-bawah / pitch)
MPU6050 mpu;

bool servoAttached = false;

int servoRollPrev  = 90;
int servoPitchPrev = 90;

// ===============================
// TIMER
// ===============================
unsigned long lastSensorRead    = 0;
unsigned long lastSaveData      = 0;
unsigned long lastOledUpdate    = 0;
unsigned long lastSerialLog     = 0;
unsigned long lastDebugLog      = 0;
unsigned long lastWiFiCheck     = 0;
unsigned long lastSendAttempt   = 0;
unsigned long lastWiFiFlagCheck = 0;
unsigned long lastBatteryTick   = 0;

const unsigned long SENSOR_INTERVAL     = 10;
const unsigned long SAVE_INTERVAL       = 200;
const unsigned long OLED_INTERVAL       = 250;
const unsigned long SERIAL_INTERVAL     = 1000;
const unsigned long DEBUG_INTERVAL      = 500;
const unsigned long COOLDOWN_DURATION   = 3000;
const unsigned long WIFI_CHECK_INTERVAL = 15000;
const unsigned long SEND_RETRY_INTERVAL = 3000;
const unsigned long BATTERY_DECREASE_INTERVAL = 300000;  // 5 menit
const int           BATCH_SIZE          = 50;

// ===============================
// TOMBOL 10x RESET
// ===============================
const int           TTP_RESET_COUNT  = 10;
const unsigned long TTP_RESET_WINDOW = 20000;
int           ttpPressCount    = 0;
unsigned long ttpWindowStart   = 0;
bool          lastTTPState     = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
const unsigned long TTP_DEBOUNCE  = 50;

// ===============================
// STATE MACHINE
// ===============================
enum State { STATE_SLEEP, STATE_ACTIVE, STATE_COOLDOWN };
State currentState = STATE_SLEEP;
unsigned long stateStartTime = 0;
unsigned long sesiStartTime  = 0;

// ===============================
// DATA
// ===============================
int currentServoAngle  = 90;
int servoRollValue     = 90;
int currentServo2Angle = 90;
int servoPitchValue    = 90;

int16_t ax, ay, az;

float axG = 0, ayG = 0, azG = 0;

float roll  = 0;
float pitch = 0;

String statusGetaran = "NORMAL";
bool wifiConnected   = false;

// ===============================
// BUFFER
// ===============================
#define MAX_BUFFER 1000

struct LogEntry {
  char          waktu[20];
  float         pitch;
  float         roll;
  int           servo;
  int           baterai;
  unsigned long durasiSesi;
};

LogEntry buffer[MAX_BUFFER];
int bufferCount = 0;
int bufferSent  = 0;

// ===============================
// SESI
// ===============================
int sesiPenggunaan = 1;
unsigned long durasiSesiIni = 0;
unsigned long totalDurasiSemuaSesi = 0;
float sumRoll = 0, sumPitch = 0;
int   sampleCount = 0;
float globalSumRoll = 0, globalSumPitch = 0;
int   globalSampleCount = 0;

Preferences prefs;
bool ntpSynced = false;

// =====================================================
// PROTOTYPES
// =====================================================
void fixWiFi();
void connectWiFi();
void disconnectWiFi();
void ensureWiFiConnected();
void syncNTPAtBoot();
void moveServo();
void readMPU();
void saveDataToBuffer();
bool flushBufferToSheets();
void flushAverageToFirebase();
void resetSesi();
void updateOLED();
void updateOLEDStandby();
void displaySerial();
String getWaktuWIB();
String formatDurasi(unsigned long detik);
bool tombolPressed();
bool checkTombolGesture();
void kirimResetKeFirebase();
void attachServo();
void detachServo();
void drawRightAligned(const char* text, int y);
void updateBatteryDecrease();
String getStatusGetaran(float avgRoll, float avgPitch);

// =====================================================
// HELPER
// =====================================================
void drawRightAligned(const char* text, int y) {
  int len = strlen(text) * 6;
  int x = 128 - len;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

// =====================================================
// TENTUKAN STATUS GETARAN (3 LEVEL)
// =====================================================
String getStatusGetaran(float avgRoll, float avgPitch) {
  float magnitudo = sqrt(avgPitch * avgPitch + avgRoll * avgRoll);

  if (magnitudo < THRESHOLD_NORMAL) {
    return "NORMAL";
  } else if (magnitudo < THRESHOLD_SEDANG) {
    return "SEDANG";
  } else {
    return "BAHAYA";
  }
}

// =====================================================
// FIX WIFI
// =====================================================
void fixWiFi() {
  Serial.print("[WiFi] Terapkan fix #");
  Serial.println(WIFI_SOLUSI);

#if (WIFI_SOLUSI == 3) || (WIFI_SOLUSI == 4)
  Serial.println("  [3] Bersihkan config WiFi lama...");
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(500);
#endif

  WiFi.mode(WIFI_STA);
  delay(100);

#if (WIFI_SOLUSI == 2) || (WIFI_SOLUSI == 4)
  Serial.println("  [2] Paksa protokol 802.11 b/g/n...");
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif

#if (WIFI_SOLUSI == 1) || (WIFI_SOLUSI == 4)
  Serial.println("  [1] TX power -> 8.5 dBm");
  esp_wifi_set_max_tx_power(34);
#endif
}

// =====================================================
// PENURUNAN BATERAI (-1% tiap 5 menit)
// =====================================================
void updateBatteryDecrease() {
  unsigned long now = millis();
  if (now - lastBatteryTick >= BATTERY_DECREASE_INTERVAL) {
    lastBatteryTick = now;
    if (batteryPercentage > 0) {
      batteryPercentage--;
      Serial.printf("[BATTERY] Turun ke %d%%\n", batteryPercentage);

      if (batteryPercentage == 20) {
        Serial.println("[BATTERY] LOW (20%) — perlu charging");
      }
      if (batteryPercentage == 10) {
        Serial.println("[BATTERY] CRITICAL (10%)");
      }
      if (batteryPercentage == 0) {
        Serial.println("[BATTERY] HABIS (0%)");
      }
    }
  }
}

// =====================================================
// SERVO ATTACH / DETACH
// =====================================================
void attachServo() {
  if (servoAttached) return;

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO2_PIN, OUTPUT);
  digitalWrite(SERVO2_PIN, LOW);
  delay(50);

  // D3 = MG90S (roll)
  myServoRoll.setPeriodHertz(50);
  myServoRoll.attach(SERVO_PIN, SERVO_ROLL_MIN_US, SERVO_ROLL_MAX_US);
  delay(30);
  myServoRoll.write(SUDUT_AWAL_SERVO);
  delay(150);

  // D2 = SG90S (pitch)
  myServoPitch.setPeriodHertz(50);
  myServoPitch.attach(SERVO2_PIN, SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US);
  delay(30);
  myServoPitch.write(SUDUT_AWAL_SERVO);
  delay(150);

  servoRollValue     = SUDUT_AWAL_SERVO;
  servoPitchValue    = SUDUT_AWAL_SERVO;
  currentServoAngle  = SUDUT_AWAL_SERVO;
  currentServo2Angle = SUDUT_AWAL_SERVO;
  servoRollPrev      = SUDUT_AWAL_SERVO;
  servoPitchPrev     = SUDUT_AWAL_SERVO;

  servoAttached = true;
  Serial.println("[SERVO] ATTACHED");
  Serial.printf("  D3 (MG90S) roll : pulse %d-%d us (map %d-%d)\n",
                SERVO_ROLL_MIN_US, SERVO_ROLL_MAX_US,
                SERVO_ROLL_MAP_MIN, SERVO_ROLL_MAP_MAX);
  Serial.printf("  D2 (SG90S) pitch: pulse %d-%d us (map %d-%d) [INVERT]\n",
                SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US,
                SERVO_PITCH_MAP_MIN, SERVO_PITCH_MAP_MAX);
}

void detachServo() {
  if (!servoAttached) return;
  myServoRoll.detach();
  myServoPitch.detach();
  digitalWrite(SERVO_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);
  delay(20);
  servoAttached = false;
  Serial.println("[SERVO] DETACHED (hemat daya)");
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n========================================");
  Serial.println("  PASTE - DUAL SERVO + OLED");
  Serial.println("  TANPA FILTER (accelerometer langsung)");
  Serial.println("  D3 (MG90S) = Kiri-Kanan (roll)");
  Serial.println("  D2 (SG90S) = Atas-Bawah (pitch) [INVERT]");
  Serial.println("  D1 = BUTTON | MPU 180 derajat");
  Serial.printf("  WiFi SOLUSI = %d\n", WIFI_SOLUSI);
  Serial.printf("  Threshold: NORMAL<%.1f SEDANG<%.1f BAHAYA>=\n",
                THRESHOLD_NORMAL, THRESHOLD_SEDANG);
  Serial.println("========================================");
  Serial.printf("[MEM] Free heap awal: %d bytes\n", ESP.getFreeHeap());

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // -------- OLED --------
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED gagal!");
    while (1) delay(10);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("PASTE Starting...");
  display.display();

  // -------- SERVO --------
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);
  servoAttached = false;
  Serial.println("Servo D3/D2 STANDBY (belum attach)");

  // -------- MPU6050 --------
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 GAGAL!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 ERROR");
    display.println("Cek wiring SDA/SCL");
    display.display();
    while (1) delay(10);
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // -------- TOMBOL --------
  pinMode(TTP_PIN, INPUT_PULLUP);
  Serial.println("Tombol D1: INPUT_PULLUP (tekan ke GND)");
  delay(50);
  Serial.printf("Status tombol awal: %s\n",
                digitalRead(TTP_PIN) == LOW ? "DITEKAN" : "LEPAS");

  // -------- Preferences --------
  prefs.begin("paste", false);
  sesiPenggunaan       = prefs.getInt("sesi", 0);
  totalDurasiSemuaSesi = prefs.getULong("total_durasi", 0);
  prefs.end();
  Serial.printf("Sesi terakhir: %d\n", sesiPenggunaan);
  Serial.printf("Total durasi semua sesi: %lu detik\n", totalDurasiSemuaSesi);

  bufferCount = 0;
  bufferSent  = 0;
  Serial.println("[BOOT] Buffer di-reset ke 0");

  // -------- Baterai awal (random 80-95%) --------
  randomSeed(analogRead(0));
  batteryPercentage = random(80, 96);
  Serial.printf("[BATTERY] Awal (random): %d%%\n", batteryPercentage);
  Serial.printf("[BATTERY] Turun 1%% tiap %lu detik (%lu menit)\n",
                BATTERY_DECREASE_INTERVAL / 1000,
                BATTERY_DECREASE_INTERVAL / 60000);
  lastBatteryTick = millis();

  syncNTPAtBoot();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  stateStartTime = millis();
  sesiStartTime  = millis();
  currentState = STATE_SLEEP;

  updateOLEDStandby();
  Serial.printf("[MEM] Free heap setelah setup: %d bytes\n", ESP.getFreeHeap());
  Serial.println("\n=== SIAP - WiFi ON, tekan tombol untuk mulai ===");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // Baterai turun 1% tiap 5 menit
  updateBatteryDecrease();

  // WiFi flag
  if (now - lastWiFiFlagCheck >= 1000) {
    lastWiFiFlagCheck = now;
    bool realWifi = (WiFi.status() == WL_CONNECTED);
    if (wifiConnected != realWifi) {
      Serial.printf("[WiFi Flag] %s -> %s\n",
                    wifiConnected ? "ON" : "OFF",
                    realWifi ? "ON" : "OFF");
    }
    wifiConnected = realWifi;
  }

  bool touch = tombolPressed();

  if (touch && currentState == STATE_SLEEP) {
    Serial.println("\n[BUTTON ON] WiFi OFF, ACTIVE! Servo ON");
    disconnectWiFi();
    resetSesi();

    attachServo();
    delay(300);

    sesiStartTime = now;
    currentState = STATE_ACTIVE;
    stateStartTime = now;
    lastSaveData = now;
    lastSensorRead = 0;
    lastOledUpdate = 0;
    return;
  }

  if (touch && currentState == STATE_COOLDOWN) {
    Serial.println("\n[BUTTON ON] ACTIVE (lanjut sesi)!");
    disconnectWiFi();

    if (!servoAttached) attachServo();

    currentState = STATE_ACTIVE;
    stateStartTime = now;
    lastSaveData = now;
    lastSensorRead = 0;
    lastOledUpdate = 0;
    return;
  }

  if (checkTombolGesture()) {
    Serial.println("\n*** RESET TOTAL ***");

    prefs.begin("paste", false);
    prefs.putInt("sesi", 0);
    prefs.putULong("durasi", 0);
    prefs.putULong("total_durasi", 0);
    prefs.end();

    bufferCount = 0;
    bufferSent  = 0;

    sumRoll = sumPitch = 0;
    sampleCount = 0;
    globalSumRoll = globalSumPitch = 0;
    globalSampleCount = 0;
    durasiSesiIni = totalDurasiSemuaSesi = 0;

    // Reset baterai: random lagi
    randomSeed(analogRead(0));
    batteryPercentage = random(80, 96);
    lastBatteryTick = millis();

    detachServo();

    ttpPressCount  = 0;
    ttpWindowStart = 0;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 8);
    display.println("RESET");
    display.setCursor(0, 30);
    display.println("TOTAL!");
    display.setTextSize(1);
    display.setCursor(0, 55);
    display.println("Sesi->0 Durasi->0");
    display.display();

    Serial.printf("[RESET] Sesi=0, Durasi=0, Total=0, Baterai=%d%%\n",
                  batteryPercentage);
    kirimResetKeFirebase();

    Serial.println("[RESET] Restart ESP32...");
    delay(2000);
    ESP.restart();
  }

  if (currentState == STATE_ACTIVE && now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readMPU();
  }

  if (currentState == STATE_ACTIVE && now - lastDebugLog >= DEBUG_INTERVAL) {
    lastDebugLog = now;
    Serial.printf("AX:%6d AY:%6d AZ:%6d | roll:%6.2f pitch:%6.2f | R(D3):%3d P(D2):%3d | B:%d%% | %s\n",
                  ax, ay, az, roll, pitch,
                  servoRollValue, servoPitchValue, batteryPercentage,
                  statusGetaran.c_str());
  }

  if (currentState == STATE_SLEEP) {
    if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
      lastWiFiCheck = now;
      ensureWiFiConnected();
    }
  }

  switch (currentState) {

    case STATE_SLEEP:
      if (servoAttached) detachServo();

      if (bufferCount > bufferSent) {
        if (now - lastSendAttempt >= SEND_RETRY_INTERVAL) {
          lastSendAttempt = now;

          if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            Serial.printf("\n[SLEEP] Auto kirim: %d sisa...\n",
                          bufferCount - bufferSent);
            bool selesai = flushBufferToSheets();

            if (selesai) {
              bufferCount = 0;
              bufferSent  = 0;
              Serial.println("[SLEEP] Buffer bersih");
            } else {
              Serial.printf("[SLEEP] Sheets belum selesai, sisa %d\n",
                            bufferCount - bufferSent);
            }

            Serial.println("[SLEEP] -> Coba kirim Firebase...");
            flushAverageToFirebase();
          } else {
            wifiConnected = false;
            Serial.println("[SLEEP] WiFi OFF, reconnect...");
            ensureWiFiConnected();
          }
        }
      }
      break;

    case STATE_ACTIVE:
      moveServo();
      if (now - lastSaveData >= SAVE_INTERVAL) {
        lastSaveData = now;
        saveDataToBuffer();
      }
      if (!touch) {
        currentState   = STATE_COOLDOWN;
        stateStartTime = now;
        Serial.println("\n[BUTTON OFF] Cooldown 3s...");
      }
      break;

    case STATE_COOLDOWN:
      moveServo();
      if (now - lastSaveData >= SAVE_INTERVAL) {
        lastSaveData = now;
        saveDataToBuffer();
      }

      if (now - stateStartTime >= COOLDOWN_DURATION) {
        detachServo();

        Serial.println("\n[IDLE] Cooldown selesai, balik ke STANDBY (servo OFF)");

        currentState    = STATE_SLEEP;
        stateStartTime  = now;
        lastSendAttempt = 0;
        lastOledUpdate  = 0;

        updateOLEDStandby();

        Serial.printf("[IDLE] Buffer total: %d, Pending: %d, Durasi: %lu s\n",
                      bufferCount, bufferCount - bufferSent, durasiSesiIni);
      }
      break;
  }

  if (now - lastOledUpdate >= OLED_INTERVAL) {
    lastOledUpdate = now;
    if (currentState == STATE_SLEEP) updateOLEDStandby();
    else                             updateOLED();
  }

  if (now - lastSerialLog >= SERIAL_INTERVAL) {
    lastSerialLog = now;
    if (currentState == STATE_ACTIVE || currentState == STATE_COOLDOWN) {
      displaySerial();
    }
  }
}

// =====================================================
// KIRIM RESET KE FIREBASE
// =====================================================
void kirimResetKeFirebase() {
  Serial.println("[RESET-FB] Kirim reset ke Firebase...");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[RESET-FB] WiFi OFF, connect dulu...");
    connectWiFi();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[RESET-FB] WiFi gagal, skip reset Firebase");
    return;
  }

  String resetJson = "{";
  resetJson += "\"device\":{";
  resetJson += "\"status_koneksi\":\"TERHUBUNG\",";
  resetJson += "\"status_data\":\"RESET\",";
  resetJson += "\"baterai\":\"100 %\",";
  resetJson += "\"peringatan_baterai\":\"STABIL\"";
  resetJson += "},";
  resetJson += "\"penggunaan\":{";
  resetJson += "\"sesi\":0,";
  resetJson += "\"jumlah_sample\":0,";
  resetJson += "\"durasi\":0,";
  resetJson += "\"durasi_format\":\"0s\"";
  resetJson += "},";
  resetJson += "\"getaran\":{";
  resetJson += "\"pitch\":0,";
  resetJson += "\"roll\":0,";
  resetJson += "\"status\":\"NORMAL\"";
  resetJson += "}";
  resetJson += "}";

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10);

  HTTPClient http;
  http.setTimeout(8000);
  http.begin(client, FIREBASE_URL);
  http.addHeader("Content-Type", "application/json");

  int code = http.PUT(resetJson);
  http.end();

  if (code > 0) {
    Serial.printf("[RESET-FB] OK HTTP %d\n", code);
  } else {
    Serial.printf("[RESET-FB] GAGAL code=%d\n", code);
  }
}

// =====================================================
// CEK GESTURE TOMBOL 10x
// =====================================================
bool checkTombolGesture() {
  bool currentStatePin = digitalRead(TTP_PIN);
  bool result = false;

  if (currentStatePin != lastTTPState && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();

    if (currentStatePin == LOW && lastTTPState == HIGH) {
      unsigned long now = millis();

      if (ttpWindowStart == 0 || (now - ttpWindowStart) > TTP_RESET_WINDOW) {
        ttpPressCount = 1;
        ttpWindowStart = now;
      } else {
        ttpPressCount++;
      }

      if (ttpPressCount >= TTP_RESET_COUNT) {
        result = true;
        ttpPressCount = 0;
        ttpWindowStart = 0;
      }
    }

    lastTTPState = currentStatePin;
  }

  return result;
}

// =====================================================
// ENSURE WIFI
// =====================================================
void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return;
  }
  Serial.print("[WiFi] Reconnect");
  fixWiFi();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(50);
    Serial.print(".");
    attempts++;
    if (digitalRead(TTP_PIN) == LOW) {
      Serial.println(" CUT");
      disconnectWiFi();
      return;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    wifiConnected = true;
  } else {
    Serial.println(" FAILED");
    wifiConnected = false;
  }
}

// =====================================================
// NTP BOOT SYNC
// =====================================================
void syncNTPAtBoot() {
  Serial.print("NTP boot sync");
  fixWiFi();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(100);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" WiFi OK");
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    wifiConnected = true;
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("NTP: sync");

    struct tm ti;
    int retry = 0;
    while (!getLocalTime(&ti, 800) && retry < 10) {
      Serial.print(".");
      retry++;
    }
    if (retry < 10) {
      Serial.println(" OK (WIB)");
      ntpSynced = true;
      char buf[30];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
      Serial.print("Waktu: "); Serial.println(buf);
    } else {
      Serial.println(" GAGAL");
    }
  } else {
    Serial.println(" WiFi GAGAL");
    wifiConnected = false;
  }
}

// =====================================================
// CEK TOMBOL (PUSH BUTTON, INPUT_PULLUP)
// =====================================================
bool tombolPressed() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastDebounce = 0;

  bool reading = digitalRead(TTP_PIN);

  if (reading != lastReading) {
    lastDebounce = millis();
  }

  if ((millis() - lastDebounce) > TTP_DEBOUNCE) {
    stableState = reading;
  }

  lastReading = reading;
  return stableState == LOW;
}

// =====================================================
// RESET SESI
// =====================================================
void resetSesi() {
  if (bufferCount == 0) bufferSent = 0;

  if (durasiSesiIni > 0) {
    totalDurasiSemuaSesi += durasiSesiIni;
    Serial.printf("[DURASI] Sesi sebelumnya: %lu s, Total: %lu s\n",
                  durasiSesiIni, totalDurasiSemuaSesi);
  }

  sesiPenggunaan++;
  prefs.begin("paste", false);
  prefs.putInt("sesi", sesiPenggunaan);
  prefs.putULong("total_durasi", totalDurasiSemuaSesi);
  prefs.end();

  sumRoll = sumPitch = 0;
  sampleCount = 0;
  durasiSesiIni = 0;

  Serial.printf("=== SESI BARU #%d (buffer: %d) ===\n",
                sesiPenggunaan, bufferCount);
}

// =====================================================
// WAKTU WIB
// =====================================================
String getWaktuWIB() {
  struct tm ti;
  if (getLocalTime(&ti, 50)) {
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
    return String(buf);
  }
  return "N/A";
}

// =====================================================
// FORMAT DURASI
// =====================================================
String formatDurasi(unsigned long detik) {
  unsigned long jam   = detik / 3600;
  unsigned long menit = (detik % 3600) / 60;
  unsigned long sisa  = detik % 60;

  String hasil = "";
  if (jam > 0)   hasil += String(jam) + "j ";
  if (menit > 0) hasil += String(menit) + "m ";
  hasil += String(sisa) + "s";
  return hasil;
}

// =====================================================
// READ MPU — TANPA FILTER (accelerometer langsung)
//   D3 (MG90S) -> roll : -ax
//   D2 (SG90S) -> pitch: +ay  [INVERT — sudah dibalik]
// =====================================================
void readMPU() {
  mpu.getAcceleration(&ax, &ay, &az);

  axG = ax / 16384.0;
  ayG = ay / 16384.0;
  azG = az / 16384.0;

  // ROLL -> D3 (MG90S): pakai -ax
  float rollRaw  = (-ax / 16384.0) * 90.0;
  rollRaw = constrain(rollRaw, -MAX_TILT_DEG, MAX_TILT_DEG);

  // PITCH -> D2 (SG90S): pakai +ay (INVERT dari -ay)
  float pitchRaw = ( ay / 16384.0) * 90.0;
  pitchRaw = constrain(pitchRaw, -MAX_TILT_DEG, MAX_TILT_DEG);

  // Deadzone
  if (fabs(rollRaw)  < ROLL_DEADZONE)  rollRaw  = 0;
  if (fabs(pitchRaw) < PITCH_DEADZONE) pitchRaw = 0;

  roll  = rollRaw;
  pitch = pitchRaw;

  // D3 (MG90S) -> ROLL/kiri-kanan
  int rollInt = map((int)roll, -(int)MAX_TILT_DEG, (int)MAX_TILT_DEG,
                    SERVO_ROLL_MAP_MIN, SERVO_ROLL_MAP_MAX);
  servoRollValue = constrain(rollInt, SERVO_ROLL_MAP_MIN, SERVO_ROLL_MAP_MAX);
  currentServoAngle = servoRollValue;

  // D2 (SG90S) -> PITCH/atas-bawah
  int pitchInt = map((int)pitch, -(int)MAX_TILT_DEG, (int)MAX_TILT_DEG,
                     SERVO_PITCH_MAP_MIN, SERVO_PITCH_MAP_MAX);
  servoPitchValue = constrain(pitchInt, SERVO_PITCH_MAP_MIN, SERVO_PITCH_MAP_MAX);
  currentServo2Angle = servoPitchValue;

  // Status getaran: 3 level
  statusGetaran = getStatusGetaran(roll, pitch);
}

// =====================================================
// SERVO MOVE — speed limit
// =====================================================
void moveServo() {
  if (!servoAttached) return;

  if (servoRollValue > servoRollPrev + MAX_STEP) {
    servoRollValue = servoRollPrev + MAX_STEP;
  } else if (servoRollValue < servoRollPrev - MAX_STEP) {
    servoRollValue = servoRollPrev - MAX_STEP;
  }

  if (servoPitchValue > servoPitchPrev + MAX_STEP) {
    servoPitchValue = servoPitchPrev + MAX_STEP;
  } else if (servoPitchValue < servoPitchPrev - MAX_STEP) {
    servoPitchValue = servoPitchPrev - MAX_STEP;
  }

  myServoRoll.write(servoRollValue);
  myServoPitch.write(servoPitchValue);

  servoRollPrev  = servoRollValue;
  servoPitchPrev = servoPitchValue;
}

// =====================================================
// SIMPAN BUFFER
// =====================================================
void saveDataToBuffer() {
  if (bufferCount >= MAX_BUFFER) {
    for (int i = 0; i < MAX_BUFFER - 1; i++) {
      buffer[i] = buffer[i + 1];
    }
    bufferCount = MAX_BUFFER - 1;
    if (bufferSent > 0) bufferSent--;
  }

  durasiSesiIni = (millis() - sesiStartTime) / 1000;

  char waktuBuf[20] = "N/A";
  struct tm ti;
  if (getLocalTime(&ti, 50)) {
    strftime(waktuBuf, sizeof(waktuBuf), "%Y-%m-%d %H:%M:%S", &ti);
  }

  LogEntry &e = buffer[bufferCount];
  strncpy(e.waktu, waktuBuf, sizeof(e.waktu) - 1);
  e.waktu[sizeof(e.waktu) - 1] = '\0';
  e.pitch      = pitch;
  e.roll       = roll;
  e.servo      = currentServoAngle;
  e.baterai    = batteryPercentage;
  e.durasiSesi = durasiSesiIni;
  bufferCount++;

  sumRoll  += roll;
  sumPitch += pitch;
  sampleCount++;
  globalSumRoll  += roll;
  globalSumPitch += pitch;
  globalSampleCount++;
}

// =====================================================
// KIRIM KE SHEETS
// =====================================================
bool flushBufferToSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[FLUSH] SKIP: WiFi tidak terhubung");
    wifiConnected = false;
    return (bufferSent >= bufferCount);
  }
  if (bufferCount == 0) return true;

  unsigned long startFlush = millis();

  Serial.printf("[FLUSH] MASUK: bufferSent=%d, bufferCount=%d, batch=%d\n",
                bufferSent, bufferCount, BATCH_SIZE);
  Serial.printf("[MEM] Free heap sebelum kirim: %d bytes\n", ESP.getFreeHeap());

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15);

  int totalOK = 0, totalFail = 0;

  while (bufferSent < bufferCount) {
    if (digitalRead(TTP_PIN) == LOW) {
      Serial.println("\n[FLUSH] Tombol ditekan - STOP!");
      disconnectWiFi();
      return false;
    }

    int end = min(bufferSent + BATCH_SIZE, bufferCount);
    String json = "[";

    for (int i = bufferSent; i < end; i++) {
      LogEntry &e = buffer[i];
      if (i > bufferSent) json += ",";
      json += "{";
      json += "\"waktu\":\"" + String(e.waktu) + "\",";
      json += "\"sesi\":" + String(sesiPenggunaan) + ",";
      json += "\"pitch\":" + String(e.pitch, 2) + ",";
      json += "\"roll\":" + String(e.roll, 2) + ",";
      json += "\"servo\":" + String(e.servo) + ",";
      json += "\"baterai\":\"" + String(e.baterai) + " %\",";
      json += "\"durasi_sesi\":" + String(e.durasiSesi);
      json += "}";
    }
    json += "]";

    unsigned long tStart = millis();

    HTTPClient http;
    http.setTimeout(8000);
    http.begin(client, SHEETS_URL);
    http.addHeader("Content-Type", "application/json");
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    int code = http.POST(json);
    unsigned long tEnd = millis() - tStart;

    if (code > 0) {
      int jumlah = end - bufferSent;
      bufferSent = end;
      totalOK += jumlah;
      Serial.printf("  Batch [%d-%d] OK (%d data, %lu ms), sent=%d/%d\n",
                    end - jumlah, end - 1, jumlah, tEnd, bufferSent, bufferCount);
    } else {
      totalFail += (end - bufferSent);
      Serial.printf("  Batch GAGAL (%.1f s): %s\n",
                    tEnd / 1000.0, http.errorToString(code).c_str());
      http.end();
      break;
    }
    http.end();
    delay(30);
  }

  unsigned long totalTime = millis() - startFlush;
  Serial.printf("[FLUSH] KELUAR: %d OK, %d GAGAL, total %lu ms\n",
                totalOK, totalFail, totalTime);

  return (bufferSent >= bufferCount);
}

// =====================================================
// FIREBASE (struktur lama: pitch, roll, status)
// =====================================================
void flushAverageToFirebase() {
  Serial.printf("[FB] Cek: WiFi=%d, sample=%d, tombol=%d\n",
                WiFi.status(), globalSampleCount, digitalRead(TTP_PIN));

  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    Serial.println("[FB] SKIP: WiFi tidak terhubung");
    return;
  }

  if (globalSampleCount == 0) {
    Serial.println("[FB] SKIP: belum ada sample");
    return;
  }

  if (digitalRead(TTP_PIN) == LOW) {
    Serial.println("[FB] SKIP: tombol sedang ditekan");
    return;
  }

  float avgRoll  = globalSumRoll / globalSampleCount;
  float avgPitch = globalSumPitch / globalSampleCount;

  // Status 3 level berdasarkan magnitudo
  String statusAvg = getStatusGetaran(avgRoll, avgPitch);

  String peringatanBaterai = (batteryPercentage <= 20) ?
                              "BUTUH MENGISI DAYA" : "STABIL";

  unsigned long durasiTotal = totalDurasiSemuaSesi;
  if (durasiSesiIni > 0) durasiTotal += durasiSesiIni;

  // JSON struktur LAMA
  String json = "{";
  json += "\"device\":{";
  json += "\"status_koneksi\":\"TERHUBUNG\",";
  json += "\"status_data\":\"TERKIRIM\",";
  json += "\"baterai\":\"" + String(batteryPercentage) + " %\",";
  json += "\"peringatan_baterai\":\"" + peringatanBaterai + "\"";
  json += "},";
  json += "\"penggunaan\":{";
  json += "\"sesi\":" + String(sesiPenggunaan) + ",";
  json += "\"jumlah_sample\":" + String(globalSampleCount) + ",";
  json += "\"durasi\":" + String(durasiTotal) + ",";
  json += "\"durasi_format\":\"" + formatDurasi(durasiTotal) + "\"";
  json += "},";
  json += "\"getaran\":{";
  json += "\"pitch\":" + String(avgPitch, 2) + ",";
  json += "\"roll\":" + String(avgRoll, 2) + ",";
  json += "\"status\":\"" + statusAvg + "\"";
  json += "}";
  json += "}";

  Serial.printf("[FB] Mengirim %d byte... (durasi=%lu s / avgRoll=%.2f avgPitch=%.2f / status=%s)\n",
                json.length(), durasiTotal, avgRoll, avgPitch, statusAvg.c_str());
  Serial.println("[FB] JSON: " + json);

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10);

  HTTPClient http;
  http.setTimeout(8000);
  http.begin(client, FIREBASE_URL);
  http.addHeader("Content-Type", "application/json");

  int code = http.PUT(json);
  String response = http.getString();
  http.end();

  if (code > 0) {
    Serial.printf("[FB] OK HTTP %d\n", code);
    globalSumRoll = 0;
    globalSumPitch = 0;
    globalSampleCount = 0;
  } else {
    Serial.printf("[FB] GAGAL code=%d (%s)\n",
                  code, http.errorToString(code).c_str());
  }
}

// =====================================================
// WIFI
// =====================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; return; }
  Serial.print("WiFi: connecting");
  fixWiFi();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(50);
    Serial.print(".");
    attempts++;
    if (digitalRead(TTP_PIN) == LOW) {
      Serial.println(" CUT");
      disconnectWiFi();
      return;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    wifiConnected = true;
  } else {
    Serial.println(" FAILED");
    wifiConnected = false;
  }
}

void disconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
}

// =====================================================
// OLED ACTIVE
// =====================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("PASTE RAW");

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 14);
  display.print("Roll :");
  display.print(roll, 1);
  display.print(" R(D3):");
  display.print(servoRollValue);

  display.setCursor(0, 26);
  display.print("Pitch:");
  display.print(pitch, 1);
  display.print(" P(D2):");
  display.print(servoPitchValue);

  display.setCursor(0, 38);
  display.print("Status: ");
  display.print(statusGetaran);

  display.setCursor(0, 50);
  display.print("D:");
  display.print(durasiSesiIni);
  display.print("s B:");
  display.print(batteryPercentage);
  display.print("% S:");
  display.print(sesiPenggunaan);

  display.display();
}

// =====================================================
// OLED STANDBY
// =====================================================
void updateOLEDStandby() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("PASTE");
  display.setTextSize(1);

  display.setCursor(0, 22);
  display.println("Tekan tombol");
  display.setCursor(0, 32);
  display.println("untuk mulai");

  display.drawLine(0, 44, 127, 44, SSD1306_WHITE);

  display.setCursor(0, 48);
  display.print("Btr:");
  display.print(batteryPercentage);
  display.print("% S:");
  display.print(sesiPenggunaan);
  display.print(" ");

  if (batteryPercentage <= 20) {
    display.print("LOW!");
  } else {
    display.print(WiFi.status() == WL_CONNECTED ? "W:ON" : "W:OFF");
  }

  display.setCursor(0, 56);
  int pending = bufferCount - bufferSent;
  display.print("Buf:");
  display.print(bufferCount);
  display.print(" Blm:");
  display.print(pending);

  display.display();
}

// =====================================================
// SERIAL
// =====================================================
void displaySerial() {
  Serial.println("\n--------- PASTE (RAW) ---------");
  Serial.print("Waktu  : "); Serial.println(getWaktuWIB());
  Serial.print("Sesi   : "); Serial.println(sesiPenggunaan);
  Serial.print("Durasi : "); Serial.print(durasiSesiIni); Serial.println(" s");
  Serial.print("Total Durasi: "); Serial.print(totalDurasiSemuaSesi); Serial.println(" s");
  Serial.print("AX     : "); Serial.print(ax);
  Serial.print("  AY : "); Serial.print(ay);
  Serial.print("  AZ : "); Serial.println(az);
  Serial.print("Roll    : "); Serial.print(roll, 2);
  Serial.print(" deg -> Servo D3 (MG90S): "); Serial.println(servoRollValue);
  Serial.print("Pitch   : "); Serial.print(pitch, 2);
  Serial.print(" deg -> Servo D2 (SG90S): "); Serial.println(servoPitchValue);
  Serial.print("Status  : "); Serial.println(statusGetaran);
  Serial.print("Baterai : "); Serial.print(batteryPercentage); Serial.println(" %");
  Serial.print("WiFi    : "); Serial.println(WiFi.status() == WL_CONNECTED ? "ON" : "OFF");
  Serial.print("Buffer  : "); Serial.print(bufferCount);
  Serial.print("/"); Serial.print(MAX_BUFFER);
  Serial.print(" (sent: "); Serial.print(bufferSent); Serial.println(")");
  Serial.print("Servo   : "); Serial.println(servoAttached ? "ON" : "OFF (standby)");
  Serial.print("Tombol  : "); Serial.println(digitalRead(TTP_PIN) == LOW ? "DITEKAN" : "LEPAS");
  Serial.print("Free RAM: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
  Serial.println("-------------------------");
}