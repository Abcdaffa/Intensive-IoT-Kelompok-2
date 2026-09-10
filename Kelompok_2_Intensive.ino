#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// ================= KONFIGURASI WiFi & FIREBASE =================
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
#define FIREBASE_HOST "your_project.firebaseio.com"  // atau URL Realtime Database
#define FIREBASE_AUTH "your_database_secret"         // atau API Key

// ================= PIN & OBJEK =================
MPU6050 mpu;
Servo servoPitch;
Servo servoRoll;

const int pinServoPitch = 18;
const int pinServoRoll  = 19;
const int pinTouch      = 4;   // TTP223

// ================= VARIABEL KALIBRASI =================
int offsetPitch = 90;
int offsetRoll  = 90;

// ================= STATE MACHINE =================
enum State {
  STATE_IDLE,          // servo mati, wifi on
  STATE_ACTIVE,        // servo aktif, wifi off
  STATE_COOLDOWN       // servo masih aktif, wifi off (tahan 2-5 detik)
};

State currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
const unsigned long COOLDOWN_DURATION = 3000; // 3 detik (2-5 detik bisa diatur)

// ================= VARIABEL UNTUK DATA TREMOR =================
float currentAmplitude = 0.0;
float currentFrequency = 0.0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 10; // baca sensor setiap 10 ms (100 Hz)

// ================= FIREBASE OBJECTS =================
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool wifiConnected = false;
bool firebaseReady = false;

// ================= FUNGSI =================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Menghubungkan WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(100);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    wifiConnected = true;
  } else {
    Serial.println(" GAGAL");
    wifiConnected = false;
  }
}

void disconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiConnected = false;
    Serial.println("WiFi dimatikan.");
  }
}

void initFirebase() {
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  firebaseReady = true;
}

void sendDataToFirebase(float amp, float freq, int status) {
  if (!wifiConnected) return;
  if (!firebaseReady) {
    initFirebase();
  }
  
  FirebaseJson json;
  json.set("amplitude", amp);
  json.set("frequency", freq);
  json.set("status", status); // 0=idle, 1=active, 2=cooldown
  
  String path = "/tremor_data/latest";
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("Data terkirim ke Firebase.");
  } else {
    Serial.print("Gagal kirim: ");
    Serial.println(fbdo.errorReason());
  }
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Hitung amplitudo (magnitude akselerasi)
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  currentAmplitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  
  // Hitung frekuensi sederhana dari gyro (untuk demonstrasi)
  // Ambil magnitude gyro (derajat/detik)
  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;
  float gyroMag = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
  
  // Estimasi frekuensi (sederhana) = perubahan gyro per detik?
  // Untuk simulasi, kita pakai nilai absolut gyro sebagai proxy frekuensi
  // Bisa diimprove dengan FFT, tapi untuk demo cukup.
  static float prevGyroMag = 0;
  static unsigned long lastGyroTime = 0;
  unsigned long now = millis();
  if (now - lastGyroTime > 100) {
    float delta = gyroMag - prevGyroMag;
    float freqEst = abs(delta) * 10; // skala kasar
    currentFrequency = constrain(freqEst, 0.1, 20.0);
    prevGyroMag = gyroMag;
    lastGyroTime = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("=== Sistem Anti-Tremor dengan Firebase ===");

  // Touch sensor
  pinMode(pinTouch, INPUT);

  // Servo
  servoPitch.attach(pinServoPitch);
  servoRoll.attach(pinServoRoll);
  servoPitch.write(offsetPitch);
  servoRoll.write(offsetRoll);
  Serial.println("Servo netral.");

  // MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 OK.");
  } else {
    Serial.println("MPU6050 GAGAL!");
  }

  // WiFi & Firebase (aktif awal)
  connectWiFi();
  if (wifiConnected) {
    initFirebase();
  }

  stateStartTime = millis();
}

void loop() {
  unsigned long now = millis();

  // === BACA SENSOR dengan interval non-blocking ===
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensor();
  }

  // === BACA TOUCH SENSOR ===
  int touch = digitalRead(pinTouch); // HIGH = dipegang

  // === STATE MACHINE ===
  switch (currentState) {
    case STATE_IDLE:
      // Servo mati (netral), WiFi on
      if (touch == HIGH) {
        // Mulai aktif
        currentState = STATE_ACTIVE;
        stateStartTime = now;
        Serial.println("STATE: ACTIVE (dipegang)");
        // Matikan WiFi agar tidak mengganggu servo
        disconnectWiFi();
      } else {
        // Tetap idle, kirim data periodik (setiap 5 detik)
        static unsigned long lastSend = 0;
        if (now - lastSend > 5000) {
          lastSend = now;
          connectWiFi();
          if (wifiConnected) {
            sendDataToFirebase(currentAmplitude, currentFrequency, 0);
          }
          // Matikan WiFi lagi? Biarkan saja, nanti disconnect di state lain.
          // Untuk efisiensi, matikan setelah kirim
          disconnectWiFi();
        }
      }
      break;

    case STATE_ACTIVE:
      // Servo aktif, WiFi off
      servoPitch.write(map(ax, -17000, 17000, 45, 135)); // sesuaikan
      servoRoll.write(map(ay, -17000, 17000, 45, 135));
      
      if (touch == LOW) {
        // Mulai cooldown
        currentState = STATE_COOLDOWN;
        stateStartTime = now;
        Serial.println("STATE: COOLDOWN (mulai hitung 3 detik)");
      }
      break;

    case STATE_COOLDOWN:
      // Servo tetap aktif, WiFi off, selama COOLDOWN_DURATION
      // Tetap baca sensor & gerakkan servo
      servoPitch.write(map(ax, -17000, 17000, 45, 135));
      servoRoll.write(map(ay, -17000, 17000, 45, 135));

      if (now - stateStartTime >= COOLDOWN_DURATION) {
        // Waktu habis, kembali ke idle
        currentState = STATE_IDLE;
        stateStartTime = now;
        Serial.println("STATE: IDLE (cooldown selesai)");
        // Kembalikan servo ke netral
        servoPitch.write(offsetPitch);
        servoRoll.write(offsetRoll);
        // Nyalakan WiFi untuk kirim data
        connectWiFi();
        if (wifiConnected) {
          sendDataToFirebase(currentAmplitude, currentFrequency, 2); // status cooldown
        }
        disconnectWiFi(); // bisa dimatikan lagi atau biarkan on
      }
      break;
  }

  // Tidak ada delay() di loop!
}