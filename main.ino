/*
  Day 3 - Following-Axis Servo Motor and Motion Detection
  Nama : Muhammad Daffa Izdihar
  NIM  : 25/560183/SV/26399

  Catatan dan asumsi:
  - MPU6050 dipakai sebagai pengganti BNO055 di simulasi Wokwi.
  - Komunikasi MPU6050 menggunakan I2C (SDA 21, SCL 22).
  - Roll dan Pitch dihitung dari accelerometer.
  - Yaw dihitung dari integrasi gyro Z.
  - Posisi awal semua servo adalah 0 derajat (1500 us).
  - Range servo dibatasi dari -90 sampai +90 derajat.
  - PWM servo menggunakan frekuensi 50 Hz dan pulse 1000-2000 us.
  State Machine Yaw:
  - IDLE: menunggu gerakan rotasi.
  - MOVING: servo mengikuti perubahan Yaw.
  - STOP_CHECK: memastikan rotasi benar-benar berhenti.
  - HOLD: menahan posisi selama 1 detik.
  - RETURN: mengembalikan servo ke 0 derajat.
  State Machine PIR:
  - IDLE: menunggu deteksi gerakan.
  - MOVE: semua servo bergerak ke 45 derajat selama 600 ms.
  - RETURN: semua servo kembali ke 0 derajat selama 500 ms.
  Semua timer menggunakan millis(), jadi tidak ada delay().
*/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <math.h>

Adafruit_MPU6050 mpu;
Servo servos[5];
// Pin servo: Roll 1-2, Pitch 3-4, Yaw 5
const int SERVO_PINS[5] = {18, 19, 23, 25, 26};
const int SERVO_MIN = 1000;
const int SERVO_MAX = 2000;
const unsigned long SERVO_INTERVAL = 20;
const unsigned long IMU_INTERVAL = 10;
const unsigned long PRINT_INTERVAL = 500;
const unsigned long STOP_CONFIRM_MS = 100;
const unsigned long YAW_HOLD_MS = 1000;
const unsigned long PIR_MOVE_MS = 600;
const unsigned long PIR_RETURN_MS = 500;
// Variabel pembacaan sensor
float roll = 0;
float pitch = 0;
float yaw = 0;
float gyroZ = 0;
float testGyroZ = 0;
// Status servo untuk monitoring
float servoAngles[5] = {0, 0, 0, 0, 0};
int servoPWM[5] = {1500, 1500, 1500, 1500, 1500};
// Mode pengujian lewat Serial Monitor
bool testMode = false;
bool testYawMode = false;
bool lastPIRState = false;
// Timer masing-masing proses
unsigned long lastIMUTime = 0;
unsigned long lastServoTime = 0;
unsigned long lastPrintTime = 0;
unsigned long yawStartTime = 0;
unsigned long pirStartTime = 0;

// State untuk pergerakan Yaw
enum YawState {
  YAW_IDLE,
  YAW_MOVING,
  YAW_STOP_CHECK,
  YAW_HOLD,
  YAW_RETURN
};
// State untuk deteksi PIR
enum PIRState {
  PIR_IDLE,
  PIR_MOVE,
  PIR_RETURN
};
YawState yawState = YAW_IDLE;
PIRState pirState = PIR_IDLE;

// Mengubah sudut menjadi PWM, lalu menggerakkan servo
void moveServo(int index, float angle) {
  angle = constrain(angle, -90, 90);
  int pwm = round(1500 + (angle / 90.0) * 500);
  servos[index].writeMicroseconds(pwm);
  servoAngles[index] = angle;
  servoPWM[index] = pwm;
}

// Membaca data accelerometer dan gyroscope MPU6050
void readIMU(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Menghitung Roll dan Pitch dari accelerometer
  roll = atan2(a.acceleration.y, a.acceleration.z)
         * 180.0 / PI;
  pitch = atan2(
    -a.acceleration.x,
    sqrt(a.acceleration.y * a.acceleration.y +
         a.acceleration.z * a.acceleration.z)
  ) * 180.0 / PI;
  // Gyroscope Z menghasilkan kecepatan rotasi dalam rad/s
  gyroZ = g.gyro.z;
  // Mode test untuk simulasi Roll dan Pitch
  if (testMode) {
    roll = 30;
    pitch = -20;
  }
  // Mode test untuk simulasi gerakan Yaw
  if (testYawMode) {
    gyroZ = testGyroZ;
  }
  // Yaw hanya diintegrasikan saat IDLE atau MOVING
  // Saat HOLD dan RETURN, posisi Yaw harus tetap
  if (yawState == YAW_IDLE || yawState == YAW_MOVING) {
    yaw += gyroZ * dt * 180.0 / PI;
    yaw = constrain(yaw, -90, 90);
  }
}


// Mengatur pergerakan Yaw menggunakan state machine
void updateYaw() {
  unsigned long now = millis();
  // Mendeteksi apakah sensor sedang berotasi
  if (fabs(gyroZ) > 0.05) {
    if (yawState == YAW_IDLE ||
        yawState == YAW_STOP_CHECK) {
      yawState = YAW_MOVING;
    }
  }
  else if (yawState == YAW_MOVING) {
    yawState = YAW_STOP_CHECK;
    yawStartTime = now;
  }
  // Mengatur perpindahan antar-state
  switch (yawState) {
    case YAW_STOP_CHECK:
      if (now - yawStartTime >= STOP_CONFIRM_MS) {
        yawState = YAW_HOLD;
        yawStartTime = now;
      }
      break;
    case YAW_HOLD:
      if (now - yawStartTime >= YAW_HOLD_MS) {
        yawState = YAW_RETURN;
      }
      break;
    case YAW_RETURN:
      yaw = 0;
      yawState = YAW_IDLE;
      break;
    default:
      break;
  }
}


// Membaca PIR dan mengatur state deteksi gerakan
void updatePIR() {
  unsigned long now = millis();
  bool pirDetected = digitalRead(27);
  // Trigger hanya saat sinyal berubah dari LOW ke HIGH
  if (pirDetected && !lastPIRState &&
      pirState == PIR_IDLE) {
    pirState = PIR_MOVE;
    pirStartTime = now;
  }
  lastPIRState = pirDetected;
  switch (pirState) {
    case PIR_MOVE:
      if (now - pirStartTime >= PIR_MOVE_MS) {
        pirState = PIR_RETURN;
        pirStartTime = now;
      }
      break;
    case PIR_RETURN:
      if (now - pirStartTime >= PIR_RETURN_MS) {
        pirState = PIR_IDLE;
      }
      break;
    default:
      break;
  }
}


// Mengatur posisi kelima servo berdasarkan sensor
void updateServos() {
  unsigned long now = millis();
  // Update servo setiap 20 ms (50 Hz)
  if (now - lastServoTime < SERVO_INTERVAL) return;
  lastServoTime = now;
  // PIR punya prioritas tertinggi: semua servo ke 45 derajat
  if (pirState == PIR_MOVE) {
    for (int i = 0; i < 5; i++) {
      moveServo(i, 45);
    }
    return;
  }
  // Setelah gerakan PIR, semua servo kembali ke 0 derajat
  if (pirState == PIR_RETURN) {
    for (int i = 0; i < 5; i++) {
      moveServo(i, 0);
    }
    return;
  }
  // Roll: Servo 1 dan 2 bergerak berlawanan arah
  moveServo(0, -roll);
  moveServo(1, -roll);
  // Pitch: Servo 3 dan 4 mengikuti arah Pitch
  moveServo(2, pitch);
  moveServo(3, pitch);
  // Yaw: Servo 5 mengikuti rotasi dan kembali ke 0
  if (yawState == YAW_IDLE ||
      yawState == YAW_RETURN) {
    moveServo(4, 0);
  }
  else {
    moveServo(4, yaw);
  }
}

// Menampilkan nilai sensor dan posisi servo
void printStatus() {
  unsigned long now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL) return;
  lastPrintTime = now;
  Serial.printf(
    "Roll: %.1f | Pitch: %.1f | Yaw: %.1f\n",
    roll, pitch, yaw
  );
  for (int i = 0; i < 5; i++) {
    Serial.printf(
      "Servo %d: %.1f deg | %d us\n",
      i + 1, servoAngles[i], servoPWM[i]
    );
  }
  Serial.println("------------------------");
}

// Membaca perintah dari Serial Monitor
void readSerialCommand() {
  static String command = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      command.trim();
      if (command == "test") {
        testMode = true;
        Serial.println("Roll/Pitch test mode ON");
      }
      else if (command == "sensor") {
        testMode = false;
        testYawMode = false;
        Serial.println("Sensor mode ON - MPU6050 active");
      }
      else if (command.startsWith("yaw ")) {
        float value = command.substring(4).toFloat();
        testYawMode = true;
        testGyroZ = value * PI / 180.0;
        Serial.printf(
          "Yaw test mode: %.1f deg/sec\n", value
        );
      }
      command = "";
    }
    else {
      command += c;
    }
  }
}

// Inisialisasi sensor, pin, dan servo
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pinMode(27, INPUT);
  if (!mpu.begin()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (true) {}
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Semua servo diinisialisasi pada posisi 0 derajat
  for (int i = 0; i < 5; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(
      SERVO_PINS[i], SERVO_MIN, SERVO_MAX
    );
    moveServo(i, 0);
  }
  Serial.println("Day 3 System Ready!");
  Serial.println("Commands: test | sensor | yaw <value>");
}


// Main loop: semua proses berjalan tanpa delay()
void loop() {
  unsigned long now = millis();
  readSerialCommand();
  // Membaca IMU setiap 10 ms
  if (now - lastIMUTime >= IMU_INTERVAL) {
    float dt = (now - lastIMUTime) / 1000.0;
    lastIMUTime = now;
    readIMU(dt);
    updateYaw();
  }
  updatePIR();
  updateServos();
  printStatus();
}