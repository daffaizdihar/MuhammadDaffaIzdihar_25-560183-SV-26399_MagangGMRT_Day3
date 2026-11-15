# Day 3 - Following-Axis Servo Motor and Motion Detection

**Magang Programmer Gadjah Mada Robotics Team (GMRT) 2026**

## 1. Profile

- **Nama:** Muhammad Daffa Izdihar
- **NIM:** 25/560183/SV/26399

## 2. Description

Project ini merupakan simulasi sistem kontrol 5 motor servo menggunakan ESP32 berdasarkan input dari sensor IMU dan PIR.

Sistem menggunakan MPU6050 sebagai alternatif BNO055 yang tersedia di simulator Wokwi. Sensor ini digunakan untuk membaca data accelerometer dan gyroscope, sedangkan PIR digunakan untuk mendeteksi gerakan eksternal.

Setiap servo memiliki fungsi yang berbeda:
- Servo 1 dan 2 merespons perubahan Roll.
- Servo 3 dan 4 merespons perubahan Pitch.
- Servo 5 merespons perubahan Yaw.
- Semua servo bergerak secara bersamaan ketika PIR mendeteksi gerakan.

### Hardware dan Pin Configuration

| Komponen | Pin ESP32 | Fungsi |
|---|---|---|
| MPU6050 SDA | GPIO21 | I2C Data |
| MPU6050 SCL | GPIO22 | I2C Clock |
| PIR OUT | GPIO27 | Digital Input |
| Servo 1 | GPIO18 | Roll |
| Servo 2 | GPIO19 | Roll |
| Servo 3 | GPIO23 | Pitch |
| Servo 4 | GPIO25 | Pitch |
| Servo 5 | GPIO26 | Yaw |

### Communication Protocol

- **I2C:** Digunakan untuk komunikasi antara ESP32 dan MPU6050 melalui GPIO21 (SDA) dan GPIO22 (SCL).
- **Digital Input:** GPIO27 membaca sinyal HIGH atau LOW dari sensor PIR.
- **PWM:** Digunakan untuk mengatur posisi kelima servo dengan frekuensi 50 Hz dan periode 20 ms.

Konfigurasi PWM servo:

| Posisi Servo | Pulse Width |
|---|---|
| -90° | 1000 us |
| 0° | 1500 us |
| +90° | 2000 us |

Rentang pergerakan servo dibatasi dari -90° sampai +90°.

**Implementation Note:**

Pergerakan Yaw dan PIR menggunakan state machine berbasis `millis()`. Dengan pendekatan non-blocking ini, sistem tetap dapat membaca sensor dan menjalankan proses lainnya tanpa menggunakan `delay()`.

## 3. Feature Table

| Fitur | Input (Sensor) | Output (Aktuator) | Perilaku |
|---|---|---|---|
| Roll | MPU6050 (Roll dari accelerometer, sumbu X) | Servo 1 (GPIO18) dan Servo 2 (GPIO19) | Bergerak berlawanan arah terhadap sudut Roll (-roll), dengan range -90° sampai +90°. |
| Pitch | MPU6050 (Pitch dari accelerometer, sumbu Y) | Servo 3 (GPIO23) dan Servo 4 (GPIO25) | Bergerak searah dengan sudut Pitch (+pitch), dengan range -90° sampai +90°. |
| Yaw | MPU6050 (Gyroscope Z yang diintegrasikan menjadi sudut Yaw, sumbu Z) | Servo 5 (GPIO26) | Mengikuti rotasi Yaw. Setelah rotasi berhenti, servo menahan posisi selama 1 detik, kemudian kembali ke 0° (1500 us). |
| Gerakan Eksternal | PIR Sensor (GPIO27) | Semua Servo (Servo 1-5) | Ketika mendeteksi gerakan (LOW ke HIGH), semua servo bergerak serentak ke +45° selama 600 ms, kemudian kembali ke 0° selama 500 ms. |

## 4. Repository Structure

```text
MuhammadDaffaIzdihar_25-560183-SV-26399_MagangGMRT_Day3/
│
├── evidence/
│   ├── TestCase1.jpg
│   ├── TestCase2.jpg
│   └── TestCase3.jpg
│
├── Link_Jawaban.txt
├── README.md
├── Schematic.jpg
└── main.ino
```

Keterangan file:

- `main.ino`: Program utama ESP32 menggunakan Arduino C++.
- `Schematic.jpg`: Screenshot rangkaian simulasi Wokwi.
- `evidence/TestCase1.jpg`: Bukti Serial Monitor untuk pengujian Roll dan Pitch.
- `evidence/TestCase2.jpg`: Bukti simulasi Yaw saat sedang berotasi.
- `evidence/TestCase3.jpg`: Bukti Yaw setelah berhenti dan Servo 5 kembali ke posisi awal.
- `Link_Jawaban.txt`: Kumpulan link simulasi Wokwi, video demonstrasi, dan repository GitHub.

## 5. How to Run

### Simulation Link

- **Wokwi Project:** https://wokwi.com/projects/480772293299576833
- **Demo Video:** https://drive.google.com/file/d/1MDSodxQPE2qH8CnwAsusXV1bFgT8qLSs/view?usp=sharing
- **GitHub Repository:** https://github.com/daffaizdihar/MuhammadDaffaIzdihar_25-560183-SV-26399_MagangGMRT_Day3

### Required Libraries

Tambahkan library berikut melalui Library Manager Wokwi:

1. ESP32Servo
2. Adafruit MPU6050
3. Adafruit Unified Sensor

Library `Wire.h` dan `math.h` sudah tersedia sebagai bagian dari environment Arduino.

### Starting the Simulation

1. Buka project melalui link Wokwi di atas.
2. Klik tombol **Start Simulation**.
3. Buka Serial Monitor.
4. Atur baud rate ke **115200**.
5. Pastikan pesan `Day 3 System Ready!` muncul di Serial Monitor.

### Testing Commands

Pengujian dapat dilakukan melalui Serial Monitor menggunakan perintah berikut.

| Command | Fungsi |
|---|---|
| `test` | Mengaktifkan simulasi Roll +30° dan Pitch -20°. |
| `yaw 30` | Mensimulasikan rotasi Yaw sebesar +30°/s. |
| `yaw 0` | Mensimulasikan kondisi Yaw berhenti. |
| `sensor` | Menonaktifkan test mode dan kembali membaca MPU6050. |

### Test Case 1: Roll and Pitch

Ketik perintah:

```text
test
```

Sistem akan menggunakan nilai simulasi:
- Roll = +30°
- Pitch = -20°

Hasil yang diharapkan:

| Servo | Sudut | PWM |
|---|---:|---:|
| Servo 1 | -30° | 1333 us |
| Servo 2 | -30° | 1333 us |
| Servo 3 | -20° | 1389 us |
| Servo 4 | -20° | 1389 us |

Output dapat dilihat melalui Serial Monitor.

### Test Case 2: Yaw Stop and Return

1. Ketik `yaw 30` untuk mensimulasikan rotasi.
2. Tunggu beberapa saat sampai Servo 5 bergerak.
3. Ketik `yaw 0` untuk mensimulasikan rotasi berhenti.
4. Sistem mengonfirmasi kondisi berhenti selama 100 ms.
5. Servo 5 menahan posisi selama 1 detik.
6. Servo 5 kemudian kembali ke 0° (1500 us).

### MPU6050 Sensor Test

Untuk menguji pembacaan sensor secara langsung:

1. Ketik `sensor` pada Serial Monitor.
2. Buka komponen MPU6050 di Wokwi.
3. Atur slider Acceleration X dan Y untuk menguji perubahan Roll dan Pitch.
4. Atur slider Rotation Z untuk menguji perubahan Yaw.
5. Amati perubahan nilai sensor dan pergerakan servo.

### PIR Motion Detection Test

1. Klik komponen PIR pada rangkaian Wokwi.
2. Pilih **Simulate Motion**.
3. Ketika gerakan terdeteksi, semua servo bergerak ke +45°.
4. Setelah 600 ms, semua servo kembali ke 0° selama 500 ms.

## 6. Implementation Notes

Beberapa hal yang perlu diperhatikan dalam implementasi:

- MPU6050 tidak menyediakan nilai Yaw absolut. Nilai Yaw dihitung melalui integrasi kecepatan sudut gyroscope Z, sehingga dapat mengalami drift.
- Roll dan Pitch dihitung menggunakan data accelerometer.
- PIR memiliki prioritas tertinggi dalam pengaturan output servo. Ketika gerakan terdeteksi, semua servo mengikuti proses gerakan PIR.
- State machine berbasis `millis()` digunakan agar proses Yaw dan PIR tidak memblokir pembacaan sensor maupun proses lainnya.
- Semua servo menggunakan posisi awal 0° dengan pulse width 1500 us.
