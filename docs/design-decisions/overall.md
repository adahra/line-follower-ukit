# Keputusan Desain - Overall

Dokumentasi keputusan desain dan alasan untuk arsitektur sistem secara keseluruhan, komponen inti, dan desain robot line-following LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain utama untuk LineFollowerFin, robot pelacak garis otonom untuk kompetisi uKit Explore 2. Keputusan ini didasarkan pada persyaratan kompetisi, keterbatasan hardware, dan praktik terbaik dalam robotika.

**File:** `docs/design-decisions/overall.md`

## 2. Tujuan Arsitektur

### 2.1 Modular Berorientasi Objek
**Alasan:** Arsitektur modular memungkinkan pengembangan, testing, dan pemeliharaan yang terpisah untuk setiap komponen inti. Ini memfasilitasi kode reuse dan mengurangi coupling antar komponen.

**Implementasi:**
- Setiap komponen (sensor_garis, penggerak, pengaman, antarmuka) adalah class terpisah
- Komunikasi antar komponen melalui interface yang didefinisikan dengan jelas
- Dependencies diinjectkan melalui constructor

### 2.2 Pemisahan Kontrol Loop Utama
**Alasan:** Memisahkan loop kontrol utama (loop() di LineFollowerFin.ino) dari komponen lainnya memungkinkan kontrol yang lebih baik dan debugging. Ini juga memfasilitasi testing komponen secara terpisah.

**Implementasi:**
- loop() menangani protocol, UI, dan scheduling komponen
- Komponen inti menangani tugas spesifik mereka (sensor, kontrol PID, safety)
- Fasilitas opsional terintegrasi melalui conditional compilation

### 2.3 Conditional Compilation untuk Fitur Opsional
**Alasan:** Fitur kompetisi seperti Bluetooth, RC, Fuzzy PID, Heading Correction, dan Path Planner adalah opsional. Ini memungkinkan penyesuaian robot berdasarkan kebutuhan kompetisi dan keterbatasan sumber daya.

**Implementasi:**
- Semua fitur opsional dibungkus dalam `#if ENABLE_*` 
- Pengecekan kompiler memastikan binary yang kompak
- Default mostly false untuk menghemat RAM

## 3. Pemilihan Komponen Inti

### 3.1 Sensor IR Sharp GP2Y0A02YK
**Alasan:** Sensor IR Sharp GP2Y0A02YK memberikan akurasi dan keandalan yang baik untuk deteksi garis hitam di atas latar belakang putih. Rentang 0-400cm mencakup kebutuhan robot line-following.

**Implementasi:**
- Sensor kiri ID 2, sensor kanan ID 1 (dari uKit wrapper)
- Filter eksponensial untuk redam noise
- Adaptive threshold dinamis (jika diaktifkan)

### 3.2 Arduino Mega2560 + uKit Explore 2
**Alasan:** Arduino Mega2560 memberikan processing power yang cukup dan I/O pins. uKit Explore 2 library menyediakan API yang abstrak untuk kontrol motor dan sensor.

**Implementasi:**
- Timer interrupt untuk loop PID yang presisi
- PWM control untuk servo motor
- ADC untuk pembacaan sensor analog

### 3.3 Servo Motors untuk Aktuator
**Alasan:** Servo motors memberikan kontrol speed dan posisi yang presisi. 180° range cukup untuk robot line-following.

**Implementasi:**
- Servo kiri: servos 4+2
- Servo kanan: servos 1+3
- Derate speed output 0-150 (dari library servo Arduino)

## 4. Desain Decision Key

### 4.1 Kontrol PID dengan Anti-Windup
**Alasan:** Kontrol PID standar rentan terhadap integral windup saat actuator saturasi. Anti-windup mencegah integral melambung.

**Implementasi:**
- I term dijepit ±BATAS_INTEGRAL (50)
- Reset integral setelah recovery exit
- Loop PID interval tetap (10ms) untuk dt konsisten

### 4.2 Non-blocking UI
**Alasan:** Delay blocking akan mencegah robot melacak garis saat UI merespon. Non-blocking UI memungkinkan robot tetap melacak garis.

**Implementasi:**
- Gunakan cooldown millis() bukan delay()
- Tombol dan tepuk tangan dengan debounce dan cooldown
- Parsing command serial dengan timeout

### 4.3 Adaptive Threshold
**Alasan:** Sensor IR semakin noise di kecepatan tinggi. Threshold dinamis menyesuaikan noise increase.

**Implementasi:**
- Formula: ambang = AMBANG_BASE + ADAPT_SPEED_FACTOR * speed
- Clamp ke ADAPT_THRESH_MIN (3) dan ADAPT_THRESH_MAX (18)
- Update tiap interval PID

### 4.4 Priority Antar Mode Operasi
**Alasan:** Mode operasi (Normal, Learning, Racing, RC) memiliki priority yang berbeda. Ultrasonic halangan memiliki priority tertinggi.

**Implementasi:**
```
priority:
halangan ultrasonik → kembali
modeOperasi = MODE_RC → RC mengambil alih (return)
modeOperasi = MODE_RACING → prediksi validasi
substate = SUBSTATE_INTER_CREEP → 150ms creep
substate = SUBSTATE_TURNING → 250ms + timeout BELOK_TIMEOUT_MS
else → tracer PID normal
```

### 4.5 Non-blocking Turn Logic
**Alasan:** Belok 90° tidak boleh block loop tracer. Substate machine memungkinkan belok non-blocking.

**Implementasi:**
- SUBSTATE_INTER_CREEP: creep melewati persimpangan (150ms)
- SUBSTATE_TURNING: belok 90° non-blocking (250ms + timeout)
- Logic diperiksa tiap loop

### 4.6 EEPROM Guard
**Alasan:** uKit Initialization() menghapus EEPROM 0-20 jika byte 3 != 77. Guard mencegah penghapusan tuning yang tersimpan.

**Implementasi:**
- EEPROM.update(3, 77) sebelum Initialization()
- Semua konfigurasi disimpan di EEPROM 24+
- Validasi magic number di startup

### 4.7 No malloc
**Alasan:** malloc bisa menyebabkan heap fragmentation dan stack overflow di microcontroller. Semua struktur statis.

**Implementasi:**
- Array statis untuk waypoint (30 x 8 bytes)
- Buffer statis untuk telemetry
- Alokasi statis untuk queue

### 4.8 Throttle Semua Cek Safety
**Alasan:** Mencegah overload CPU dengan membatasi frekuensi pemeriksaan dangerous condition. Setiap cek punya interval sendiri.

**Implementasi:**
- CEK_CAHAYA_MS = 200 (lampu gelap)
- CEK_HALANGAN_MS = 100 (ultrasonic)
- CEK_BATERAI_MS = 500 (voltase)
- CEK_IMU_MS = 50 (miring)
- TEPUK_COOLDOWN_MS = 800 (histeresis tepuk)

## 5. Desain Trade-offs

### 5.1 Adaptive Threshold vs Tuning Manual
**Trade-off:** Adaptive threshold mengurangi kebutuhan tuning manual tapi menambah kompleksitas komputasi. Memilih adaptive threshold karena mengurangi waktu setup.

### 5.2 Non-blocking vs Blocking UI
**Trade-off:** Non-blocking UI memungkinkan pelacakan garis yang terus berlanjut tapi menambah kompleksitas coding. Memilih non-blocking untuk performance yang lebih baik.

### 5.3 Path Planning Manual vs Otonom
**Trade-off:** Path planning manual memungkinkan debugging yang mudah tapi bukan navigasi otonom. Memilih manual karena bukan kebutuhan kompetisi utama.

### 5.4 Fuzzy PID vs PID Biasa
**Trade-off:** Fuzzy PID memberikan penyesuaian yang dinamis tapi menambah kompleksitas komputasi. Memilih fuzzy PID untuk robustness yang lebih baik.

## 6. Validasi Desain

### 6.1 Penjadwalan Speed
**Validasi:** Penjadwalan speed berdasarkan kecepatan (rendah/mid/high) memberikan respons yang optimal di setiap kecepatan.

**Implementasi:**
- Speed rendah (≤60): Kp tinggi (responsif), Kd rendah (tidak butuh damping keras)
- Speed tinggi (>100): Kp rendah (anti overshoot), Kd tinggi (stabil)
- Speed menengah: gain menengah

### 6.2 Heading Correction sebagai Lapisan Atas
**Validasi:** Heading correction berbasis gyro Z komplementer dengan PID berbasis error sensor.

**Implementasi:**
- Heading correction sebagai tambahan PID, bukan replacement
- Gain HEADING_GAIN yang rendah (0.5) untuk correction yang halus
- Reset heading saat start/stop

### 6.3 Adaptive Threshold Validasi
**Validasi:** Adaptive threshold diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- ADAPT_SPEED_FACTOR = 0.25 (bobot speed terhadap ambang)
- ADAPT_THRESH_MIN = 3, ADAPT_THRESH_MAX = 18 (batas aman)

## 7. Risiko & Mitigasi

### 7.1 EEPROM Wear
**Risiko:** EEPROM write cycle terbatas (~100k cycles). **Mitigasi:** Hanya tulis jika berubah (eepromPutHemat), batasi tulis ke tuning dan kalibrasi.

### 7.2 Heap Fragmentation
**Risiko:** malloc bisa menyebabkan fragmentation. **Mitigasi:** Tidak ada malloc, semua statis.

### 7.3 Sensor Noise
**Risiko:** Noise sensor menyebabkan false positive. **Mitigasi:** Filter eksponensial, adaptive threshold, non-blocking UI.

### 7.4 Race Condition
**Risiko:** Multiple thread akses shared resource. **Mitigasi:** Gunakan mutex dan atomic operations, batasi concurrency dengan throttle.

## 8. Referensi

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Filter Moving Average - https://en.wikipedia.org/wiki/Moving_average
3. Sharp GP2Y0A02YK - https://www.sparkfun.com/products/10039
4. EEPROM wear leveling - https://www.arduino.cc/en/Tutorial/EEPROM
5. Fuzzy Logic Control - https://www.arduino.cc/en/Tutorial/FuzzyControl
6. A* Algorithm - https://www.arduino.cc/en/Tutorial/A*
7. Arduino Threading - https://www.arduino.cc/en/Tutorial/Threading

## 9. Catatan Perkembangan

### 9.1 Iterasi Desain
- v1.0: Arsitektur modular dasar, kontrol PID, sensor IR
- v1.1: Adaptive threshold, non-blocking UI, EEPROM persistensi
- v1.2: Fuzzy PID, heading correction, path planning manual
- v1.3: Safety checks, throttle cek, error handling
- v1.4: RC control, Bluetooth telemetry, race prediction

### 9.2 Validasi yang Dilakukan
- Unit test untuk komponen inti (sensor, PID, safety)
- Integration test untuk loop kontrol utama
- Performance test untuk konsumsi RAM dan waktu
- Functional test untuk kompetisi (tracking garis, belok 90°, kalibrasi)

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/architecture/modules/` untuk dokumentasi per modul
- Lihat `docs/design-decisions/` untuk keputusan desain lainnya