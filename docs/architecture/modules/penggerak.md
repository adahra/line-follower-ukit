# Modul Penggerak - Dokumentasi Teknis

## 1. Ikhtisar

Penggerak (PID controller) adalah modul kontrol utama yang mengubah error pelacakan garis menjadi command motor. Ini menerapkan kontrol PID anti windup, deadband servo, dan fitur kontrol motor canggih.

**File:** `LineFollowerFin/penggerak.h/.cpp` (2.9KB / 11.9KB)
**Dependencies:** `fwd.h`, `config.h`, `fuzzy_pid.h` (jika diaktifkan), `heading_tracker.h` (jika diaktifkan)

## 2. Desain & Arsitektur

### 2.1 Kontrol Motor
- **Aktuator:** Two 180° servos (kiri: servos 4+2, kanan: servos 1+3)
- **Derate:** Speed output dibatasi 0-150 (dari library servo Arduino)
- **Deadband:** Speed 1-24 dipaksa ke 25 atau 0 (SERVO_MIN_EFEKTIF = 25)
- **Slew:** Maximum change per loop: ±SLEW_MAKS_PER_LOOP (12)

### 2.2 Kontrol PID
**Formasi:** Standard PID dengan anti windup dan integral clamping:
```cpp
float P = error * Kp;
float I = I + error * Ki;           // I term dijepit ±BATAS_INTEGRAL (50)
float D = (error - lastError) * Kd / dt;  // dt konstan = PID_INTERVAL_MS
float PID_value = P + I + D;
```

**Tantangan utama:** Delay tombol membuat D melonjak acak → butuh PID_INTERVAL_MS konstan (10ms) untuk dt konsisten.

### 2.3 Penjadwalan Speed
**Feature:** Dynamic gain scheduling berdasarkan kecepatan (jika diaktifkan)
- Speed rendah (≤60): Kp tinggi (responsif), Kd rendah (tidak butuh damping keras)
- Speed tinggi (>100): Kp rendah (anti overshoot), Kd tinggi (stabil)
- Speed menengah: gain menengah

### 2.4 Heading Correction
**Feature:** Korreksi berbasis gyro Z (jika diaktifkan)
- Integrasi gyro Z → heading absolut
- Kalibrasi bias gyro untuk kompensasi drift
- Digunakan sebagai lapisan koreksi di atas PID

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `reset()` | `void` | Reset PID + speed + telemetri ke nol |
| `resetIntegralSetelahRecovery()` | `void(float errorBaru)` | Buang hutang integral, sinkronkan lastError |
| `hitung()` | `bool(int bacaKiri, int bacaKanan, bool paksa)` | Hitung PID, return true jika interval PID selesai |
| `terapkanTerakhir()` | `void` | Pertahankan speed terakhir antar interval PID |
| `jalankan()` | `void(int speedKiri, int speedKanan)` | Jalankan motor pada speed tertentu (0-150) |
| `jalankanArah()` | `void(int speedKiri, int speedKanan)` | Jalankan motor dengan tanda: positif = maju, negatif = mundur |
| `berhenti()` | `void` | Hentikan kedua motor |
| `saatBerhenti()` | `void` | Berhenti + buang memori slew |
| `maju()` | `void(int speed)` | Gerakkan robot maju |
| `putarKiri()` | `void(int speed)` | Belok kiri |
| `putarKanan()` | `void(int speed)` | Belok kanan |
| `putarKalibrasi()` | `void` | Putar robot untuk kalibrasi sensor |
| `putarCariGaris()` | `void(float lastErr)` | Recovery mode saat hilang garis |
| `errorTerakhir()` | `float errorTerakhir() const` | Kembalikan error PID terakhir |
| `telemetri()` | `void(int bacaKiri, int bacaKanan)` | Kirim CSV telemetry |

### 3.2 Method Fuzzy PID (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setFuzzyEnabled()` | `void(bool on)` | Aktifkan/deaktifkan fuzzy PID |
| `isFuzzyEnabled()` | `bool isFuzzyEnabled() const` | Status fuzzy PID |

### 3.3 Method Gain Scheduling (jika diaktifkan)

| Method | Signature | Keterangan |
|--------x---------|-------------|
| `setGainScheduling()` | `void(bool on)` | Aktifkan/deaktifkan gain scheduling |
| `isGainSchedulingEnabled()` | `bool isGainSchedulingEnabled() const` | Status scheduling |
| `getGainScheduleKp()` | `float getGainScheduleKp() const` | Kp saat ini (sesuai schedule) |
| `getGainScheduleKi()` | `float getGainScheduleKi() const` | Ki saat ini (sesuai schedule) |
| `getGainScheduleKd()` | `float getGainScheduleKd() const` | Kd saat ini (sesuai schedule) |

### 3.4 Method Heading Correction (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setHeadingCorrection()` | `void(bool on)` | Aktifkan/deaktifkan heading correction |
| `isHeadingCorrectionEnabled()` | `bool isHeadingCorrectionEnabled() const` | Status correction |
| `updateHeading()` | `void` | Integrasi gyro tiap loop |
| `setHeadingTarget()` | `void(float t)` | Target heading |
| `getHeading()` | `float getHeading() const` | Heading saat ini |
| `getTargetHeading()` | `float getTargetHeading() const` | Target heading |
| `getHeadingError()` | `float getHeadingError() const` | Error heading |
| `resetHeading()` | `void` | Reset heading ke target |
| `calibrateHeadingBias()` | `void` | Kalibrasi bias gyro |
| `simpanHeading()` | `void` | Simpan heading ke EEPROM |
| `muatHeading()` | `void` | Muat heading dari EEPROM |

### 3.5 Method Private

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `terapkanDeadband()` | `int terapkanDeadband(int speed)` | Konversi speed kecil ke 25 atau 0 |
| `batasiSlew()` | `int batasiSlew(int target, int prev)` | Batasi perubahan speed per loop |

## 4. Implementasi Detil

### 4.1 Loop PID
```cpp
bool Penggerak::hitung(int bacaKiri, int bacaKanan, bool paksa) {
  unsigned long sekarang = millis();
  
  // Hitung error (untuk PID & heading)
  float errorBaru = (bacaKiri - bacaKanan) / 2.0;
  
  if (pacsa || sekarang - pidTerakhir >= PID_INTERVAL_MS) {
    dt = (sekarang - pidTerakhir) / 1000.0;  // detik
    
    // Fuzzy PID (jika diaktifkan)
#if ENABLE_FUZZY_PID
    if (isFuzzyEnabled()) {
      float dKp, dKd, dKi;
      fuzzyPid.compute(errorBaru, errorBaru - lastError, dKp, dKd, dKi);
      Kp += dKp; Ki += dKi; Kd += dKd;
    }
#endif

    // Gain scheduling (jika diaktifkan)
#if ENABLE_GAIN_SCHEDULING
    if (isGainSchedulingEnabled()) {
      updateGainSchedule berdasar kecepatanDasar;
    }
#endif

    // Hitung PID terms
    float P = errorBaru * Kp;
    I = I + errorBaru * Ki;
    I = constrain(I, -BATAS_INTEGRAL, BATAS_INTEGRAL);  // anti windup
    float D = (errorBaru - lastError) * Kd / dt;
    
    PID_value = P + I + D;
    error = errorBaru;
    pidTerakhir = sekarang;
    
    // Hitung speed target dengan heading correction (jika diaktif)
#if ENABLE_HEADING_CORRECTION
    float headingCorrection = getHeadingError() * HEADING_GAIN;
    headingCorrection = constrain(headingCorrection, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
    PID_value += headingCorrection;
#endif
    
    // Terapkan deadband & slew limit
    int targetKiri = PID_value + kecepatanDasar;
    int targetKanan = PID_value - kecepatanDasar;
    
    targetKiri = terapkanDeadband(targetKiri);
    targetKanan = terapkanDeadband(targetKanan);
    
    kecepatanKiri = batasiSlew(targetKiri, prevKiri);
    kecepatanKanan = batasiSlew(targetKanan, prevKanan);
    
    prevKiri = kecepatanKiri;
    prevKanan = kecepatanKanan;
    
    return true;
  }
  return false;
}
```

### 4.2 Penjadwalan Speed
```cpp
void Penggerak::updateGainSchedule(int kecepatanDasar) {
  if (!isGainSchedulingEnabled()) return;
  
  if (kecepatanDasar <= GS_SPEED_LOW_MAX) {
    Kp = GS_KP_LOW; Ki = GS_KI_LOW; Kd = GS_KD_LOW;
  } else if (kecepatanDasar <= GS_SPEED_MID_MAX) {
    Kp = GS_KP_MID; Ki = GS_KI_MID; Kd = GS_KD_MID;
  } else {
    Kp = GS_KP_HIGH; Ki = GS_KI_HIGH; Kd = GS_KD_HIGH;
  }
}
```

### 4.3 Heading Correction
```cpp
void Penggerak::updateHeading() {
#if ENABLE_HEADING_CORRECTION
  if (!isHeadingCorrectionEnabled()) return;
  
  unsigned long sekarang = millis();
  float dt = (sekarang - lastUpdateTime) / 1000.0;
  lastUpdateTime = sekarang;
  
  float gyroZ = IMU::getGyroZ();
  heading += gyroZ * dt;  // Integrasi
  
  // Kalibrasi bias (background)
  if (isHeadingCalibrating()) {
    float bias = calibrateHeadingBias(dt);
    heading -= bias * dt;
  }
#endif
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan SensorGaris
- Input: `bacaKiri()`, `bacaKanan()` → hitung error
- Output: speed kiri/kanan (PID_value ± kecepatanDasar)

### 5.2 Integrasi dengan Pengaman
- Bekerja sama untuk deteksi macet: `aman.macet(bacaKiri, bacaKanan)`
- Digunakan saat recovery: `resetIntegralSetelahRecovery()`
- Bekerja sama untuk heading correction: `resetHeading()`

### 5.3 Integrasi dengan Antarmuka
- Used oleh `ui.parseCommand()` untuk setel Kp/Ki/Kd/speed/ambang
- Bekerja sama untuk modeSetelKd (toggle Kd)
- Bekerja sama untuk tuningSequence (mode B1+B2 combo)

### 5.4 Integrasi dengan Mapper
- Bekerja sama untuk racing prediction: `mapper.checkPrediction()` mengembalikan ActionType
- Bekerja sama untuk mode operasi: `status.modeOperasi = MODE_RACING`

### 5.5 Integrasi dengan LapTimer
- Input: `lapTimer.update(bacaKiri, bacaKanan, kecepatanDasar, error)`
- Bekerja sama untuk perhitungan stat lap

## 6. Desain Decisions & Alasan

### 6.1 Deadband Servo
**Alasan:** Hardware servo punya deadband: command speed kecil hanya berdengung, roda tidak bergerak. 25 adalah minimum efektif.

### 6.2 Anti Windup
**Alasan:** Tanpa anti windup, I term bisa melambung saat actuator saturasi (mis. motor berhenti). Clamping ±50 mencegah integral melambung.

### 6.3 PID Interval Tetap
**Alasan:** Delay antar loop membuat D melonjak acak. Interval tetap (10ms) memastikan dt konstan.

### 6.4 Heading Correction Sebagai Lapisan Atas
**Alasan:** Heading correction berbasis gyro Z komplementer dengan PID berbasis error sensor. Ini adalah koreksi steer tambahan, bukan replacement.

### 6.5 Penjadwalan Speed
**Alasan:** Karakteristik respon PID berbeda saat speed rendah vs tinggi. Kp tinggi saat speed rendah membuat responsif, Kd tinggi saat speed tinggi membuat stabil.

### 6.6 Slew Limit
**Alasan:** Mencegah perubahan speed tiba-tiba yang bisa membuat robot slip atau overshoot.

## 7. Error Handling & Edge Cases

### 7.1 Error PID
- **Integral windup:** Clamp ±BATAS_INTEGRAL
- **Derivative spike:** Pastikan dt > 0 (interval PID)
- **Heading error:** Clamp ke HEADING_MAX_OFFSET
- **Fuzzy PID:** Pastikan gains tidak negatif

### 7.2 Error Motor
- **Deadband:** Konversi speed kecil ke 25 atau 0
- **Slew:** Batasi perubahan speed → mencegah overshoot
- **Speed underflow:** Clamp ke SERVO_MIN_EFEKTIF

### 7.3 Edge Cases
- **Robot stuck:** Pengaman.macet() mengembalikan ke stop
- **Lost line:** Recovery timer → putarCariGaris()
- **Heading drift:** Heading correction basler kesalahan gyro
- **Sensor noise:** Filter eksponensial mengurangi error

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `Kp` | 1.2 | 0.5-3 | Responsivitas error |
| `Ki` | 1.0 | 0-5 | Stabilitas integral |
| `Kd` | 0.8 | 0-2 | Damping derivatif |
| `BATAS_INTEGRAL` | 50 | 30-100 | Anti windup |
| `SLEW_MAKS_PER_LOOP` | 12 | 5-20 | Perubahan speed per loop |
| `SERVO_MIN_EFEKTIF` | 25 | 20-30 | Minimum speed efektif |
| `SERVO_MAKS_DIPAKAI` | 150 | 120-180 | Derate maximum servo |

### 8.1 Penjadwalan Speed
```cpp
#define GS_SPEED_LOW_MAX 60
#define GS_SPEED_MID_MAX 100
#define GS_KP_LOW 1.2
#define GS_KP_MID 1.�י�
#define GS_KP_HIGH 0.85
```

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- PID state: 4 float (error, lastError, I, PID_value) = 16 bytes
- Gain scheduling: 6 float (Kp/Ki/Kd per level) = 24 bytes
- Heading tracker: ~50 bytes (jika diaktifkan)
- Fuzzy PID: ~50 bytes (jika diaktifkan)
- Total: ~50-100 bytes per instance

### 9.2 Konsumsi Waktu
- Loop PID per interval: ~200-400 microseconds
- Heading update per loop: ~50 microseconds
- Fuzzy PID per loop: ~100-200 microseconds (jika diaktifkan)
- Total per loop: 1-2 milliseconds

### 9.3 Output Motor
- Command PWM: 0-255 (library Arduino)
- Derate: 0-150 (output efektif)
- PWM frequency: 50Hz (default Arduino)

## 10. Referensi & Sumber

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Servo Control - https://www.arduino.cc/en/Tutorial/KHI0-07c
3. PID Implementation - https://www.geeksforgeeks.org/pid-controller/
4. Heading Filtering - https://www.roboticsproceedings.org/rss04/p17.pdf

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar PID controller
- v1.1: Tambahkan anti windup & integral clamping
- v1.2: Tambahkan deadband servo & slew limit
- v1.3: Tambahkan heading correction
- v1.4: Tambahkan fuzzy PID & gain scheduling
- v1.5: Penyesuaian tuning & error handling

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/penggerak.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/pid.md` untuk keputusan desain

