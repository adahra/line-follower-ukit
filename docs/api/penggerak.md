# API Dokumentasi - Penggerak

Dokumentasi API lengkap untuk modul Penggerak, controller PID utama yang mengubah error pelacakan garis menjadi command motor. Ini menerapkan kontrol PID anti windup, deadband servo, dan fitur kontrol motor canggih.

## 1. Ikhtisar

Penggerak (PID controller) adalah modul kontrol utama yang mengubah error pelacakan garis menjadi command motor. Ini menerapkan kontrol PID anti windup, deadband servo, dan fitur kontrol motor canggih.

**File:** `LineFollowerFin/penggerak.h/.cpp` (2.9KB / 11.9KB)
**Dependencies:** `fwd.h`, `config.h`, `fuzzy_pid.h` (jika diaktifkan), `heading_tracker.h` (jika diaktifkan)

## 2. Method Publik

### 2.1 Method Inti

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

### 2.2 Method Fuzzy PID (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setFuzzyEnabled()` | `void(bool on)` | Aktifkan/deaktifkan fuzzy PID |
| `isFuzzyEnabled()` | `bool isFuzzyEnabled() const` | Status fuzzy PID |

### 2.3 Method Gain Scheduling (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setGainScheduling()` | `void(bool on)` | Aktifkan/deaktifkan gain scheduling |
| `isGainSchedulingEnabled()` | `bool isGainSchedulingEnabled() const` | Status scheduling |
| `getGainScheduleKp()` | `float getGainScheduleKp() const` | Kp saat ini (sesuai schedule) |
| `getGainScheduleKi()` | `float getGainScheduleKi() const` | Ki saat ini (sesuai schedule) |
| `getGainScheduleKd()` | `float getGainScheduleKd() const` | Kd saat ini (sesuai schedule) |

### 2.4 Method Heading Correction (jika diaktifkan)

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

### 2.5 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `terapkanDeadband()` | `int terapkanDeadband(int speed)` | Konversi speed kecil ke 25 atau 0 |
| `batasiSlew()` | `int batasiSlew(int target, int prev)` | Batasi perubahan speed per loop |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class SpeedMode {
  MAJU = 1,    // Kecepatan positif (maju)
  MUNDUR = -1, // Kecepatan negatif (mundur)
  BERHENTI = 0 // Kecepatan nol
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `Kp` | float | 1.2 | Responsivitas error |
| `Ki` | float | 1.0 | Stabilitas integral |
| `Kd` | float | 0.8 | Damping derivatif |
| `BATAS_INTEGRAL` | int | 50 | Anti windup |
| `SLEW_MAKS_PER_LOOP` | int | 12 | Perubahan speed per loop |
| `SERVO_MIN_EFEKTIF` | int | 25 | Minimum speed efektif |
| `SERVO_MAKS_DIPAKAI` | int | 150 | Derate maximum servo |

## 4. Method Khusus

### 4.1 Gain Scheduling
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

### 4.2 Heading Correction
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

### 4.3 Loop PID
```cpp
bool Penggerak::hitung(int bacaKiri, int bacaKanan, bool paksa) {
  unsigned long sekarang = millis();
  
  // Hitung error (untuk PID & heading)
  float errorBaru = (bacaKiri - bacaKanan) / 2.0;
  
  if (paksa || sekarang - pidTerakhir >= PID_INTERVAL_MS) {
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

## 8. Referensi & Sumber

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Filter Moving Average - https://en.wikipedia.org/wiki/Moving_average
3. Sharp GP2Y0A02YK - https://www.sparkfun.com/products/10039
4. EEPROM wear leveling - https://www.arduino.cc/en/Tutorial/EEPROM

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/penggerak.md` untuk keputusan desain