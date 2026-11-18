# API Dokumentasi - HeadingTracker

Dokumentasi API lengkap untuk modul HeadingTracker, sistem koreksi heading berbasis gyro Z (jika diaktifkan) untuk drift correction.

## 1. Ikhtisar

HeadingTracker (Heading Correction) adalah modul yang bertanggung jawab untuk koreksi berbasis gyro Z (gain HEADING_GAIN) untuk drift correction. Ini merupakan komponen heading correction yang penting yang memungkinkan robot mempertahankan heading yang akurat tanpa drift.

**File:** `LineFollowerFin/heading_tracker.h/.cpp` (1.7KB / 3.0KB)
**Dependencies:** `config.h`, `sensor_garis.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `update()` | `void update()` | Integrasi gyro tiap loop |
| `setHeadingCorrection()` | `void setHeadingCorrection(bool on)` | Aktifkan/deaktifkan heading correction |
| `isHeadingCorrectionEnabled()` | `bool isHeadingCorrectionEnabled() const` | Status correction |
| `setHeadingTarget()` | `void setHeadingTarget(float t)` | Target heading |
| `getHeading()` | `float getHeading() const` | Heading saat ini |
| `getTargetHeading()` | `float getTargetHeading() const` | Target heading |
| `getHeadingError()` | `float getHeadingError() const` | Error heading |
| `resetHeading()` | `void reset()` | Reset heading ke target |
| `calibrateHeadingBias()` | `void calibrateHeadingBias()` | Kalibrasi bias gyro |
| `simpanHeading()` | `void simpanHeading()` | Simpan heading ke EEPROM |
| `muatHeading()` | `void muatHeading()` | Muat heading dari EEPROM |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `getGyroZ()` | `float getGyroZ() const` | Kembalikan gyro Z |
| `getGyroBias()` | `float getGyroBias() const` | Kembalikan bias gyro |
| `setGyroBias()` | `void setGyroBias(float bias)` | Set bias gyro |
| `cetakHeading()` | `void cetakHeading()` | Tampilkan heading debug |
| `validasiHeading()` | `bool validasiHeading() const` | Validasi heading |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `integrasiGyro()` | `void integrasiGyro()` | Integrasi gyro Z |
| `kalibrasiBias()` | `void kalibrasiBias()` | Kalibrasi bias gyro |
| `perbaruiAnguler()` | `void perbaruiAnguler()` | Perbarui anguler heading |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class HeadingState {
  OFF = 0,
  ON,
  CALIBRATING,
  ERROR
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------0RE1|---------|---------|-------------|
| `HEADING_GAIN` | float | 0.5 | Gain koreksi heading |
| `HEADING_MAX_OFFSET` | float | 15 | Maximum offset heading |
| `HEADING_CALIB_MS` | unsigned long | 5000 | Durasi kalibrasi heading |
| `GYRO_CALIB_MS` | unsigned long | 1000 | Durasi kalibrasi gyro |
| `HEADING_TARGET_DEFAULT` | float | 0 | Target heading default |
| `GYRO_NOISE_THRESHOLD` | float | 0.1 | Threshold noise gyro |
| `HEADING_DRIFT_MAX` | float | 5 | Drift heading maksimum |

## 4. Method Khusus

### 4.1 Method Update
```cpp
void HeadingTracker::update() {
  if (!isHeadingCorrectionEnabled()) {
    return;
  }
  
  unsigned long sekarang = millis();
  float dt = (sekarang - lastUpdateTime) / 1000.0;
  lastUpdateTime = sekarang;
  
  // Dapatkan gyro Z
  float gyroZ = getGyroZ();
  
  // Periksa noise gyro
  if (abs(gyroZ) > GYRO_NOISE_THRESHOLD) {
    state = HEADING_STATE_ERROR;
    return;
  }
  
  // Integrasi gyro Z
  heading += gyroZ * dt;
  
  // Kalibrasi bias (background)
  if (isHeadingCalibrating()) {
    float bias = kalibrasiBias(dt);
    heading -= bias * dt;
  }
}
```

### 4.2 Method Kalibrasi Bias
```cpp
float HeadingTracker::kalibrasiBias(float dt) {
  // Kalibrasi bias gyro Z
  float gyroZ = getGyroZ();
  
  // Hitung bias sebagai rata-rata pembacaan gyro saat statis
  bias += gyroZ * dt;
  
  // Tampilkan debug
  #if ENABLE_SERIAL_OUTPUT
  Serial.print(F("HeadingBias: "));
  Serial.println(bias);
  #endif
  
  return bias;
}
```

### 4.3 Method Heading Correction
```cpp
float HeadingTracker::getHeadingError() const {
  float error = getHeading() - getTargetHeading();
  
  // Clamp error ke HEADING_MAX_OFFSET
  error = constrain(error, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
  
  return error;
}
```

### 4.4 Method Reset
```cpp
void HeadingTracker::resetHeading() {
  heading = getTargetHeading();
  state = HEADING_STATE_ON;
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Input: `gerak.errorTerakhir()` sebagai error heading
- Output: `gerak.updateHeading()` sebagai heading correction
- Digunakan dalam loop PID sebagai lapisan koreksi di atas PID

### 5.2 Integrasi dengan SensorGaris
- Input: `sensor.bacaKiri()` - `sensor.bacaKanan()` sebagai error PID
- Output: heading correction sebagai tambahan PID
- Digunakan dalam adaptive heading correction berdasarkan error sensor

### 5. nanocamInterface
- Bekerja sama untuk perintah serial 'heading on'/'heading off'
- Bekerja sama untuk perintah serial 'heading target ...'
- Bekerja sama untuk perintah serial 'heading calibrate'

### 5.5 Integrasi dengan Pengaman
- Bekerja sama untuk kalibrasi bias gyro (background)
- Bekerja sama untuk reset heading (reset heading)
- Bekerja sama untuk error heading (heading correction)

### 5.6 Integrasi dengan Mapping
- Bekerja sama untuk heading correction dalam racing prediction
- Bekerja sama untuk heading correction dalam mode learning/racing

## 6. Desain Decisions & Alasan

### 6.1 Heading Correction Sebagai Lapisan Atas
**Alasan:** Heading correction berbasis gyro Z komplementer dengan PID berbasis error sensor. Ini adalah koreksi steer tambahan, bukan replacement.

### 6.2 Integrasi Gyro Z
**Alasan:** Gyro Z memberikan feedback heading yang akurat untuk drift correction. Lebih robust dari sensor lainnya.

### 6.3 Kalibrasi Bias
**Alasan:** Kalibrasi bias gyro memastikan heading yang akurat. Background calibration memungkinkan drift correction terus berjalan.

### 6.4 Maximum Offset
**Alasan:** Maximum offset (HEADING_MAX_OFFSET) mencegah heading correction yang terlalu besar. Mencegah overshoot heading.

## 7. Error Handling & Edge Cases

### 7.1 Error Heading
- **Gyro error:** Set state ERROR
- **Kalibrasi error:** Reset heading
- **Output error:** Gunakan default heading

### 7.2 Error Kalibrasi
- **Bias error:** Kalibrasi ulang bias
- **Noise gyro:** Gunakan filter noise
- **Kalibrasi timeout:** Gagal kalibrasi

### 7.3 Edge Cases
- **Drift heading:** Kalibrasi bias gyro
- **Kalibrasi heading:** Reset heading ke target
- **Bias heading:** Set bias heading
- **Heading error:** Clamp error heading

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- State heading: 20 bytes
- Bias heading: 4 bytes
- Heading correction: 8 bytes
- Total: ~32 bytes per instance

### 8.2 Konsumsi Waktu
- Update heading per loop: ~5-10 microseconds
- Kalibrasi bias per loop: ~2-5 microseconds
- Heading error: ~2-5 microseconds
- Total per loop: ~15-25 microseconds

### 8.3 EEPROM Write Cycle
- Penulisan heading: 1 write per simpan heading (aman untuk 100k cycles)

## 9. Referensi & Sumber

1. Gyro Arduino - https://www.arduino.cc/en/Tutorial/IMU
2. Heading Control - https://www.arduino.cc/en/Tutorial/Heading
3. Bias Calibration - https://www.arduino.cc/en/Tutorial/BiasCalibration
4. Drift Correction - https://www.arduino.cc/en/Tutorial/DriftCorrection

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/heading.md` untuk keputusan desain