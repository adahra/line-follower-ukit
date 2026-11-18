# Modul HeadingTracker - Dokumentasi Teknis

## 1. Ikhtisar

HeadingTracker adalah module IMU yaw integration dan heading correction untuk LineFollowerFin. Ini mengintegrasikan gyro Z (derajat/detik) menjadi absolute heading (derajat) dengan kalibrasi bias gyro untuk kompensasi drift. Digunakan oleh Penggerak sebagai lapisan koreksi steering di atas PID.

**File:** `LineFollowerFin/heading_tracker.h/.cpp` (1.7KB / 3.0KB)
**Dependencies:** `Arduino.h`, `config.h` (agar HEADING_CALIB_MS terdefinisi)

## 2. Desain & Arsitektur

### 2.1 Koreksi Heading
- **Input:** gyro Z (derajat/detik) dari IMU
- **Output:** absolute heading (derajat), heading error
- **Kalibrasi:** bias gyro (drift compensation)
- **Gain:** HEADING_GAIN (0.8 default)
- **Max offset:** HEADING_MAX_OFFSET (40 derajat)

### 2.2 Arsitektur
```cpp
class HeadingTracker {
public:
  void begin();
  void update();          // integrasi gyro tiap loop
  void reset();
  void setTarget(float targetHeading);
  void setBias(float b);
  void calibrateBias(unsigned long durationMs = HEADING_CALIB_MS);

  float getHeading() const { return heading; }
  float getTargetHeading() const { return targetHeading; }
  float getHeadingError() const;
  float getGyroZ() const { return gyroZ; }
  float getBias() const { return gyroBias; }
  bool isBiasCalibrated() const { return biasCalibrated; }
  bool isEnabled() const { return enabled; }
  void setEnabled(bool on) { enabled = on; }
  bool isCalibrating() const { return calibrating; }

private:
  float heading = 0.0;
  float targetHeading = 0.0;
  float gyroBias = 0.0;
  float gyroZ = 0.0;
  float lastUpdateTime = 0;
  bool enabled = false;
  bool biasCalibrated = false;
  bool calibrating = false;
  unsigned long calibStart = 0;
  unsigned long calibDuration = HEADING_CALIB_MS;
  int calibSamples = 0;
  float calibSum = 0.0;
};
```

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void` | Inisialisasi heading tracker |
| `update()` | `void` | Integrasi gyro tiap loop |
| `reset()` | `void` | Reset heading ke target |
| `setTarget()` | `void setTarget(float t)` | Set target heading |
| `setBias()` | `void setBias(float b)` | Set bias gyro |
| `calibrateBias()` | `void calibrateBias(unsigned long durationMs = HEADING_CALIB_MS)` | Kalibrasi bias gyro |

### 3.2 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `heading` | `float` | Absolute heading (derajat) |
| `targetHeading` | `float` | Target heading |
| `gyroBias` | `float` | Bias gyro yang sudah dikalibrasi |
| `gyroZ` | `float` | Gyro Z saat ini |
| `lastUpdateTime` | `float` | Waktu update terakhir |
| `enabled` | `bool` | Flag enable/disable |
| `biasCalibrated` | `bool` | Flag bias calibrated |
| `calibrating` | `bool` | Flag sedang kalibrasi |
| `calibStart` | `unsigned long` | Waktu kalibrasi dimulai |
| `calibDuration` | `unsigned long` | Durasi kalibrasi |
| `calibSamples` | `int` | Jumlah sampel kalibrasi |
| `calibSum` | `float` | Total sum sampel kalibrasi |

## 4. Integrasi dengan Modul Lain

### 4.1 Integrasi dengan Penggerak
- Bekerja sama untuk heading correction: `gerak.updateHeading()`
- Bekerja sama untuk heading error: `gerak.getHeadingError()`
- Bekerja sama untuk reset: `gerak.resetHeading()`
- Bekerja sama untuk kalibrasi: `gerak.calibrateHeadingBias()`
- Bekerja sama untuk penyimpanan: `gerak.simpanHeading()` / `gerak.muatHeading()`

### 4.2 Integrasi dengan SensorGaris
- Bekerja sama untuk error: `error = headingError + PID_error`
- Bekerja sama untuk calibration: Tidak langsung

## 5. Desain Decisions & Alasan

### 5.1 Integration-Based Heading
**Alasan:** Heading correction sebagai lapisan tambahan di atas PID. Tidak replacement, tapi koreksi.

### 5.2 Bias Gyro
**Alasan:** Gyro drift seiring waktu. Kalibrasi bias gyro untuk kompensasi drift.

### 5.3 Max Offset
**Alasan:** Batasi heading correction ke HEADING_MAX_OFFSET (40 derajat) mencegah overcorrection.

### 5.4 Calibration Timer
**Alasan:** Kalibrasi bias gyro butuh waktu (HEADING_CALIB_MS = 1500ms). Background calibration.

##  and More
