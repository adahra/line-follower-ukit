# Keputusan Desain - Sensor & Kalibrasi

Dokumentasi keputusan desain dan alasan untuk sensor IR dan sistem kalibrasi di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk sensor IR dan sistem kalibrasi di LineFollowerFin, komponen yang membaca posisi robot relatif terhadap jalur hitam.

**File:** `docs/design-decisions/sensor.md`

## 2. Pemilihan Sensor

### 2.1 Sharp GP2Y0A02YK IR Distance Sensor
**Alasan:** Sharp GP2Y0A02YK memberikan akurasi dan keandalan yang baik untuk deteksi jarak IR. Rentang 0-400cm mencakup kebutuhan robot line-following (biasanya < 30cm dari jalur).

**Spesifikasi:**
- Output: Analog (0-5V)
- Rentang: 0-400cm
- Resolusi: ~1cm
- Response time: ~20ms
- Current: ~15mA

### 2.2 Sensor Kiri ID 2, Sensor Kanan ID 1
**Alasan:** uKit wrapper menyediakan mapping pin yang intuitif. Sensor kiri ID 2, sensor kanan ID 1 memudahkan implementasi sistem kontrol.

**Implementasi:**
- Sensor kiri: ID 2 (port kiri uKit)
- Sensor kanan: ID 1 (port kanan uKit)
- Error = (bacaKiri - bacaKanan) / 2. CAN

## 3. Desain Sensor Filter

### 3.1 Filter Eksponensial
**Alasan:** Filter moving average lebih stabil dari filter rata-rata sederhana, redam noise tinggi frekuensi tanpa lag signifikan. Eksponensial filter memberikan trade-off yang baik antara smoothing dan responsivitas.

**Implementasi:**
```cpp
float filtKiri = -1;  // <0 = belum diinisialisasi
float filtKanan = -1;
int bacaHalus(int id) {
  float &filt = (id == 1) ? filtKiri : filtKanan;
  if (filt < 0) return filt = bacaHalus(id);
  return filt = ALPHA * nilaiBaru + (1 - ALPHA) * filt;
}
```

### 3.2 Filter Parameters
**Alasan:** ALPHA sensor memberikan trade-off antara smoothing dan lag. ALPHA tinggi memberikan smoothing yang lebih baik tapi lag yang lebih besar.

**Implementasi:**
- ALPHA_SENSOR = 0.65 (default)
- Range: 0.4-0.8
- Lebih tinggi = lebih halus, lebih besar = lebih lambat

## 4. Sistem Kalibrasi

### 4.1 Kalibrasi Otomatis vs Manual
**Trade-off:** Kalibrasi manual memakan waktu & subjektif. Kalibrasi otomatis memungkinkan kalibrasi 360° yang repeatable dengan timeout. Memilih otomatis untuk convenience.

**Implementasi:**
- Otomatis: Gunakan IMU untuk spin 360° berdasarkan gyro Z
- Manual: Pengguna memutar robot dengan tangan (mode manual)
- Fallback: Timeout timer jika IMU tidak tersedia

### 4.2 Adaptive Threshold
**Alasan:** Sensor IR semakin noise di kecepatan tinggi. Threshold dinamis menyesuaikan noise increase.

**Implementasi:**
```cpp
void updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar) {
  if (!isAdaptiveThresholdEnabled()) return;
  
  // Tingkatkan ambang seiring kecepatan (noise meningkat di kecepatan tinggi)
  float faktor = ADAPT_SPEED_FACTOR;  // 0.25 default
  int tambahan = (int)(faktor * kecepatanDasar);
  
  ambangKiriEfektif = AMBANG_KIRI + tambahan;
  ambangKananEfektif = AMBANG_KANAN + tambahan;
  
  // Clamp ke batas aman
  ambangKiriEfektif = constrain(ambangKiriEfektif, ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
  ambangKananEfektif = constrain(ambangKananEfektif, ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
}
```

## 5. Desain Decision Key

### 5.1 Filter Eksponensial
**Alasan:** Filter moving average lebih stabil dari filter rata-rata sederhana, redam noise tinggi frekuensi tanpa lag signifikan.

**Implementasi:**
- ALPHA_SENSOR = 0.65 (default)
- Filter diterapkan pada setiap pembacaan sensor
- Reset filter saat kalibrasi

### 5.2 Kalibrasi Otomatis
**Alasan:** Kalibrasi manual memakan waktu & subjektif. IMU memungkinkan kalibrasi 360° yang repeatable dengan timeout.

**Implementasi:**
- Gunakan gyro Z untuk deteksi rotasi
- Kalibrasi otomatis hanya jika AUTO_CALIBRATE_ON_BOOT true
- Fallback ke manual jika IMU tidak tersedia

### 5.3 Adaptive Threshold
**Alasan:** Sensor IR semakin noise di kecepatan tinggi. Meningkatkan ambang mengurangi false trigger di kecepatan tinggi.

**Implementasi:**
- ADAPT_SPEED_FACTOR = 0.25 (bobot speed terhadap ambang)
- ADAPT_THRESH_MIN = 3, ADAPT_THRESH_MAX = 18 (batas aman)
- Update tiap interval PID

### 5.4 eepromPutHemat Template di Header
**Alasan:** eepromPutHemat digunakan oleh banyak modul (sensor_garis, penggerak, dll). Didefinisi di header menghindari multiple definition errors.

**Implementasi:**
- template di sensor_garis.h
- Deklarasi forward sebelum setup()
- Semua modul menggunakan template yang sama

### 5.5 Filter Berbasis IMU
**Alasan:** Gyro Z memberikan feedback rotasi yang akurat untuk kalibrasi otomatis, mengurangi false positive dari gerakan tidak terdeteksi.

**Implementasi:**
- IMU::init() di setup()
- IMU::getGyroZ() saat kalibrasi
- Timeout fallback jika IMU tidak responsif

## 6. Desain Trade-offs

### 6.1 Filter Eksponensial vs Rata-rata Sederhana
**Trade-off:** Eksponensial filter memberikan smoothing yang lebih baik dengan lag yang moderat. Rata-rata sederhana lebih presisi tapi lebih sensitif terhadap noise. Memilih eksponensial untuk robustness.

### 6.2 Kalibrasi Otomatis vs Manual
**Trade-off:** Otomatis memberikan consistency tapi memerlukan IMU. Manual lebih fleksibel tapi memakan waktu. Memilih otomatis sebagai default, manual sebagai fallback.

### 6.3 Adaptive Threshold vs Tuning Manual
**Trade-off:** Adaptive threshold mengurangi kebutuhan tuning manual tapi menambah kompleksitas komputasi. Memilih adaptive threshold karena mengurangi waktu setup.

## 7. Validasi Desain

### 7.1 Parameter Filter
**Validasi:** Parameter filter diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- ALPHA_SENSOR = 0.65 (default)
- Uji dengan noise sensor
- Verifikasi lag filter

### 7.2 Kalibrasi Otomatis
**Validasi:** Kalibrasi otomatis diuji dengan IMU.

**Implementasi:**
- Kalibrasi otomatis dengan gyro Z
- Fallback ke manual jika IMU tidak tersedia
- Validasi hasil kalibrasi

### 7.3 Adaptive Threshold
**Validasi:** Adaptive threshold diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- ADAPT_SPEED_FACTOR = 0.25
- ADAPT_THRESH_MIN = 3, ADAPT_THRESH_MAX = 18
- Uji dengan noise sensor

## 8. Risiko & Mitigasi

### 8.1 Noise Sensor
**Risiko:** Noise sensor menyebabkan false positive. **Mitigasi:** Filter eksponensial, adaptive threshold, kalibrasi otomatis.

### 8.2 Error Kalibrasi
**Risiko:** Kalibrasi tidak lengkap menyebabkan ambang tidak valid. **Mitigasi:** Timeout fallback, validasi hasil.

### 8.3 Filter Lag
**Risiko:** Filter eksponensial menyebabkan lag yang bisa mempengaruhi kontrol. **Mitigasi:** Pilih ALPHA yang tepat, gunakan adaptive threshold.

## 9. Referensi

1. Arduino Sensor Library - https://www.arduino.cc/en/Tutorial/Sensor
2. Moving Average Filter - https://en.wikipedia.org/wiki/Moving_average
3. Sharp GP2Y0A02YK - https://www.sparkfun.com/products/10039
4. IMU - https://www.arduino.cc/en/Tutorial/IMU
5. Adaptive Threshold - https://www.arduino.cc/en/Tutorial/AdaptiveThreshold

## 10. Catatan Perkembangan

### 10.1 Iterasi Sensor
- v1.0: Sensor IR biasa, kalibrasi manual
- v1.1: Filter eksponensial, adaptive threshold
- v1.2: Kalibrasi otomatis berbasis IMU
- v1.3: eepromPutHemat template di header
- v1.4: Reset filter, kalibrasi manual fallback

### 10.2 Validasi Sensor
- Uji dengan noise sensor
- Uji kalibrasi otomatis vs manual
- Uji parameter filter
- Uji adaptive threshold

## 11. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/sensor_garis.md` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/pid.md` untuk kontrol PID