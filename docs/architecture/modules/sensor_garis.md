# Modul SensorGaris - Dokumentasi Teknis

## 1. Ikhtisar

SensorGaris adalah modul sensor IR yang membaca posisi robot relatif terhadap jalur hitam. Ini merupakan komponen input kritis yang menyediakan data mentah untuk sistem kontrol tracer.

**File:** `LineFollowerFin/sensor_garis.h/.cpp` (1.8KB / 8.4KB)
**Dependencies:** `config.h`, `fwd.h`, `<EEPROM.h>`, `<string.h>`

## 2. Desain & Arsitektur

### 2.1 Desain Sensor
- **Sensor:** Sharp GP2Y0A02YK IR distance sensor
- **Port:** Sensor kiri ID 2, sensor kanan ID 1 (dari uKit wrapper)
- **Range:** 0-400cm (output analog via ADC)
- **Sampling:** Throttled untuk menghindari pembacaan berlebihan

### 2.2 Filter Noise
Implementasi filter moving average eksponensial:
```cpp
float filtKiri = -1;  // <0 = belum diinisialisasi
float filtKanan = -1;
int bacaHalus(int id) {
  float &filt = (id == 1) ? filtKiri : filtKanan;
  if (filt < 0) return filt = bacaHalus(id);
  return filt = ALPHA * nilaiBaru + (1 - ALPHA) * filt;
}
```

**Konstanta tune:** `ALPHA_SENSOR = 0.65` (default)

### 2.3 Kalibrasi
Sensor memerlukan kalibrasi di awal setiap run untuk menentukan ambang kiri/kanan.

**Protokol kalibrasi:**
1. Robot berputar 360° (manual atau otomatis)
2. IMU melacak rotasi (CALIB_USE_IMU = true)
3. Catat min/max pembacaan per sensor
4. Hitung ambang = (min + max) / 2
5. Simpan ke EEPROM (jika berubah) melalui eepromPutHemat()

**Tantangan utama:** `eepromPutHemat` template harus didefinisi di header karena multiple definition errors saat include di banyak .cpp files.

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `resetFilter()` | `void` | Adaptasi ulang ke garis baru (reset filter) |
| `bacaKiri()` | `int` | Baca sensor kiri (ID 2), sudah filter |
| `bacaKanan()` | `int` | Baca sensor kanan (ID 1), sudah filter |
| `kalibrasi()` | `void(Penggerak &gerak, bool manual)` | Spin 360° (manual/oto) hitung ambang |
| `simpan()` | `void` | Tulis config (Kp/Ki/Kd/speed/ambang) ke EEPROM |
| `muat()` | `void` | Baca config dari EEPROM (validasi magic) |
| `cetak()` | `void` | Serial print pembacaan sensor saat boot |

### 3.2 Method Adaptive Threshold (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `ambangDinamis()` | `int ambangDinamis(int id) const` | Ambang dinamis berdasarkan speed |
| `setAdaptiveThreshold()` | `void setAdaptiveThreshold(bool on)` | Aktifkan/deaktifkan adaptive threshold |
| `isAdaptiveThresholdEnabled()` | `bool isAdaptiveThresholdEnabled() const` | Status flag |
| `updateAmbangDinamis()` | `void updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar)` | Update ambang tiap PID interval |
| `resetAdaptive()` | `void resetAdaptive()` | Reset ke ambang dasar |
| `simpanAdaptive()` | `void simpanAdaptive()` | Simpan ambang dasar ke EEPROM |
| `muatAdaptive()` | `void muatAdaptive()` | Muat ambang dasar dari EEPROM |

### 3.3 Method Private

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `bacaHalus()` | `int bacaHalus(int id)` | Baca mentah + filter eksp |

## 4. Implementasi Detil

### 4.1 Logika Pembacaan
```cpp
int SensorGaris::bacaKiri() {
  return bacaHalus(2);  // ID untuk sensor kiri
}

int SensorGaris::bacaKanan() {
  return bacaHalus(1);  // ID untuk sensor kanan
}
```

### 4.2 Kalibrasi Mode Manual
```cpp
void SensorGaris::kalibrasi(Penggerak &gerak, bool manual) {
  if (manual) {
    // Mode manual: pengguna memutar robot dengan tangan
    gerak.putarKalibrasi();  // speed servo rendah
  } else {
    // Mode otomatis: gunakan IMU untuk spin 360°
    gerak.putarCariGaris(0);  // gunakan errorTerakhir() sebagai arah
  }
  // Tunggu hingga rotasi selesai (berdasarkan gyro Z atau timeout)
  // Catat min/max pembacaan
  int minKiri = 1023, maxKiri = 0;
  int minKanan = 1023, maxKanan = 0;
  // ... loop pembacaan ...
  AMBANG_KIRI = (minKiri + maxKiri) / 2;
  AMBANG_KANAN = (minKanan + maxKanan) / 2;
  simpanAdaptive();  // tulis ke EEPROM
}
```

### 4.3 Adaptive Threshold Update
```cpp
void SensorGaris::updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar) {
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

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Mengirimkan `bacaKiri()`/`bacaKanan()` ke PID controller
- Berbagi state kalibrasi (ambang disimpan di Penggerak?)
- Digunakan dalam recovery: `putarCariGaris()` butuh sensor reading untuk error

### 5.2 Integrasi dengan Pengaman
- Bekerja sama untuk deteksi macet (`macet(int bacaKiri, int bacaKanan)`)
- Bekerja sama untuk deteksi miring (tidak langsung)
- Bekerja sama untuk deteksi halangan (tidak langsung)

### 5.3 Integrasi dengan Antarmuka
- Used oleh `Antarmuka::parseCommand()` untuk perintah 'calibrate'
- Membaca penyimpanan EEPROM untuk tuning
- Menampilkan status sensor di serial output

### 5.4 Integrasi dengan Mapper
- Tidak langsung, tapi digunakan oleh loop() untuk racing prediction
- Prediction dilakukan berdasarkan reading sensor di persimpangan

## 6. Desain Decisions & Alasan

### 6.1 Filter Eksponensial
**Alasan:** Filter moving average lebih stabil dari filter rata-rata sederhana, redam noise tinggi frekuensi tanpa lag signifikan.

### 6.2 Kalibrasi Otomatis
**Alasan:** Kalibrasi manual memakan waktu & subjektif. IMU memungkinkan kalibrasi 360° yang repeatable dengan timeout.

### 6.3 Adaptive Threshold
**Alasan:** Sensor IR semakin noise di kecepatan tinggi. Meningkatkan ambang mengurangi false trigger di kecepatan tinggi.

### 6.4 eepromPutHemat Template di Header
**Alasan:** eepromPutHemat digunakan oleh banyak modul (sensor_garis, penggerak, dll). Didefinisi di header menghindari multiple definition errors.

### 6.5 Filter Berbasis IMU
**Alasan:** Gyro Z memberikan feedback rotasi yang akurat untuk kalibrasi otomatis, mengurangi false positive dari gerakan tidak terdeteksi.

## 7. Error Handling & Edge Cases

### 7.1 Error Pembacaan
- **Out of range:** Clamp ke batas ADC (0-1023)
- **Filter uninitialized:** Filter < 0 dianggap tidak valid, baca mentah
- **Noise spike:** Filter mengurangi noise spike tinggi frekuensi

### 7.2 Error Kalibrasi
- **Rotasi tidak lengkap:** Fallback ke timer (CALIB_FALLBACK_MS)
- **Min/max tidak valid:** Batalkan kalibrasi, tampilkan error
- **EEPROM write error:** Tulis ulang di startup berikutnya (simpan local)

### 7.3 Edge Cases
- **Both sensors on line:** Ambang dinamis mungkin terlalu tinggi
- **Both sensors off line:** Aktifkan recovery timer
- **Noise di sensor:** Filter eksponensial mengurangi false trigger

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `ALPHA_SENSOR` | 0.65 | 0.4-0.8 | Filter smoothness (lebih tinggi = lebih halus) |
| `ADAPT_SPEED_FACTOR` | 0.25 | 0-1 | Pengaruh speed terhadap ambang |
| `ADAPT_THRESH_MIN` | 3 | 0-10 | Ambang minimum di kecepatan rendah |
| `ADAPT_THRESH_MAX` | 18 | 10-30 | Ambang maximum di kecepatan tinggi |
| `CALIB_TARGET_DERAJAT` | 360.0 | 300-360 | Rotasi target untuk kalibrasi |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- Filter: 2 float (8 bytes)
- Adaptive: 4 int (16 bytes) jika diaktifkan
- Total: ~24 bytes per instance

### 9.2 Konsumsi Waktu
- Pembacaan per loop: ~100-200 microseconds
- Filter per pembacaan: 10-20 microseconds
- Kalibrasi per loop: 5-10 milliseconds (saat spin)

### 9.3 EEPROM Write Cycle
- eepromPutHemat() hanya tulis jika berubah
- Kalibrasi: 1 write per kalibrasi (aman untuk 100k cycles)
- Tuning: 1 write per perubahan parameter

## 10. Referensi & Sumber

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Filter Moving Average - https://en.wikipedia.org/wiki/Moving_average
3. Sharp GP2Y0A02YK - https://www.sparkfun.com/products/10039
4. EEPROM Wear Leveling - https://www.arduino.cc/en/Tutorial/EEPROM

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar, pembacaan filter
- v1.1: Tambahkan kalibrasi otomatis berbasis IMU
- v1.2: Tambahkan adaptive threshold
- v1.3: Perbaikan bug filter (resetFilter)
- v1.4: Implementasi eepromPutHemat template

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/sensor_garis.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/sensor.md` untuk keputusan desain
