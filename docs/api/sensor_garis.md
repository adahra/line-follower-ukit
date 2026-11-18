# API Dokumentasi - SensorGaris

Dokumentasi API lengkap untuk modul SensorGaris, komponen inti yang membaca posisi robot relatif terhadap jalur hitam menggunakan sensor IR Sharp GP2Y0A02YK.

## 1. Ikhtisar

SensorGaris adalah modul sensor IR yang membaca posisi robot relatif terhadap jalur hitam. Ini merupakan komponen input kritis yang menyediakan data mentah untuk sistem kontrol tracer.

**File:** `LineFollowerFin/sensor_garis.h/.cpp` (1.8KB / 8.4KB)
**Dependencies:** `config.h`, `fwd.h`, `<EEPROM.h>`, `<string.h>`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `resetFilter()` | `void` | Adaptasi ulang ke garis baru (reset filter) |
| `bacaKiri()` | `int` | Baca sensor kiri (ID 2), sudah filter |
| `bacaKanan()` | `int` | Baca sensor kanan (ID 1), sudah filter |
| `kalibrasi()` | `void(Penggerak &gerak, bool manual)` | Spin 360° (manual/oto) hitung ambang |
| `simpan()` | `void` | Tulis config (Kp/Ki/Kd/speed/ambang) ke EEPROM |
| `muat()` | `void` | Baca config dari EEPROM (validasi magic) |
| `cetak()` | `void` | Serial print pembacaan sensor saat boot |

### 2.2 Method Adaptive Threshold (jika diaktifkan)

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `ambangDinamis()` | `int ambangDinamis(int id) const` | Ambang dinamis berdasarkan speed |
| `setAdaptiveThreshold()` | `void setAdaptiveThreshold(bool on)` | Aktifkan/deaktifkan adaptive threshold |
| `isAdaptiveThresholdEnabled()` | `bool isAdaptiveThresholdEnabled() const` | Status flag |
| `updateAmbangDinamis()` | `void updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar)` | Update ambang tiap PID interval |
| `resetAdaptive()` | `void resetAdaptive()` | Reset ke ambang dasar |
| `simpanAdaptive()` | `void simpanAdaptive()` | Simpan ambang dasar ke EEPROM |
| `muatAdaptive()` | `void muatAdaptive()` | Muat ambang dasar dari EEPROM |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `bacaHalus()` | `int bacaHalus(int id)` | Baca mentah + filter eksp |

## 3. Method Privasi

### 3.1 Method Private

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `bacaHalus()` | `int bacaHalus(int id)` | Baca mentah + filter eksp |

## 4. Tipe Data Bersama

### 4.1 Enum

```cpp
enum class SensorID {
  KIRI = 2,    // ID port sensor IR kiri
  KANAN = 1,   // ID port sensor IR kanan
  CAHAYA = 1   // ID port sensor cahaya (opsional)
};
```

### 4.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `ALPHA_SENSOR` | float | 0.65 | Filter eksponensial smoothing |
| `ADAPT_SPEED_FACTOR` | float | 0.25 | Pengaruh speed terhadap ambang |
| `ADAPT_THRESH_MIN` | int | 3 | Ambang minimum di kecepatan rendah |
| `ADAPT_THRESH_MAX` | int | 18 | Ambang maximum di kecepatan tinggi |

## 5. Implementasi Detil

### 5.1 Method Inti

#### 5.1.1 `bacaKiri()` dan `bacaKanan()`
```cpp
int SensorGaris::bacaKiri() {
  return bacaHalus(2);  // ID untuk sensor kiri
}

int SensorGaris::bacaKanan() {
  return bacaHalus(1);  // ID untuk sensor kanan
}
```

#### 5.1.2 Filter Eksponensial
```cpp
int SensorGaris::bacaHalus(int id) {
  float &filt = (id == 1) ? filtKiri : filtKanan;
  if (filt < 0) return filt = bacaHalus(id); // inisialisasi
  int nilaiBaru = analogRead(id);
  return filt = ALPHA * nilaiBaru + (1 - ALPHA) * filt;
}
```

#### 5.1.3 Kalibrasi
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

### 5.2 Method Adaptive Threshold Update
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

## 6. Integrasi dengan Modul Lain

### 6.1 Integrasi dengan Penggerak
- Mengirimkan `bacaKiri()`/`bacaKanan()` ke PID controller
- Berbagi state kalibrasi (ambang disimpan di Penggerak?)
- Digunakan dalam recovery: `putarCariGaris()` butuh sensor reading untuk error

### 6.2 Integrasi dengan Pengaman
- Bekerja sama untuk deteksi macet (`macet(int bacaKiri, int bacaKanan)`)
- Bekerja sama untuk deteksi miring (tidak langsung)
- Bekerja sama untuk deteksi halangan (tidak langsung)

### 6.3 Integrasi dengan Antarmuka
- Used oleh `Antarmuka::parseCommand()` untuk perintah 'calibrate'
- Membaca penyimpanan EEPROM untuk tuning
- Menampilkan status sensor di serial output

### 6.4 Integrasi dengan Mapper
- Tidak langsung, tapi digunakan oleh loop() untuk racing prediction
- Prediction dilakukan berdasarkan reading sensor di persimpangan

## 7. Desain Decisions & Alasan

### 7.1 Filter Eksponensial
**Alasan:** Filter moving average lebih stabil dari filter rata-rata sederhana, redam noise tinggi frekuensi tanpa lag signifikan.

### 7.2 Kalibrasi Otomatis
**Alasan:** Kalibrasi manual memakan waktu & subjektif. IMU memungkinkan kalibrasi 360° yang repeatable dengan timeout.

### 7.3 Adaptive Threshold
**Alasan:** Sensor IR semakin noise di kecepatan tinggi. Meningkatkan ambang mengurangi false trigger di kecepatan tinggi.

### 7.4 eepromPutHemat Template di Header
**Alasan:** eepromPutHemat digunakan oleh banyak modul (sensor_garis, penggerak, dll). Didefinisi di header menghindari multiple definition errors.

### 7.5 Filter Berbasis IMU
**Alasan:** Gyro Z memberikan feedback rotasi yang akurat untuk kalibrasi otomatis, mengurangi false positive dari gerakan tidak terdeteksi.

## 8. Error Handling & Edge Cases

### 8.1 Error Pembacaan
- **Out of range:** Clamp ke batas ADC (0-1023)
- **Filter uninitialized:** Filter < 0 dianggap tidak valid, baca mentah
- **Noise spike:** Filter mengurangi noise spike tinggi frekuensi

### 8.2 Error Kalibrasi
- **Rotasi tidak lengkap:** Fallback ke timer (CALIB_FALLBACK_MS)
- **Min/max tidak valid:** Batalkan kalibrasi, tampilkan error
- **EEPROM write error:** Tulis ulang di startup berikutnya (simpan local)

### 8.3 Edge Cases
- **Both sensors on line:** Ambang dinamis mungkin terlalu tinggi
- **Both sensors off line:** Aktifkan recovery timer
- **Noise di sensor:** Filter eksponensial mengurangi false trigger

## 9. Referensi & Sumber

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Filter Moving Average - https://en.wikipedia.org/wiki/Moving_average
3. Sharp GP2Y0A02YK - https://www.sparkfun.com/products/10039
4. EEPROM wear leveling - https://www.arduino.cc/en/Tutorial/EEPROM

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/sensor.md` untuk keputusan desain