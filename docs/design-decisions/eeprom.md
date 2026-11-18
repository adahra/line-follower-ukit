# Keputusan Desain - EEPROM

Dokumentasi keputusan desain dan alasan untuk sistem penyimpanan EEPROM di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk sistem penyimpanan EEPROM di LineFollowerFin, yang bertanggung jawab untuk persistensi data tuning, kalibrasi sensor, dan waypoint antar power cycle.

**File:** `docs/design-decisions/eeprom.md`

## 2. Tujuan Sistem EEPROM

### 2.1 Persistensi Data
**Alasan:** Data tuning, kalibrasi sensor, dan waypoint perlu persistensi antar power cycle. EEPROM menyediakan penyimpanan non-volatile yang sederhana.

**Implementasi:**
- PID gains (Kp, Ki, Kd)
- Speed max
- Sensor thresholds (left/right)
- Waypoint memory (30 waypoints)
- Heading gain and bias
- Adaptive threshold

### 2.2 EEPROM Guard
**Alasan:** uKit Initialization() library menghapus EEPROM 0-20 jika byte 3 != 77. Guard mencegah penghapusan data yang tersimpan.

**Implementasi:**
- EEPROM.update(3, 77) sebelum Initialization()
- Semua data disimpan di EEPROM 24+

### 2.3 Wear Leveling
**Alasan:** EEPROM memiliki jumlah write cycle yang terbatas (~100k cycles). Wear leveling meminimalkan write cycle untuk memperpanjang umur EEPROM.

**Implementasi:**
- eepromPutHemat template: hanya tulis jika berubah
- Batasi write frequency
- Gunakan checksum untuk validasi

## 3. Desain Decision Key

### 3.1 EEPROM Layout
**Alasan:** Layout EEPROM yang terstruktur memfasilitasi akses data yang mudah dan mencegah corruption.

**Implementasi:**
```cpp
// Alokasi (byte address):
// 3 | '77' | Guard uKit: Initialization() hapus 0-20 bila byte 3 != 77
// 0 | '123' | Magic/check validitas EEPROM
// 4-15 | 12 bytes | Kp, Ki, Kd | PID gains
// 16-17 | 2 bytes | kecepatanDasarMaks | Speed max
// 18-19 | 2 bytes | AMBANG_KIRI | Ambang kiri
// 20-21 | 2 bytes | AMBANG_KANAN | Ambang kanan
// 24-266 | 242 bytes | Waypoint | Peta waypoint (30 x 8)
// 280 | 4 bytes | HEADING_GAIN | Heading correction gain
// 284 | 4 bytes | HEADING_TARGET | Target heading saved
// 288-289 | 2 bytes | AMBANG_KIRI_BASE/KANAN | Dasar untuk adaptive threshold
// 292 | 4 bytes | HEADING_BIAS | Bias gyro sudah dikalibrasi
// 296 | 1 byte | '88' | Magic data heading
// 297 | 1 byte | '66' | Magic data adaptive threshold
```

### 3.2 eepromPutHemat Template
**Alasan:** eepromPutHemat template digunakan oleh banyak modul. Didefinisi di header menghindari multiple definition errors.

**Implementasi:**
- Template di sensor_garis.h
- Semua modul menggunakan template yang sama
- Batasi write cycle dengan chekc perubahan

### 3.3 Magic Numbers
**Alasan:** Magic numbers memfasilitasi validasi data EEPROM. Ini mencegah corruption dan memastikan data valid.

**Implementasi:**
- Byte 3 = 77 (guard)
- Byte 0 = 123 (magic)
- Byte 296 = 88 (magic heading)
- Byte 297 = 66 (magic adaptive threshold)

## 4. Desain Trade-offs

### 4.1 EEPROM vs Flash
**Trade-off:** EEPROM write cycle terbatas (~100k cycles). Flash lebih tahan tapi lebih kompleks. Memilih EEPROM untuk simplicity.

### 4.2 Persistency vs Complexity
**Trade-off:** Persistency memfasilitasi pemulihan data tapi menambah kompleksitas. Tanpa persistency lebih sederhana tapi kehilangan data antar power cycle. Memilih persistency untuk user convenience.

### 4.3 Wear Leveling vs Complexity
**Trade-off:** Wear leveling memanjangkan umur EEPROM tapi menambah kompleksitas. Tanpa wear leveling lebih sederhana tapi mengurangi umur EEPROM. Memilih wear leveling untuk longevity.

## 5. Validasi Desain

### 5.1 Layout EEPROM
**Validasi:** Layout EEPROM diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- Validasi magic numbers
- Validasi offset EEPROM
- Validasi ukuran data

### 5.2 Wear Leveling
**Validasi:** Wear leveling diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- eepromPutHemat: hanya tulis jika berubah
- Batasi write frequency
- Gunakan checksum untuk validasi

### 5.3 Magic Numbers
**Validasi:** Magic numbers diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- Byte 3 = 77
- Byte 0 = 123
- Byte 296 = 88
- Byte 297 = 66

## 6. Risiko & Mitigasi

### 6.1 EEPROM Wear
**Risiko:** EEPROM write cycle terbatas. **Mitigasi:** Wear leveling, batasi write frequency.

### 6.2 Data Corruption
**Risiko:** Data EEPROM bisa corrupted. **Mitigasi:** Magic numbers, checksum, validasi.

### 6.3 Read Error
**Risiko:** Pembacaan EEPROM bisa error. **Mitigasi:** Retry, fallback, validasi.

## 7. Referensi

1. EEPROM Arduino - https://www.arduino.cc/en/Tutorial/EEPROM
2. Wear Leveling - https://www.arduino.cc/en/Tutorial/WearLeveling
3. Flash Memory - https://www.arduino.cc/en/Tutorial/FlashMemory
4. Checksum - https://www.arduino.cc/en/Tutorial/Checksum
5. Magic Number - https://www.arduino.cc/en/Tutorial/MagicNumber

## 8. Catatan Perkembangan

### 8.1 Iterasi EEPROM
- v1.0: EEPROM dasar (PID gains)
- v1.1: Tambah sensor thresholds, speed max
- v1.2: Tambah waypoint memory, heading data
- v1.3: Tambah adaptive threshold, wear leveling
- v1.4: Tambah magic numbers, eepromPutHemat template

### 8.2 Validasi EEPROM
- Uji dengan robot pada berbagai kondisi
- Uji dengan EEPROM yang corrupted
- Uji dengan wear leveling
- Uji dengan magic numbers

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/sensor.md` untuk sensor