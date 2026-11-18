# Keputusan Desain - Telemetry & Debugging

Dokumentasi keputusan desain dan alasan untuk sistem telemetry dan debugging di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk sistem telemetry dan debugging di LineFollowerFin, yang bertanggung jawab untuk output data robot untuk monitoring dan debugging.

**File:** `docs/design-decisions/telemetry.md`

## 2. Tujuan Sistem Telemetry

### 2.1 Output Data Robot
**Alasan:** Operator perlu memantau data robot untuk debugging dan analysis performa. Telemetry memberikan data real-time tentang status robot.

**Implementasi:**
- Serial output: debugging lokal
- Bluetooth telemetry: monitoring jarak jauh
- CSV format: mudah di-parse dan kompatibel dengan spreadsheet

### 2.2 Debugging
**Alasan:** Debugging diperlukan untuk mengidentifikasi masalah robot. Serial output dan Bluetooth telemetry memfasilitasi debugging.

**Implementasi:**
- Serial print: debugging lokal
- Debug flag: enable/disable debug output
- Timestamp: tracking waktu event

### 2.3 Monitoring
**Alasan:** Monitoring diperlukan untuk memastikan robot berfungsi dengan baik. Bluetooth telemetry memungkinkan monitoring jarak jauh.

**Implementasi:**
- Interval telemetry: interval output data
- Auto telemetry: enable/disable telemetry otomatis
- Connection handling: manage koneksi Bluetooth

## 3. Desain Decision Key

### 3.1 CSV Format
**Alasan:** Format CSV mudah di-parse dan kompatibel dengan spreadsheet. Mendukung telemetry real-time.

**Implementasi:**
```csv
Time(ms),Kp,Ki,Kd,SpeedLeft,SpeedRight,Error,Heading
123456,1.2,1.0,0.8,100,95,2.5,0
```

### 3.2 Interval Telemetry
**Alasan:** Interval telemetry mengontrol frekuensi output data. Interval yang terlalu cepat menambah beban CPU, interval yang terlalu lambat mengurangi keakuratan monitoring.

**Implementasi:**
- BT_TELEMETRY_MS = 1000 (interval telemetry Bluetooth)
- SERIAL_TELEMETRY_MS = 100 (interval telemetry serial)
- Interval disesuaikan berdasarkan kebutuhan bandwidth

### 3.3 Auto Telemetry
**Alasan:** Auto telemetry memungkinkan output data yang otomatis tanpa intervensi operator. Memfasilitasi monitoring kontinu.

**Implementasi:**
- setAutoTelemetry(true, BT_TELEMETRY_MS) untuk mengaktifkan
- Matikan saat debugging atau konektivitas buruk
- Logging telemetry ke file (jika SD card tersedia)

### 3.4 Connection Handling
**Alasan:** Koneksi Bluetooth bisa terputus. Handling koneksi yang tepat memastikan robot tetap responsif.

**Implementasi:**
- Timeout koneksi: BT_MAX_CONNECTION_AGE_MS = 30000
- Reconnect: coba reconnect setelah timeout
- Fallback: serial command saat Bluetooth tidak tersedia

### 3.5 Filtering Data
**Alasan:** Filter data mengurangi noise dan memberikan representasi yang lebih akurat tentang status robot.

**Implementasi:**
- Moving average filter untuk speed
- Derivative filter untuk error
- Filter heading untuk noise gyro

## 4. Desain Trade-offs

### 4.1 Interval Telemetry vs Non-telemetry
**Trade-off:** Interval telemetry memberikan data real-time tapi menambah beban CPU. Non-telemetry lebih sederhana tapi tidak memberikan data real-time. Memilih interval telemetry untuk monitoring.

### 4.2 CSV Format vs Format Lain
**Trade-off:** CSV format mudah di-parse dan kompatibel dengan spreadsheet. Format lain (JSON, XML) lebih fleksibel tapi lebih kompleks. Memilih CSV untuk simplicity.

### 4.3 Auto Telemetry vs Manual
**Trade-off:** Auto telemetry memberikan monitoring kontinu tapi menambah beban CPU. Manual telemetry lebih fleksibel tapi memerlukan intervensi operator. Memilih auto telemetry untuk convenience.

## 5. Validasi Desain

### 5.1 Interval Telemetry
**Validasi:** Interval telemetry diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- BT_TELEMETRY_MS = 1000 (cukup cepat untuk monitoring)
- SERIAL_TELEMETRY_MS = 100 (cukup cepat untuk debugging)

### 5.2 Filtering Data
**Validasi:** Filtering data diuji dengan noise sensor.

**Implementasi:**
- Moving average filter untuk speed
- Derivative filter untuk error
- Filter heading untuk noise gyro

### 5.3 Connection Handling
**Validasi:** Connection handling diuji dengan konektivitas yang buruk.

**Implementasi:**
- Timeout koneksi: 30 detik
- Reconnect setelah timeout
- Fallback ke serial command

## 6. Risiko & Mitigasi

### 6.1 Bandwidth Limit
**Risiko:** Interval telemetry yang terlalu cepat bisa menyebabkan overflow buffer. **Mitigasi:** Sesuaikan interval berdasarkan bandwidth.

### 6.2 Data Corruption
**Risiko:** Data telemetry bisa corrupted. **Mitigasi:** Checksum, CRC, validasi data.

### 6.3 Connection Dropout
**Risiko:** Koneksi Bluetooth terputus. **Mitigasi:** Reconnect otomatis, fallback ke serial.

## 7. Referensi

1. Arduino Telemetry - https://www.arduino.cc/en/Tutorial/Telemetry
2. CSV Format - https://www.arduino.cc/en/Tutorial/CSV
3. Bluetooth Communication - https://www.arduino.cc/en/Tutorial/Bluetooth
4. Serial Communication - https://www.arduino.cc/en/Tutorial/Serial
5. Data Filtering - https://www.arduino.cc/en/Tutorial/DataFiltering

## 8. Catatan Perkembangan

### 8.1 Iterasi Telemetry
- v1.0: Serial output dasar
- v1.1: Tambah Bluetooth telemetry
- v1.2: Interval telemetry, auto telemetry
- v1.3: Filtering data, connection handling
- v1.4: CSV format, timestamp

### 8.2 Validasi Telemetry
- Uji dengan noise sensor
- Uji dengan konektivitas yang buruk
- Uji dengan interval telemetry yang berbeda
- Uji dengan filtering data

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/bluetooth_ctrl.md` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/ui.md` untuk antarmuka