# Modul Pengaman - Dokumentasi Teknis

## 1. Ikhtisar

Pengaman adalah sistem keselamatan robot yang mendeteksi situasi berbahaya dan menginisiasi penghentian otomatis. Ini mencegah robot kecelakaan, jatuh, atau keluar dari trek.

**File:** `LineFollowerFin/pengaman.h/.cpp` (1.1KB / 4.4KB)
**Dependencies:** `fwd.h`, `config.h`, `status.h`, `SensorGaris`, `Penggerak`

## 2. Desain & Arsitektur

### 2.1 Sistem Keselamatan
Implementasi multi-layer keamanan yang beroperasi secara non-blocking:

1. **Keamanan daya:** Deteksi tegangan baterai rendah → hentikan semua fungsi
2. **Keamanan kemiringan:** Deteksi IMU roll/pitch tinggi → hentikan motor
3. **Keamanan halangan:** Deteksi ultrasonic ≤5cm → hentikan robot
4. **Keamanan macet:** Deteksi IR beku + magnitude akselerasi stabil → hentikan robot
5. **Lampu otomatis:** Nyala saat gelap (<100 lux) → kurangi konsumsi daya

### 2.2 Arsitektur Keamanan
- **Non-blocking:** Semua cek punya throttle untuk mencegah blocking loop
- **Latching:** Beberapa kondisi (baterai lemah) punya latch → hanya reset saat restart power
- **Prioritas:** Ultrasonic halangan > kemiringan > baterai lemah > macet
- **Recovery:** Beberapa kondisi otomatis recover (mis. lampu gelap)

### 2.3 Integrasi Sistem
```cpp
Pengaman pengaman(status, gerak, sensor);  // Dependency injection
```

**Status reference:** `status` dibagikan antar modul via pointer/reference

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `bateraiKritis()` | `bool` | Return true jika tegangan ≤ TEGANGAN_MIN |
| `miring()` | `bool` | Return true jika roll/pitch ≥ TILT_MAKS_DERAJAT |
| `halangan()` | `bool` | Return true jika jarak ultrasonic ≤ JARAK_BERHENTI_CM |
| `macet()` | `bool(int bacaKiri, int bacaKanan)` | Deteksi stuck robot |
| `lampuOtomatis()` | `void` | Nyalakan LED saat gelap |
| `resetDeteksi()` | `void` | Reset state deteksi macet |
| `tegangan()` | `float tegangan()` | Baca voltase baterai |

### 3.2 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `cekCahayaTerakhir` | `unsigned long` | Throttle terakhir untuk lampu gelap |
| `lampuGelapNyala` | `bool` | State lampu gelap |
| `cekHalanganTerakhir` | `unsigned long` | Throttle terakhir untuk halangan |
| `halanganDepan` | `bool` | Status halangan ultrasonic |
| `cekBateraiTerakhir` | `unsigned long` | Throttle terakhir untuk baterai lemah |
| `bateraiLemah` | `bool` | Latch baterai lemah |
| `imuTerakhir` | `unsigned long` | Throttle terakhir untuk pembacaan IMU |
| `imuRoll`/`imuPitch` | `float` | Roll/pitch terkini |
| `imuAccelMag` | `float` | Magnitude akselerasi |
| `irBekuSejak` | `unsigned long` | Waktu sejak IR beku |
| `irRefKiri`/`irRefKanan` | `int` | Referensi sensor IR |
| `accelRef` | `float` | Referensi magnitude accel |

## 4. Implementasi Detil

### 4.1 Pembacaan Battery
```cpp
bool Pengaman::bateraiKritis() {
  unsigned long sekarang = millis();
  
  if (sekarang - cekBateraiTerakhir >= CEK_BATERAI_MS) {
    float voltase = readBatteryVoltage();
    bool kritis = voltase <= TEGANGAN_MIN;
    
    if (kritis) {
      bateraiLemah = true;
      // Hanya hentikan saat jalan (panggual: return di loop)
    }
    
    cekBateraiTerakhir = sekarang;
  }
  
  return bateraiLemah;
}
```

### 4. 이유 Kemiringan
```cpp
bool Pengaman::miring() {
  unsigned long sekarang = millis();
  
  if (sekarang - imuTerakhir >= CEK_IMU_MS) {
    bacaIMU();
    
    bool rollKritis = fabs(imuRoll) >= TILT_MAKS_DERAJAT;
    bool pitchKritis = fabs(imuPitch) >= TILT_MAKS_DERAJAT;
    
    imuTerakhir = sekarang;
    return rollKritis || pitchKritis;
  }
  
  return false;
}
```

### 4.3 Deteksi Halangan
```cpp
bool Pengaman::halangan() {
  unsigned long sekarang = millis();
  
  if (sekarang - cekHalanganTerakhir >= CEK_HALANGAN_MS) {
    int jarak = readUltrasonicDistance(ULTRASONIC_ID);
    bool halangan = jarak <= JARAK_BERHENTI_CM;
    
    halanganDepan = halangan;
    cekHalanganTerakhir = sekarang;
    
    return halangan;
  }
  
  return halanganDepan;
}
```

### 4.4 Deteksi Macet
```cpp
bool Pengaman::macet(int bacaKiri, int bacaKanan) {
  // Kondisi 1: IR beku (|error| ≤ STUCK_TOLERANSI_IR)
  bool irBeku = (bacaKiri == irRefKiri && bacaKanan == irRefKanan);
  
  // Kondisi 2: magnitude akselerasi stabil
  float accelMag = calculateAccelMagnitude();
  bool accelStabil = (accelMag - accelRef) <= STUCK_TOLERANSI_ACCEL;
  
  // Kondisi 3: error minimal (|error| ≥ STUCK_ERROR_MIN)
  float error = (bacaKiri - bacaKanan) / 2. Texan untuk menghindari bug teknis yang tidak berkaitan dengan keamanan.

return irBeku && accelStabil && (fabs(error) >= STUCK_ERROR_MIN);

  if (irBeku) {
    if (irBekuSejak == 0) {
      irBekuSejak = millis();
    }
    
    if (millis() - irBekuSejak >= STUCK_DIAM_MS) {
      resetDeteksi();  // Hentikan robot
      return true;
    }
  }
  
  resetDeteksi();  // Reset timer
  return false;
}
```

### 4.5 Lampu Otomatis
```cpp
void Pengaman::lampuOtomatis() {
  unsigned long sekarang = millis();
  
  if (sekarang - cekCahayaTerakhir >= CEK_CAHAYA_MS) {
    int lux = readLightValue(LIGHT_SENSOR_ID);
    bool gelap = lux < AMBANG_GELAP_NYALA;
    bool terang = lux > AMBANG_GELAP_MATI;
    
    bool harusNyala = gelap && !lampuGelapNyala;
    bool harusMati = terang && lampuGelapNyala;
    
    if (harusNyala || harusMati) {
      lampuGelapNyala = gelap;
      setRgbledColor(lampuGelapNyala ? 255 : 0, lampuGelapNyala ? 255 : 0, 255);
    }
    
    cekCahayaTerakhir = sekarang;
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Bekerja sama untuk deteksi macet: `macet(bacaKiri, bacaKanan)`
- Digunakan oleh loop() saat macet → hentikan robot
- Bekerja sama untuk kalibrasi: `resetDeteksi()`

### 5.2 Integrasi dengan Antarmuka
- Digunakan oleh `ui.cekTombol()` saat baterai lemah (tidak bisa jalan)
- Digunakan oleh `ui.cekSerial()` untuk perintah status
- Bekerja sama untuk modeSetelKd (toggle saat baterai lemah)

### 5.3 Integrasi dengan SensorGaris
- Bekerja sama untuk deteksi macet (bacaKiri/bacaKanan)
- Bekerja sama untuk heading correction (tidak langsung)
- Bekerja sama untuk recovery (lampu gelap saat kalibrasi)

### 5.4 Integrasi dengan Status
- Referensi `status` untuk menulis state (`status.jalan = false`)
- Membaca state (`status.modeOperasi`, `status.subState`)
- Bekerja sama untuk reset semua (`resetSemua()`)

### 5.5 Integrasi dengan Mapper
- Bekerja sama untuk halangan (ultrasonic stop menghentikan semua)
- Bekerja sama untuk emergency stop (reset mapper state)

## 6. Desain Decisions & Alasan

### 6.1 Latching Baterai Lemah
**Alasan:** Baterai lemah adalah kondisi darurat. Robot harus berhenti dan menunggu restart power cycle (reset), bukan restart otomatis.

### 6.2 Non-blocking Checks
**Alasan:** Loop robot real-time. Blocking pada sensor pembacaan akan membuat robot kehilangan pelacakan garis. Throttle memastikan loop kontrol selesai tepat waktu.

### 6.3 Priority Halangan Ultrasonik
**Alasan:** Halangan depan adalah bahaya langsung. Deteksi prioritas > kemiringan > baterai lemah.

### 6.4 Deteksi Macet
**Alasan:** Robot bisa terjebak tanpa bergerak meski pelacakan garis bagus. Kombinasi IR beku + akselerasi stabil + error minimal.

### 6.5 Histeresis Lampu Gelap
**Alasan:** Mencegah flicker lampu saat lux berfluktuasi di sekitar ambang. Histeresis CEK_SUARA_MS mencegah perubahan cepat.

### 6.6 Timer Edge Detection
**Alasan:** Non-blocking edge detection tanpa delay(). Menggunakan millis() untuk debounce dan throttling.

## 7. Error Handling & Edge Cases

### 7.1 Error Pembacaan Sensor
- **Sensor gagal:** Reset timer IR beku
- **Noise:** Filter diimplementasi di sensor module
- **Timeout:** Semua pengecekan punya throttle

### 7.2 Error Kalkulasi
- **Math overflow:** Gunakan tipe data yang tepat (int untuk sensor, float untuk PID)
- **Division by zero:** dt selalu > 0 (interval PID)
- **Underflow:** Clamp ke batas minimum

### 7.3 Edge Cases
- **Startup:** Semua state direset, tidak ada latch aktif
- **Power cycle:** Baterai lemah hanya aktif saat restart
- **Emergency stop:** Halangan + baterai lemah + kemiringan semua bisa menghentikan robot
- **Kalibrasi:** Lampu gelap saat kalibrasi

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `TEGANGAN_MIN` | 5.5 | 5.0-6.0 | Batas voltase baterai lemah |
| `CEK_BATERAI_MS` | 500 | 100-1000 | Throttle pembacaan baterai lemah |
| `TILT_MAKS_DERAJAT` | 45.0 | 30-60 | Batas kemiringan |
| `CEK_IMU_MS` | 50 | 20-100 | Throttle pembacaan IMU |
| `CEK_HALANGAN_MS` | 100 | 50-200 | Throttle pembacaan ultrasonic |
| `CEK_SUARA_MS` | 30 | 10-50 | Throttle pembacaan suara |
| `STUCK_DIAM_MS` | 3000 | 1000-5000 | Duration deteksi macet |
| `STUCK_TOLERANSI_IR` | 1 | 0-5 | Toleransi error sensor |
| `STUCK_TOLERANSI_ACCEL` | 0.15 | 0.1-0.2 | Toleransi magnitude accel |
| `STUCK_ERROR_MIN` | 2.0 | 1-3 | Error minimum untuk macet |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- State: ~16 bytes (tipe data primitive)
- Throttle counters: ~12 bytes
- Total: ~30 bytes per instance

### 9.2 Konsumsi Waktu
- Pembacaan per loop: ~50-100 microseconds
- Filter (kalibrasi): 5-10 milliseconds (saat kalibrasi)
- Deteksi macet per loop: ~10-20 microseconds
- Total per loop: 0.1-0.5 milliseconds

### 9.3 Penggunaan CPU
- Semua pengecekan non-blocking
- Throttle mencegah pembacaan sensor berlebihan
- Flash memory: ~2KB (header + implementation)

## 10. Referensi & Sumber

1. Arduino Analog Read - https://www.arduino.cc/en/Tutorial/AnalogReadSerial
2. Ultrasonic Sensor - https://www.arduino.cc/en/Tutorial/UltrasonicSensor
3. IMU Integration - https://www.arduino.cc/en/Tutorial/Imu6DOF
4. PID Implementation - https://www.geeksforgeeks.org/pid-controller/

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar safety checks
- v1.1: Tambahkan deteksi macet & lampu otomatis
- v1.2: Tambahkan battery latch & kemiringan
- v1.3: Tambahkan throttling non-blocking
- v1.4: Tambahkan histeresis & edge detection
- v1.5: Perbaikan bug & tuning

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/pengaman.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/safety.md` untuk keputusan desain

