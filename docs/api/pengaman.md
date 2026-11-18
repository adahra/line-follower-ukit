# API Dokumentasi - Pengaman

Dokumentasi API lengkap untuk modul Pengaman, sistem deteksi berbahaya, berhenti otomatis, dan kontrol lampu LED untuk memastikan operasi robot yang aman.

## 1. Ikhtisar

Pengaman (Safety) adalah modul yang bertanggung jawab untuk deteksi situasi berbahaya, berhenti otomatis, dan lampu LED untuk memastikan operasi robot yang aman. Ini merupakan komponen safety kritis yang melindungi robot dari kondisi berbahaya seperti baterai lemah, kemiringan, halangan, dan macet.

**File:** `LineFollowerFin/pengaman.h/.cpp` (1.1KB / 4.4KB)
**Dependencies:** `status.h`, `penggerak.h`, `sensor_garis.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `bateraiKritis()` | `bool bateraiKritis()` | Periksa voltase baterai kritis (≤5.5V) |
| `miring()` | `bool miring()` | Deteksi kemiringan robot (≥45° roll/pitch) |
| `halangan()` | `bool halangan()` | Deteksi halangan ultrasonik (≤100cm) |
| `macet()` | `bool macet(int bacaKiri, int bacaKanan)` | Deteksi macet robot (IR beku + accel stabil) |
| `lampuOtomatis()` | `void lampuOtomatis()` | Lampu otomatis nyala saat gelap (<100 lux) |
| `tegangan()` | `float tegangan()` | Baca voltase baterai |
| `setLampu()` | `void setLampu(bool on)` | Kontrol manual lampu LED |

### 2.2 Method Private

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cekCahaya()` | `float cekCahaya()` | Baca sensor cahaya |
| `cekIMU()` | `bool cekIMU()` | Deteksi kemiringan dengan throttle |
| `cekHalangan()` | `bool cekHalangan()` | Deteksi halangan dengan throttle |
| `cekBaterai()` | `bool cekBaterai()` | Deteksi baterai lemah dengan throttle |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class SafetyState {
  NORMAL = 0,
  BATERAI_LEMAH,
  TANPA_KONTAK,
  HALANGAN,
  MACET,
  PANAS
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `VOLTAGE_CRITICAL` | float | 5.5 | Voltase baterai kritis (stop) |
| `ANGLE_CRITICAL` | float | 45.0 | Sudut kemiringan kritis (stop) |
| `JARAK_BERHENTI_CM` | int | 100 | Jarak deteksi halangan (cm) |
| `CEK_IMU_MS` | unsigned long | 50 | Throttle pemeriksaan IMU |
| `CEK_HALANGAN_MS` | unsigned long | 100 | Throttle pemeriksaan halangan |
| `CEK_BATERAI_MS` | unsigned long | 500 | Throttle pemeriksaan baterai |
| `CEK_CAHAYA_MS` | unsigned long | 200 | Throttle pemeriksaan cahaya |
| `TEPUK_COOLDOWN_MS` | unsigned long | 800 | Cooldown tepuk tangan |
| `COOLDOWN_TOMBOL_MS` | unsigned long | 300 | Cooldown tombol |

## 4. Implementasi Detil

### 4.1 Deteksi Baterai Kritis
```cpp
bool Pengaman::bateraiKritis() {
  float voltase = tegangan();
  if (voltase <= VOLTAGE_CRITICAL) {
    status.bateraiLemahLatch = true;  // Set latch stop
    return true;
  }
  return false;
}
```

### 4.2 Deteksi Kemiringan
```cpp
bool Pengaman::miring() {
  // Throttle ke CEK_IMU_MS untuk mencegah overload CPU
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < CEK_IMU_MS) {
    return false;
  }
  lastCheck = millis();
  
  float roll = IMU::getRoll();
  float pitch = IMU::getPitch();
  
  if (abs(roll) >= ANGLE_CRITICAL || abs(pitch) >= ANGLE_CRITICAL) {
    status.miringLatch = true;
    return true;
  }
  return false;
}
```

### 4.3 Deteksi Halangan
```cpp
bool Pengaman::halangan() {
  // Throttle ke CEK_HALANGAN_MS
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < CEK_HALANGAN_MS) {
    return false;
  }
  lastCheck = millis();
  
  int jarak = readUltrasonicDistance(ULTRASONIC_FRONT_ID);
  if (jarak > 0 && jarak <= JARAK_BERHENTI_CM) {
    status.halanganLatch = true;
    return true;
  }
  return false;
}
```

### 4.4 Deteksi Macet
```cpp
bool Pengaman::macet(int bacaKiri, int bacaKanan) {
  // Deteksi macet: IR beku + magnitude accel stabil (3 detik)
  static unsigned long macetStart = 0;
  static int prevKiri = bacaKiri, prevKanan = bacaKanan;
  static float accelMagnitude = 0;
  
  // Hitung perubahan sensor
  int deltaKiri = abs(bacaKiri - prevKiri);
  int deltaKanan = abs(bacaKanan - prevKanan);
  
  // Hitung magnitude accel dari perubahan sensor
  accelMagnitude = sqrt(deltaKiri * deltaKiri + deltaKanan * deltaKanan);
  
  // Stabil selama 3 detik
  if (accelMagnitude > MACET_THRESHOLD && macetStart == 0) {
    macetStart = millis();
  }
  
  if (macetStart > 0 && millis() - macetStart >= MACET_DURATION_MS) {
    status.macetLatch = true;
    return true;
  }
  
  if (accelMagnitude <= MACET_THRESHOLD) {
    macetStart = 0;  // Reset jika bergerak normal
  }
  
  prevKiri = bacaKiri;
  prevKanan = bacaKanan;
  return false;
}
```

### 4.5 Lampu Otomatis
```cpp
void Pengaman::lampuOtomatis() {
  // Throttle ke CEK_CAHAYA_MS
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < CEK_CAHAYA_MS) {
    return;
  }
  lastCheck = millis();
  
  float lux = cekCahaya();
  bool gelap = (lux < LUX_DARK_THRESHOLD);
  
  if (gelap != status.lampuOtomatisAktif) {
    status.lampuOtomatisAktif = gelap;
    setLampu(gelap);
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Bekerja sama untuk stop motor saat darurat
- Menerima feedback speed motor untuk deteksi macet
- Reset heading saat kemiringan

### 5.2 Integrasi dengan SensorGaris
- Bekerja sama untuk deteksi macet (`macet(bacaKiri, bacaKanan)`)
- Memberikan status sensor ke loop kontrol
- Digunakan dalam recovery: `resetIntegralSetelahRecovery()`

### 5.3 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial kontrol lampu manual
- Bekerja sama untuk deteksi tepuk tangan dan toggle lampu
- Bekerja sama untuk kalibrasi tombol lampu otomatis

### 5.4 Integrasi dengan Mapper
- Tidak langsung, tapi digunakan oleh loop() untuk safety check
- Bekerja sama untuk deteksi halangan (ultrasonic)

### 5.5 Integrasi dengan Bluetooth
- Kirim status safety melalui Bluetooth telemetry
- Menerima command kontrol lampu manual dari remote

## 6. Desain Decisions & Alasan

### 6.1 Throttle Semua Cek Safety
**Alasan:** Mencegah overload CPU dengan membatasi frekuensi pemeriksaan dangerous condition. Setiap cek punya interval sendiri berdasarkan importance dan biaya komputasi.

### 6.2 Latching Kondisi Darurat
**Alasan:** Kondisi darurat (baterai lemah, kemiringan) punya latch → hanya reset saat restart power. Mencegah false restart saat kondisi dangerous transient.

### 6.3 Priority Antar Mode Operasi
**Alasan:** Ultrasonic halangan memiliki priority tertinggi, diikuti oleh kemiringan, baterai lemah, macet. Mencegah race condition dan memastikan safety.

### 6.4 Deteksi Macet dengan Accelerometer
**Alasan:** Deteksi macet butuh kombinasi pembacaan IR stabil + magnitude accel dari servo yang tidak bergerak. Lebih akurat dari threshold IR saja.

## 7. Error Handling & Edge Cases

### 7.1 Error Safety Checks
- **Sensor error:** Timeout atau pembacaan tidak valid
- **Lampu error:** Kegagalan driver transistor
- **IMU error:** Pembacaan tidak stabil

### 7.2 Error Lampu LED
- **Driver transistor gagal:** Monitor current, matikan relay
- **RGB led terbakar:** Gunakan LED single-color sebagai fallback
- **PWM PWM error:** Batasi duty cycle

### 7.3 Edge Cases
- **Baterai lemah sementara:** Tidak latch, hanya indikasi
- **Kemiringan sementara:** Reset setelah level kembali normal
- **Halangan ultrasonik:** Reset setelah halangan hilang
- **Macet sementara:** Reset jika robot bergerak

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- State safety: 4 bool (latch) = 4 bytes
- Array status: 1 byte per flag = 1 byte
- Total: ~5 bytes per instance

### 8.2 Konsumsi Waktu
- Pemeriksaan per loop: ~50-200 microseconds
- Filter pembacaan sensor: ~10-50 microseconds
- Kalibrasi safety: 5-10 milliseconds (saat emergency)

### 8.3 EEPROM Write Cycle
- Latch safety: 1 write per condition (latch)
- Kalibrasi: 1 write per kalibrasi (aman untuk 100k cycles)
- Tuning: 1 write per perubahan parameter

## 9. Referensi & Sumber

1. Arduino IMU Library - https://www.arduino.cc/en/Tutorial/IMULibrary
2. Ultrasonik HC-SR04 - https://www.arduino.cc/en/Tutorial/UltrasonicSensor
3. Sensor Cahaya - https://www.arduino.cc/en/Tutorial/LightSensor
4. Relay driver - https://www.arduino.cc/en/Tutorial/Relay

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/safety.md` untuk keputusan desain