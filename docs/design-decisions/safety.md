# Keputusan Desain - Safety

Dokumentasi keputusan desain dan alasan untuk sistem safety, deteksi berbahaya, dan berhenti otomatis di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk sistem safety di LineFollowerFin, yang bertanggung jawab untuk deteksi situasi berbahaya, berhenti otomatis, dan lampu LED untuk memastikan operasi robot yang aman.

**File:** `docs/design-decisions/safety.md`

## 2. Tujuan Sistem Safety

### 2.1 Deteksi Bahaya
**Alasan:** Robot line-following harus mendeteksi kondisi berbahaya dan berhenti otomatis untuk mencegah kerusakan atau kecelakaan. Ini merupakan komponen safety kritis yang melindungi robot dan operator.

**Implementasi:**
- Baterai kritis → kembali
- Kemiringan IMU → kembali
- Halangan ultrasonik → kembali
- Macet → kembali

### 2.2 Lampu LED Otomatis
**Alasan:** Lampu otomatis membantu operator melihat robot di lingkungan yang gelap. Ini memfasilitasi debugging dan monitoring.

**Implementasi:**
- Lampu nyala saat gelap (<100 lux)
- Non-blocking (throttle 200ms)
- Kontrol manual melalui antarmuka

### 2.3 Priority Antar Mode Operasi
**Alasan:** Mode operasi (Normal, Learning, Racing, RC) memiliki priority yang berbeda. Ultrasonic halangan memiliki priority tertinggi.

**Implementasi:**
```
priority:
halangan ultrasonik → kembali
modeOperasi = MODE_RC → RC mengambil alih (return)
modeOperasi = MODE_RACING → prediksi validasi
substate = SUBSTATE_INTER_CREEP → 150ms creep
substate = SUBSTATE_TURNING → 250ms + timeout BELOK_TIMEOUT_MS
else → tracer PID normal
```

## 3. Pemilihan Komponen Safety

### 3.1 Sensor Baterai
**Alasan:** Sensor voltase baterai memberikan feedback tentang kesehatan baterai. Baterai lemah bisa menyebabkan performa yang buruk.

**Implementasi:**
- Baterai kritis saat voltase ≤ 5.5V
- Latch stop untuk mencegah restart otomatis
- Monitoring throttle 500ms

### 3.2 IMU untuk Deteksi Kemiringan
**Alasan:** IMU mendeteksi kemiringan robot (>45° roll/pitch). Kemiringan bisa menyebabkan kerusakan hardware atau kecelakaan.

**Implementasi:**
- Roll/pitch ≥ 45° → kembali
- Throttle CEK_IMU_MS = 50ms
- Tidak ada latch (reset saat level kembali normal)

### 3.3 Ultrasonic untuk Deteksi Halangan
**Alasan:** Ultrasonic mendeteksi halangan depan robot (≤100cm). Halangan depan harus dihindari untuk mencegah tabrakan.

**Implementasi:**
- Jarak ≤ JARAK_BERHENTI_CM (100cm)
- Throttle CEK_HALANGAN_MS = 100ms
- Latch halangan (reset saat halangan hilang)

### 3.4 Sensor IR untuk Deteksi Macet
**Alasan:** Sensor IR mendeteksi macet robot (IR beku + magnitude accel stabil). Macet bisa menyebabkan overheating dan kerusakan.

**Implementasi:**
- IR beku + magnitude accel stabil (3 detik)
- Throttle CEK_MACET_MS = 200ms
- Latch macet (reset saat robot bergerak)

## 4. Desain Decision Key

### 4.1 Throttle Semua Cek Safety
**Alasan:** Mencegah overload CPU dengan membatasi frekuensi pemeriksaan dangerous condition. Setiap cek punya interval sendiri berdasarkan importance dan biaya komputasi.

**Implementasi:**
```cpp
// Throttle pemeriksaan
static unsigned long lastCheck = 0;
if (millis() - lastCheck < CEK_BATERAI_MS) {
  return false;
}
lastCheck = millis();
```

### 4.2 Latching Kondisi Darurat
**Alasan:** Kondisi darurat (baterai lemah, kemiringan) punya latch → hanya reset saat restart power. Mencegah false restart saat kondisi dangerous transient.

**Implementasi:**
- status.bateraiLemahLatch = true
- status.miringLatch = true
- status.halanganLatch = true
- status.macetLatch = true

### 4.3 Priority Antar Mode Operasi
**Alasan:** Ultrasonic halangan memiliki priority tertinggi, diikuti oleh kemiringan, baterai lemah, macet. Mencegah race condition dan memastikan safety.

**Implementasi:**
```cpp
if (status.jalan == true) {
  // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
  if (aman.halangan()) {
    return;
  }
  
  // Robot miring/terangkat: matikan motor selama tidak rata (tak dikunci).
  if (aman.miring()) {
    return;
  }
  
  // Baterai lemah: hentikan semua operasi.
  if (aman.bateraiKritis()) {
    return;
  }
  
  // ... cek safety lainnya ...
}
```

### 4.4 Deteksi Macet dengan Accelerometer
**Alasan:** Deteksi macet butuh kombinasi pembacaan IR stabil + magnitude accel dari servo yang tidak bergerak. Lebih akurat dari threshold IR saja.

**Implementasi:**
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

## 5. Desain Trade-offs

### 5.1 Latching vs Non-latching
**Trade-off:** Latching kondisi darurat mencegah restart otomatis tapi bisa membuat robot tidak bisa digunakan. Non-latching memungkinkan restart otomatis tapi rentan terhadap false restart. Memilih latching untuk safety.

### 5.2 Throttle vs Non-throttle
**Trade-off:** Throttle mencegah overload CPU tapi menambah latency. Non-throttle lebih responsif tapi berisiko overheating. Memilih throttle untuk reliability.

### 5.3 Priority vs Concurrent
**Trade-off:** Priority memastikan safety condition memiliki precedence. Concurrent memungkinkan parallel processing tapi berisiko race condition. Memilih priority untuk safety.

## 6. Validasi Desain

### 6.1 Throttle Safety
**Validasi:** Throttle safety diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- CEK_CAHAYA_MS = 200 (lampu gelap)
- CEK_HALANGAN_MS = 100 (ultrasonic)
- CEK_BATERAI_MS = 500 (voltase)
- CEK_IMU_MS = 50 (miring)
- TEPUK_COOLDOWN_MS = 800 (histeresis tepuk)

### 6.2 Latching Safety
**Validasi:** Latching safety diuji dengan robot pada kondisi berbahaya.

**Implementasi:**
- Baterai lemah: latch hingga restart power
- Kemiringan: reset saat level kembali normal
- Halangan: reset saat halangan hilang
- Macet: reset saat robot bergerak

### 6.3 Priority Safety
**Validasi:** Priority safety diuji dengan robot pada kondisi yang berbeda.

**Implementasi:**
- Ultrasonic > kemiringan > baterai lemah > macet
- Setiap cek memiliki priority yang berbeda

## 7. Risiko & Mitigasi

### 7.1 False Positive
**Risiko:** Safety checks memberikan false positive. **Mitigasi:** Throttle, hysteresis, validasi hasil.

### 7.2 False Negative
**Risiko:** Safety checks memberikan false negative. **Mitigasi:** Reduce threshold, batasi interval, tambah sensor.

### 7.3 Race Condition
**Risiko:** Multiple thread akses shared resource. **Mitigasi:** Gunakan mutex dan atomic operations, batasi concurrency dengan throttle.

## 8. Referensi

1. Arduino Safety Library - https://www.arduino.cc/en/Tutorial/Safety
2. Ultrasonic Sensor - https://www.arduino.cc/en/Tutorial/UltrasonicSensor
3. IMU - https://www.arduino.cc/en/Tutorial/IMU
4. Battery Monitor - https://www.arduino.cc/en/Tutorial/BatteryMonitor
5. Motion Detection - https://www.arduino.cc/en/Tutorial/MotionDetection

## 9. Catatan Perkembangan

### 9.1 Iterasi Safety
- v1.0: Safety checks dasar (baterai, kemiringan)
- v1.1: Tambah halangan ultrasonik dan macet
- v1.2: Throttle safety, latching
- v1.3: Priority safety, hysteresis
- v1.4: Kalibrasi safety, reset safety

### 9.2 Validasi Safety
- Uji dengan robot pada kondisi berbahaya
- Uji false positive/negative
- Uji throttle safety
- Uji latching safety

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/pengaman.md` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/ui.md` untuk antarmuka