# Modul LapTimer - Dokumentasi Teknis

## 1. Ikhtisar

LapTimer adalah modul statistik dan timing lap untuk LineFollowerFin. Ini menghitung waktu lap, kecepatan rata-rata, error, dan menyediakan output CSV untuk analisis.

**File:** `LineFollowerFin/lap_timer.h/.cpp` (2.1KB / 5.2KB)
**Dependencies:** `Arduino.h`, `config.h`

## 2. Desain & Arsitektur

### 2.1 Fitur Utama
- **Deteksi start/finish:** Dual sensor threshold (kedua sensor ≥ THRESHOLD)
- **Minimal durasi lap:** 3000ms per lap (prevent false trigger)
- **Statistik:** Waktu, error, kecepatan, max error
- **Output:** Serial CSV & summary
- **Storage:** Maks 15 laps (MAX_LAPS = 15)

### 2.2 Arsitektur Penyimpanan
- **RAM:** Array statis `LapRecord laps[LAP_STORAGE]`
- **Config:** Threshold start/finish & minimal lap duration
- **State:** Deteksi start/finish, hitungan lap

### 2.3 Integrasi Sistem
```cpp
LapTimer lapTimer;
```

**Digunakan oleh:** Loop utama LineFollowerFin.ino saat `ENABLE_LAP_TIMER = true`

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void` | Inisialisasi lap timer |
| `reset()` | `void` | Reset semua lap |
| `update()` | `bool update(int bacaKiri, int bacaKanan, int kecepatanDasar, float error)` | Update statistik lap, return true jika lap terdeteksi |
| `getLapCount()` | `int getLapCount() const` | Kembalikan jumlah lap |
| `getLastLapTime()` | `unsigned long getLastLapTime() const` | Kembalikan waktu lap terakhir |
| `getBestLapTime()` | `unsigned long getBestLapTime() const` | Kembalikan waktu lap terbaik |
| `getTotalTime()` | `unsigned long getTotalTime() const` | Kembalikan total waktu |
| `getAverageLapTime()` | `float getAverageLapTime() const` | Kembalikan rata-rata waktu lap |
| `printSummary()` | `void` | Tampilkan summary lap di serial |
| `printCSV()` | `void` | Tampilkan CSV lap di serial |
| `exportCSV()` | `void` | Alias printCSV |
| `setThresholds()` | `void setThresholds(int startFinishThreshold, unsigned long minLapMs)` | Konfigurasi threshold start/finish |

### 3.2 Data Types

```cpp
struct LapRecord {
  unsigned long lapTime;    // ms
  unsigned long timestamp;  // millis() saat finish
  float avgError;           // rata-rata error selama lap
  float maxError;           // error maksimum
  int avgSpeed;             // kecepatan dasar rata-rata
};
```

### 3.3 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `laps` | `LapRecord laps[LAP_STORAGE]` | Array record lap |
| `lapCount` | `int` | Jumlah lap |
| `wasAtStartFinish` | `bool` | State deteksi start/finish sebelumnya |
| `lastLapStartMs` | `unsigned long` | Waktu start lap terakhir |
| `lapStartMs` | `unsigned long` | Waktu start lap saat ini |
| `currentLapSamples` | `unsigned long` | Jumlah sampel saat ini |
| `currentLapErrorSum` | `float` | Total error saat ini |
| `currentLapMaxError` | `float` | Max error saat ini |
| `currentLapSpeedSum` | `long` | Total speed saat ini |
| `sfThreshold` | `int` | Threshold start/finish |
| `minLapMs` | `unsigned long` | Minimal lap duration |

## 4. Implementasi Detil

### 4.1 Method Update
```cpp
bool LapTimer::update(int bacaKiri, int bacaKanan, int kecepatanDasar, float error) {
  // Periksa start/finish (kedua sensor >= threshold)
  bool sekarangDiStartFinish = isAtStartFinish(bacaKiri, bacaKanan);
  
  if (sekarangDiStartFinish && !wasAtStartFinish) {
    // Start lap baru
    lapStartMs = millis();
    wasAtStartFinish = true;
    
    // Reset statistik lap saat ini
    currentLapSamples = 0;
    currentLapErrorSum = 0;
    currentLapMaxError = 0;
    currentLapSpeedSum = 0;
  } else if (!sekarangDiStartFinish && wasAtStartFinish) {
    // Finish lap
    unsigned long lapTime = millis() - lapStartMs;
    
    // Validasi lap (minimal durasi)
    if (lapTime >= minLapMs) {
      finishLap((currentLapErrorSum / currentLapSamples), currentLapMaxError, currentLapSpeedSum / currentLapSamples);
      return true;
    }
    
    // Reset start finish
    wasAtStartFinish = false;
    lapStartMs = 0;
  }
  
  // Update statistik lap saat ini (jika di start finish)
  if (wasAtStartFinish) {
    currentLapSamples++;
    currentLapErrorSum += fabs(error);
    currentLapMaxError = max(currentLapMaxError, fabs(error));
    currentLapSpeedSum += kecepatanDasar;
  }
  
  return false;
}
```

### 4.2 Method Finish Lap
```cpp
void LapTimer::finishLap(float avgError, float maxError, int avgSpeed) {
  unsigned long lapTime = millis() - lapStartMs;
  
  if (lapCount < MAX_LAPS) {
    laps[lapCount].lapTime = lapTime;
    laps[lapCount].timestamp = lapStartMs;
    laps[lapCount].avgError = avgError;
    laps[lapCount].maxError = maxError;
    laps[lapCount].avgSpeed = avgSpeed;
    lapCount++;
  }
  
  // Cetak CSV untuk telemetry
  printCSV();
}
```

### 4.3 Helper Functions
```cpp
bool LapTimer::isAtStartFinish(int bacaKiri, int bacaKanan) const {
  return (bacaKiri >= sfThreshold) && (bacaKanan >= sfThreshold);
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Loop Utama
- Dipanggil setiap loop saat `status.jalan == true`
- Input: `bacaKiri`, `bacaKanan`, `kecepatanDasar`, `error`
- Output: Deteksi lap baru, statistika lap

### 5.2 Integrasi dengan Penggerak
- Input: `kecepatanDasar` → untuk statistik kecepatan lap

### 5.3 Integrasi dengan SensorGaris
- Input: `bacaKiri`, `bacaKanan` → untuk deteksi start/finish

### 5.4 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial: `lap reset`, `lap print`
- Bekerja sama untuk telemetry: `ENABLE_TELEMETRI`

### 5.5 Integrasi dengan Bluetooth
- Bekerja sama untuk telemetry: Kirim data lap via Bluetooth
- Bekerja sama untuk summary: Kirim ringkasan lap via Bluetooth

### 5.6 Integrasi dengan Mapper
- Tidak langsung (opsional)
- Bekerja sama untuk racing: Hitung kecepatan lap di racing

## 6. Desain Decisions & Alasan

### 6.1 Dual Sensor Threshold
**Alasan:** Deteksi start/finish memerlukan kedua sensor mendeteksi garis (lebih andal daripada satu sensor).

### 6.2 Minimal Durasi Lap
**Alasan:** Mencegah false trigger dari noise atau gerakan tidak sengaja. 3000ms cukup untuk deteksi yang andal.

### 6.3 Array Statis
**Alasan:** Hindari malloc di embedded system. Maks 15 laps (MAX_LAPS) hemat RAM (15 x 20 = 300 bytes).

### 6.4 Statistik Real-time
**Alasan:** Update statistik setiap loop (throttle: PID interval) menghindari pembacaan sensor berlebihan.

### 6.5 Output CSV
**Alasan:** Format CSV mudah di-parse oleh tools analisis. Mendukung serial & Bluetooth output.

## 7. Error Handling & Edge Cases

### 7.1 Error Deteksi
- **False positive:** Minimal durasi lap mencegah lap cepat
- **False negative:** Threshold bisa tune
- **Sensor error:** Menggunakan dua sensor

### 7.2 Error Kalkulasi
- **Division by zero:** Cek `currentLapSamples > 0`
- **Overflow:** Gunakan tipe data yang tepat (unsigned long)
- **Underflow:** Lap time selalu positif

### 7.3 Edge Cases
- **Multiple finish:** Hanya lap yang valid (minimal durasi)
- **No lap:** Tidak ada record lap
- **Full memory:** Hanya simpan lap terbaru
- **Power cycle:** Reset semua lap

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `LAP_SF_THRESHOLD` | 12 | 8-16 | Threshold start/finish |
| `LAP_MIN_LAP_MS` | 3000 | 1000-5000 | Minimal lap duration |
| `MAX_LAPS` | 15 | 5-50 | Jumlah lap maksimum |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- Array lap: MAX_LAPS x sizeof(LapRecord) = 15 x 20 = 300 bytes
- State: ~20 bytes (tipe data primitive)
- Total: ~330 bytes per instance

### 9.2 Konsumsi Waktu
- Update per loop: ~100-200 microseconds
- Finish lap: ~50-100 microseconds
- Print CSV: ~200-500 microseconds
- Total per loop: 0.2-0.5 milliseconds

### 9.3 Penggunaan Flash
- Implementation: ~2KB
- Total: ~2.5KB

## 10. Referensi & Sumber

1. Arduino String - https://www.arduino.cc/en/Reference/String
2. Array Management - https://www.arduino.cc/en/Tutorial/Arrays
3. Time Functions - https://www.arduino.cc/en/Reference/Millis
4. CSV Format - https://tools.ietf.org/html/rfc4180

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar statistik lap
- v1.1: Tambahkan deteksi start/finish
- v1.2: Tambahkan minimal durasi lap
- v1.3: Tambahkan storage array
- v1.4: Tambahkan output CSV & summary
- v1.5: Perbaikan bug & tuning

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/lap_timer.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/timing.md` untuk keputusan desain

