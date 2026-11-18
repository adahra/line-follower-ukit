# API Dokumentasi - LapTimer

Dokumentasi API lengkap untuk modul LapTimer, sistem perhitungan waktu & statistik lap (opsional) untuk kompetisi balapan yang memerlukan timing presisi.

## 1. Ikhtisar

LapTimer (Lap Timer) adalah modul yang bertanggung jawab untuk hitung waktu & statistik lap (opsional) untuk kompetisi balapan yang memerlukan timing presisi. Ini merupakan komponen timing yang penting yang memungkinkan timing lap yang akurat, recording finish event, dan telemetry statistik balapan.

**File:** `LineFollowerFin/lap_timer.h/.cpp` (2.1KB / 5.2KB)
**Dependencies:** `config.h`, `sensor_garis.h`, `penggerak.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void begin()` | Inisialisasi lap timer |
| `update()` | `void update(int bacaKiri, int bacaKanan, int kecepatanDasar, float error)` | Deteksi start/finish (kedua sensor ≥ THRESHOLD), hitung error/time avg/max |
| `printCSV()` | `void printCSV()` | Output telemetri di interval TELEM_MS |
| `reset()` | `void reset()` | Reset semua data lap |
| `simpan()` | `void simpan()` | Simpan data lap ke EEPROM |
| `muat()` | `void muat()` | Baca data lap dari EEPROM |
| `getLapCount()` | `int getLapCount() const` | Kembalikan jumlah lap yang selesai |
| `getLapTime()` | `unsigned long getLapTime(int lapIndex) const` | Kembalikan waktu lap di index tertentu |
| `getFinishTime()` | `unsigned long getFinishTime() const` | Kembalikan waktu finish balapan |
| `getAverageLapTime()` | `float getAverageLapTime() const` | Kembalikan waktu rata-rata lap |
| `isLapFinished()` | `bool isLapFinished() const` | Kembalikan true jika lap saat ini selesai |
| `isRaceFinished()` | `bool isRaceFinished() const` | Kembalikan true jika balapan selesai |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `getCurrentLap()` | `int getCurrentLap() const` | Kembalikan nomor lap saat ini (0 = belum start) |
| `getTotalRaceTime()` | `unsigned long getTotalRaceTime() const` | Kembalikan total waktu balapan |
| `getFastestLapTime()` | `float getFastestLapTime() const` | Kembalikan waktu lap tercepat |
| `getSlowestLapTime()` | `float getSlowestLapTime() const` | Kembalikan waktu lap terjangkit |
| `getLapErrorRange()` | `float getLapErrorRange() const` | Kembalikan range error antar lap |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cekStartFinish()` | `void cekStartFinish(int bacaKiri, int bacaKanan)` | Deteksi start/finish event |
| `hitungStatistik()` | `void hitungStatistik()` | Hitung statistik lap |
| `updateTelemetry()` | `void updateTelemetry()` | Update telemetry CSV |
| `cekanLapTimeout()` | `bool cekanLapTimeout() const` | Cek timeout lap |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class LapState {
  BELUM_START = 0,
  SEDANG_BERLANJUT,
  SELESAI
};

enum class RaceState {
  SEDANG_BERLANJUT = 0,
  SELESAI
};
```

### 3.2 Struct

```cpp
struct LapData {
  unsigned long startTime;     // Waktu start lap (ms)
  unsigned long finishTime;    // Waktu finish lap (ms)
  unsigned long duration;      // Durasi lap (ms)
  float errorAvg;              // Error rata-rata selama lap
  float errorMax;              // Error maksimum selama lap
  float errorMin;              // Error minimum selama lap
  bool valid;                  // Flag validitas lap
};
```

### 3.3 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `MAX_LAPS` | int | 15 | Jumlah lap maksimum yang dapat disimpan |
| `LAP_START_THRESHOLD` | int | 15 | Threshold sensor untuk start lap |
| `LAP_FINISH_THRESHOLD` | int | 15 | Threshold sensor untuk finish lap |
| `LAP_TIMEOUT_MS` | unsigned long | 300000 | Timeout maksimum per lap (5 menit) |
| `TELEM_MS` | unsigned long | 1000 | Interval telemetri CSV |
| `LAP_SF_THRESHOLD` | int | 15 | Threshold untuk start/finish lap |
| `LAP_MIN_LAP_MS` | unsigned long | 10000 | Waktu minimum lap (10 detik) |

## 4. Method Khusus

### 4.1 Method Update
```cpp
void LapTimer::update(int bacaKiri, int bacaKanan, int kecepatanDasar, float error) {
  unsigned long sekarang = millis();
  
  // Deteksi start lap (kedua sensor ≥ THRESHOLD)
  if (lapState == LAP_STATE_BELUM_START && 
      bacaKiri >= LAP_START_THRESHOLD && bacaKanan >= LAP_START_THRESHOLD) {
    lapState = LAP_STATE_SEDANG_BERLANJUT;
    lapSaatIni.startTime = sekarang;
    lapSaatIni.errorAvg = error;
    lapSaatIni.errorMax = abs(error);
    lapSaatIni.errorMin = abs(error);
    lapSaatIni.valid = true;
  
  // Deteksi finish lap (kedua sensor ≥ THRESHOLD)
  } else if (lapState == LAP_STATE_SEDANG_BERLANJUT && 
             bacaKiri >= LAP_FINISH_THRESHOLD && bacaKanan >= LAP_FINISH_THRESHOLD) {
    lapState = LAP_STATE_SELESAI;
    lapSaatIni.finishTime = sekarang;
    lapSaatIni.duration = lapSaatIni.finishTime - lapSaatIni.startTime;
    
    // Validasi durasi lap
    if (lapSaatIni.duration < LAP_MIN_LAP_MS) {
      lapSaatIni.valid = false;
    }
    
    // Simpan data lap
    simpanDataLap(lapSaatIni);
    hitungStatistik();
    
    // Reset untuk lap berikutnya
    lapSaatIni.startTime = 0;
    lapSaatIni.finishTime = 0;
    lapSaatIni.duration = 0;
    
    // Cek finish balapan (MAX_LAPS lap)
    if (lapCount >= MAX_LAPS) {
      balapanSelesai = true;
      finishTime = sekarang;
    }
  
  // Update statistik error selama lap
  } else if (lapState == LAP_STATE_SEDANG_BERLANJUT) {
    lapSaatIni.errorAvg = (lapSaatIni.errorAvg * (errorCount + 1) + error) / (errorCount + 1);
    lapSaatIni.errorMax = max(lapSaatIni.errorMax, abs(error));
    lapSaatIni.errorMin = min(lapSaatIni.errorMin, abs(error));
    errorCount++;
  }
}
```

### 4.2 Method Print CSV
```cpp
void LapTimer::printCSV() {
  Serial.print(F("Lap#"));
  Serial.print(F(","));
  Serial.print(F("Start(ms)"));
  Serial.print(F(","));
  Serial.print(F("Finish(ms)"));
  Serial.print(F(","));
  Serial.print(F("Duration(ms)"));
  Serial.print(F(","));
  Serial.print(F("ErrorAvg"));
  Serial.print(F(","));
  Serial.print(F("ErrorMax"));
  Serial.print(F(","));
  Serial.print(F("ErrorMin"));
  Serial.print(F(","));
  Serial.print(F("Valid"));
  Serial.println();
  
  for (int i = 0; i < lapCount; i++) {
    LapData data = lapDataArray[i];
    
    Serial.print(i + 1);
    Serial.print(F(","));
    Serial.print(data.startTime);
    Serial.print(F(","));
    Serial.print(data.finishTime);
    Serial.print(F(","));
    Serial.print(data.duration);
    Serial.print(F(","));
    Serial.print(data.errorAvg, 2);
    Serial.print(F(","));
    Serial.print(data.errorMax, 2);
    Serial.print(F(","));
    Serial.print(data.errorMin, 2);
    Serial.print(F(","));
    Serial.print(data.valid ? "1" : "0");
    Serial.println();
  }
  
  // Print total race time
  Serial.print(F("TotalRace(ms)"));
  Serial.print(F(","));
  Serial.println(getTotalRaceTime());
}
```

### 4.3 Method Penyimpanan
```cpp
void LapTimer::simpan() {
  // Mulai EEPROM di offset yang benar
  int pos = LAP_DATA_OFFSET;
  
  // Tulis magic number
  EEPROM.update(pos++, LAP_MAGIC_NUMBER);
  
  // Tulis jumlah lap
  EEPROM.update(pos++, (byte)lapCount);
  
  // Tulis setiap data lap (8 bytes per lap: 4 byte start, 4 byte finish)
  for (int i = 0; i < lapCount; i++) {
    LapData data = lapDataArray[i];
    
    // Tulis startTime (4 bytes)
    EEPROM.update(pos++, (byte)(data.startTime >> 24));
    EEPROM.update(pos++, (byte)(data.startTime >> 16));
    EEPROM.update(pos++, (byte)(data.startTime >> 8));
    EEPROM.update(pos++, (byte)data.startTime);
    
    // Tulis finishTime (4 bytes)
    EEPROM.update(pos++, (byte)(data.finishTime >> 24));
    EEPROM.update(pos++, (byte)(data.finishTime >> 16));
    EEPROM.update(pos++, (byte)(data.finishTime >> 8));
    EEPROM.update(pos++, (byte)data.finishTime);
    
    // Tulis flag valid (1 byte)
    EEPROM.update(pos++, data.valid ? 1 : 0);
  }
}
```

### 4.4 Method Pembacaan
```cpp
void LapTimer::muat() {
  // Validasi magic number
  if (!validasiMagic()) {
    reset();
    return;
  }
  
  // Ambil jumlah lap
  int pos = LAP_DATA_OFFSET + 1;
  int count = EEPROM.read(pos++);
  
  // Batasi jumlah lap
  lapCount = min(count, MAX_LAPS);
  
  // Baca setiap data lap
  for (int i = 0; i < lapCount; i++) {
    LapData data;
    
    // Baca startTime (4 bytes)
    data.startTime = ((unsigned long)EEPROM.read(pos++) << 24) |
                     ((unsigned long)EEPROM.read(pos++) << 16) |
                     ((unsigned long)EEPROM.read(pos++) << 8) |
                     EEPROM.read(pos++);
    
    // Baca finishTime (4 bytes)
    data.finishTime = ((unsigned long)EEPROM.read(pos++) << 24) |
                      ((unsigned long)EEPROM.read(pos++) << 16) |
                      ((unsigned long)EEPROM.read(pos++) << 8) |
                      EEPROM.read(pos++);
    
    // Baca flag valid (1 byte)
    data.valid = (EEPROM.read(pos++) == 1);
    
    // Hitung duration
    data.duration = data.finishTime - data.startTime;
    
    // Simpan data lap
    lapDataArray[i] = data;
  }
  
  hitungStatistik();
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Input: `gerak.errorTerakhir()` untuk error lap
- Input: `gerak.kecepatanDasar` untuk threshold speed
- Output: timing lap untuk robot

### 5.2 Integrasi dengan SensorGaris
- Input: `sensor.bacaKiri()`/`sensor.bacaKanan()` untuk deteksi start/finish
- Output: threshold start/finish lap (`LAP_START_THRESHOLD`, `LAP_FINISH_THRESHOLD`)

### 5.3 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial 'reset' → reset lap timer
- Bekerja sama untuk perintah serial 'save' → simpan data lap
- Bekerja sama untuk perintah serial 'print' → output CSV lap timer

### 5.排名与服务
- Sync LapTimer data dengan Bluetooth Telemetry
- Menerima perintah lap timer manual melalui remote

## 6. Desain Decisions & Alasan

### 6.1 Deteksi Start/Finish Berbasis Sensor
**Alasan:** Menggunakan threshold sensor IR yang sama dengan sistem kontrol tracer. Lebih robust dari marker fisik.

### 6.2 Penyimpanan Lap Timer Berbasis EEPROM
**Alasan:** Simpan data lap ke EEPROM untuk persistensi antar power cycle. Gunakan 8 bytes per lap (startTime + finishTime).

### 6.3 Deteksi Finish Lap Otomatis
**Alasan:** Deteksi finish otomatis saat kedua sensor ≥ THRESHOLD. Lebih akurat dari start/stop manual.

### 6.4 Deteksi Timeout Lap
**Alasan:** Validasi durasi lap (LAP_MIN_LAP_MS). Mencegah timeout balapan yang tidak valid.

## 7. Error Handling & Edge Cases

### 7.1 Error Deteksi Start/Finish
- **Start tidak valid:** Tidak ada start/finish lap
- **Finish tidak valid:** Lap tidak valid (duration < minimum)
- **Double start:** Deteksi start berturut-turut

### 7.2 Error Penyimpanan Data
- **Magic number tidak valid:** Reset lap timer
- **Data corrupted:** Gunakan validasi checksum
- **Offset EEPROM salah:** Gunakan validasi magic number

### 7.3 Edge Cases
- **Start/finish bertentangan:** Deteksi finish sebelum start lap
- **Multiple finish:** Deteksi finish berturut-turut
- **Invalid data:** Gunakan validasi flag valid
- **Timeout lap:** Hentikan lap setelah timeout

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Array data lap: 15 * 20 bytes = 300 bytes
- State lap timer: 20 bytes
- Total: ~320 bytes per instance

### 8.2 Konsumsi Waktu
- Update lap timer per loop: ~5-10 microseconds
- Pemeriksaan start/finish: ~2-5 microseconds
- Penulisan data lap: 50-100 microseconds saat finish
- Pembacaan data lap: 50-100 microseconds saat start

### 8.3 EEPROM Write Cycle
- Penulisan data lap: 1 write per lap (aman untuk 100k cycles)
- Penulisan magic number: 1 write per simpan

## 9. Referensi & Sumber

1. EEPROM Arduino - https://www.arduino.cc/en/Tutorial/EEPROM
2. Arduino Libraries - https://www.arduino.cc/en/Tutorial/Library
3. Time management - https://www.arduino.cc/en/Tutorial/Time
4. Data structures - https://www.arduino.cc/en/Tutorial/DataStructure

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/telemetry.md` untuk keputusan desain