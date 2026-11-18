# Modul Mapper - Dokumentasi Teknis

## 1. Ikhtisar

Mapper adalah sistem memory waypoint untuk mode Racing. Ini merekam event (persimpangan, finish) dengan timing relatif dan menyediakan prediction untuk balapan otonom.

**File:** `LineFollowerFin/mapping.h/.cpp` (1.4KB / 3.8KB)
**Dependencies:** `Arduino.h`, `config.h`, `status.h`

## 2. Desain & Arsitektur

### 2.1 Mode Pembelajaran vs Racing
- **Mode Learning:** Rekam event yang terjadi (`recordEvent()`)
- **Mode Racing:** Prediksi event berikutnya berdasarkan timing (`checkPrediction()`)

### 2.2 Memory Persisten
- **EEPROM:** Blok 24-266 (30 waypoint x 8 bytes)
- **RAM:** Array waypoint statis `Waypoint waypoints[MAX_WAYPOINTS]`
- **Magic bytes:** Validasi EEPROM (magic 77 di address 3, magic 123 di address 0)

### 2.3 Arsitektur Prediction
- **Baseline:** `runStartTime` untuk timing relatif
- **Prediction:** Berdasarkan `timeOffset` dari waypoint sebelumnya
- **Hold:** Event prediction aktif selama `RACE_PREDICTION_HOLD_MS` (1200ms)

### 2.4 Integrasi Sistem
```cpp
Mapper mapper;
```

**Reference:** `mapper` dibagikan antar modul melalui global variable

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `recordEvent()` | `void(EventType type, ActionType action)` | Rekam event di mode pembelajaran |
| `checkPrediction()` | `bool checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction)` | Prediksi event berikutnya (return true jika prediction valid) |
| `simpan()` | `void` | Simpan waypoints ke EEPROM |
| `muat()` | `void` | Muat waypoints dari EEPROM |
| `reset()` | `void` | Reset waypoints |
| `beginRun()` | `void` | Mulai run timer untuk baseline |
| `getWaypointCount()` | `int getWaypointCount() const` | Kembalikan jumlah waypoint terrekam |
| `debugPrint()` | `void` | Tampilkan waypoints di serial |

### 3.2 Data Types

```cpp
enum EventType {
  EVENT_NONE = 0,
  EVENT_INTERSECTION, // Persimpangan/Garis melintang
  EVENT_FINISH
};

enum ActionType {
  ACT_NONE = 0,
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT
};

struct Waypoint {
  EventType type;
  ActionType action;
  unsigned long timeOffset; // Waktu sejak waypoint sebelumnya (ms)
};
```

### 3.3 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `waypoints` | `Waypoint waypoints[MAX_WAYPOINTS]` | Array waypoint statis |
| `count` | `int` | Jumlah waypoint terrekam |
| `lastEventTime` | `unsigned long` | Waktu event terakhir |
| `runStartTime` | `unsigned long` | Baseline waktu start balapan |
| `lastPredictedIndex` | `int` | Index waypoint yang diprediksi terakhir |

## 4. Implementasi Detil

### 4.1 Mode Pembelajaran
```cpp
void Mapper::recordEvent(EventType type, ActionType action) {
  if (count < MAX_WAYPOINTS) {
    waypoints[count].type = type;
    waypoints[count].action = action;
    waypoints[count].timeOffset = millis() - lastEventTime;
    count++;
    lastEventTime = millis();
  }
}
```

### 4.2 Mode Racing
```cpp
bool Mapper::checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction) {
  if (count == 0) return false;  // Tidak ada waypoint
  
  // Hitung waktu relatif sejak run start
  unsigned long elapsed = currentTime - runStartTime;
  
  // Cari waypoint yang waktunya habis
  int predIndex = -1;
  for (int i = 0; i < count; i++) {
    if (elapsed >= (runStartTime + waypoints[i].timeOffset)) {
      // Waktu habis, periksa apakah sudah diprediksi
      if (i > lastPredictedIndex) {
        predIndex = i;
        lastPredictedIndex = i;
      }
    }
  }
  
  if (predIndex >= 0) {
    outType = waypoints[predIndex].type;
    outAction = waypoints[predIndex].action;
    return true;
  }
  
  return false;
}
```

### 4.3 Penyimpanan EEPROM
```cpp
void Mapper::simpan() {
  // Validasi: magic 77 di address 3, magic 123 di address 0
  // Tulis count + waypoint array (30 x 8 bytes)
  
  EEPROM.update(3, 77);  // Guard uKit
  
  // Tulis magic dan count
  EEPROM.update(0, 123);
  EEPROM.update(1, count);
  
  // Tulis array waypoint
  for (int i = 0; i < count; i++) {
    int addr = 2 + i * sizeof(Waypoint);
    EEPROM.put(addr, waypoints[i]);
  }
}
```

### 4.4 Loading EEPROM
```cpp
void Mapper::muat() {
  bool valid = (EEPROM.read(0) == 123) && (EEPROM.read(1) == 77);
  
  if (valid) {
    count = EEPROM.read(1);
    
    for (int i = 0; i < count; i++) {
      int addr = 2 + i * sizeof(Waypoint);
      EEPROM.get(addr, waypoints[i]);
    }
  } else {
    reset();
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Status
- Bekerja sama untuk `status.modeOperasi = MODE_RACING`
- Bekerja sama untuk `status.modeOperasi = MODE_LEARNING`
- Bekerja sama untuk `status.jalan` (mulai run)
- Bekerja sama untuk `status.subState` (prediction di persimpangan)

### 5.2 Integrasi dengan Penggerak
- Bekerja sama untuk action prediction (ACT_STRAIGHT/ACT_LEFT/ACT_RIGHT)
- Bekerja sama untuk racing prediction timing (currentTime)

### 5.3 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial: `path planner` commands
- Bekerja sama untuk save/load: `simpan()`/`muat()`

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk safety: halangan ultrasonic menghentikan prediction
- Bekerja sama untuk kalibrasi: reset waypoints saat kalibrasi

### 5.5 Integrasi dengan Bluetooth
- Bekerja sama untuk telemetry: kirim waypoints via Bluetooth
- Bekerja sama untuk command: terima commands prediction

## 6. Desain Decisions & Alasan

### 6.1 Array Statis
**Alasan:** Hindari malloc di embedded system. Array statis `MAX_WAYPOINTS = 30` hemat RAM (30 x 8 = 240 bytes).

### 6.2 Baseline Timer
**Alasan:** `runStartTime` menyediakan timing relatif yang konsisten antar balapan. Tanpa baseline, prediksi timing akan tidak akurat.

### 6.3 Prediction Hold
**Alasan:** Event prediction berlaku selama `RACE_PREDICTION_HOLD_MS` (1200ms). Mencegah prediction berulang di persimpangan yang sama.

### 6.4 Waypoint Time Offset
**Alasan:** Menyimpan waktu relatif antar waypoint memungkinkan timing prediction yang akurat. Berbasis waktu lebih andal daripada distance.

### 6.5 Magic Bytes Validasi
**Alasan:** Cegah corrupted data di EEPROM. Magic 77 (guard) + magic 123 (valid) memastikan data yang tersimpan valid.

## 7. Error Handling & Edge Cases

### 7.1 Error Waypoint
- **Overfill:** Cek `count < MAX_WAYPOINTS`
- **Time offset negatif:** Clamp ke minimum positif
- **Waypoint duplikat:** Abaikan (atau overwite?)

### 7.2 Error Prediction
- **Tidak ada waypoint:** Return false
- **Baseline tidak valid:** Gunakan runStartTime default
- **Timeout prediction:** Reset lastPredictedIndex

### 7.3 Edge Cases
- **Kalibrasi:** Reset waypoints
- **Power cycle:** Muat waypoints dari EEPROM
- **Full memory:** Hentikan recording
- **Corrupted EEPROM:** Reset ke default

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `MAX_WAYPOINTS` | 30 | 10-50 | Ukuran memory waypoint |
| `RACE_PREDICTION_HOLD_MS` | 1200 | 500-2000 | Duration prediction aktif |
| `RACE_PREDICTION_MARGIN` | 2 | 0-5 | Ambang selisih kiri-kanan untuk prediction |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- Array waypoint: 30 x 8 = 240 bytes
- State: ~4 bytes (int)
- Total: ~250 bytes per instance

### 9.2 Konsumsi Waktu
- Record event per loop: ~10-20 microseconds
- Prediction per loop: ~50-100 microseconds
- Save/load EEPROM: ~1-2 milliseconds
- Total per loop: 0.1-0.2 milliseconds

### 9.3 Penggunaan Flash
- Implementation: ~1KB
- Total: ~1.5KB

## 10. Referensi & Sumber

1. EEPROM - https://www.arduino.cc/en/Tutorial/EEPROM
2. Serial Communication - https://www.arduino.cc/en/Tutorial/SerialBegin
3. Time Functions - https://www.arduino.cc/en/Reference/Millis
4. Array Management - https://www.arduino.cc/en/Tutorial/Arrays

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar waypoint & prediction
- v1.1: Tambahkan penyimpanan EEPROM & loading
- v1.2: Tambahkan baseline timer & hold
- v1.3: Tambahkan error handling & validation
- v1.4: Perbaikan bug & tuning

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/mapping.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/racing.md` untuk keputusan desain

