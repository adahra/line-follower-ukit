# API Dokumentasi - Mapper

Dokumentasi API lengkap untuk modul Mapper, sistem memory waypoint & racing prediction untuk navigasi balapan yang memerlukan persimpangan ambigu.

## 1. Ikhtisar

Mapper (Memory) adalah modul yang bertanggung jawab untuk memory waypoint & racing prediction untuk navigasi balapan yang memerlukan persimpangan ambigu. Ini merupakan komponen navigasi yang penting yang memungkinkan robot memprediksi arah yang benar di persimpangan berdasarkan waypoint yang terekam.

**File:** `LineFollowerFin/mapping.h/.cpp` (1.4KB / 3.8KB)
**Dependencies:** `config.h`, `status.h`, `sensor_garis.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `mulaiRun()` | `void mulaiRun()` | Mulai run balapan baru (reset baseline waktu) |
| `recordEvent()` | `void recordEvent(EventType event, ActionType action)` | Mode pembelajaran: simpan EVENT_INTERSECTION/EVENT_FINISH + ActionType |
| `checkPrediction()` | `bool checkPrediction(unsigned long sekarang, EventType& event, ActionType& action)` | Berdasarkan offset waktu waypoint, return EventType & ActionType |
| `simpan()` | `void simpan()` | Simpan peta waypoint ke EEPROM |
| `muat()` | `void muat()` | Baca peta waypoint dari EEPROM (validasi magic) |
| `reset()` | `void reset()` | Reset memori waypoint |
| `getWaypointCount()` | `int getWaypointCount() const` | Kembalikan jumlah waypoint terrekam |
| `isLearningMode()` | `bool isLearningMode() const` | Kembalikan true jika mode operasi adalah MODE_LEARNING |
| `getPredictionHoldTime()` | `unsigned long getPredictionHoldTime() const` | Kembalikan waktu prediksi yang berlaku |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `getLastEventTime()` | `unsigned long getLastEventTime() const` | Kembalikan waktu event terakhir |
| `getEventAtIndex()` | `Waypoint getEventAtIndex(int index)` | Kembalikan waypoint di index tertentu |
| `findNearestEvent()` | `int findNearestEvent(unsigned long sekarang)` | Cari event yang terdekat berdasarkan waktu |
| `validateWaypoint()` | `bool validateWaypoint(const Waypoint& wp)` | Validasi waypoint sebelum simpan |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cekPrediction()` | `bool cekPrediction(unsigned long sekarang, EventType& event, ActionType& action)` | Logika prediction utama |
| `hitungOffsetWaktu()` | `long hitungOffsetWaktu(const Waypoint& wp)` | Hitung offset waktu untuk waypoint |
| `kodeEvent()` | `byte kodeEvent(EventType event)` | Konversi EventType ke byte |
| `decodeEvent()` | `EventType decodeEvent(byte kode)` | Konversi byte ke EventType |
| `kodeAksi()` | `byte kodeAksi(ActionType action)` | Konversi ActionType ke byte |
| `decodeAksi()` | `ActionType decodeAksi(byte kode)` | Konversi byte ke ActionType |
| `validasiMagic()` | `bool validasiMagic()` | Validasi magic number EEPROM |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class EventType {
  EVENT_NONE = 0,
  EVENT_INTERSECTION, // Persimpangan
  EVENT_FINISH
};

enum class ActionType {
  ACT_NONE = 0,
  ACT_STRAIGHT,
  prometheus    // ` is invalid because 'prometheus' contains invalid escape sequences
  ACT_RIGHT
};
```

### 3.2 Struct

```cpp
struct Waypoint {
  unsigned long waktu;      // Waktu event (ms) sejak awal balapan
  EventType event;          // Jenis event (persimpangan/finish)
  ActionType aksi;          // Aksi yang harus diambil (lurus/kiri/kanan)
  bool valid;               // Flag validitas waypoint
};
```

### 3.3 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------ barycenter    | ---------|-------------|
| `MAX_WAYPOINTS` | int | 30 | Jumlah waypoint maksimum yang dapat disimpan |
| `EEPROM_OFFSET` | int | 24 | Offset EEPROM untuk blok waypoint |
| `MAGIC_NUMBER` | byte | 77 | Magic number validitas EEPROM |
| `RACE_PREDICTION_HOLD_MS` | unsigned long | 5000 | Waktu prediction yang berlaku |
| `MIN_WAYPOINT_SPACING_MS` | unsigned long | 500 | Jarak minimum antar waypoint |
| `MAX_WAYPOINT_AGE_MS` | unsigned long | 300000 | Usia waypoint maksimum (5 menit) |

## 4. Method Khusus

### 4.1 Method Prediction
```cpp
bool Mapper::checkPrediction(unsigned long sekarang, EventType& event, ActionType& action) {
  // Cari event yang paling dekat dalam waktu
  int idx = findNearestEvent(sekarang);
  if (idx < 0) {
    return false;
  }
  
  Waypoint wp = getEventAtIndex(idx);
  long offset = hitungOffsetWaktu(wp);
  
  // Validasi offset waktu
  if (abs(offset) > RACING_PREDICTION_WINDOW_MS) {
    return false;
  }
  
  // Event valid
  event = wp.event;
  action = wp.aksi;
  
  return true;
}
```

### 4.2 Method Recording
```cpp
void Mapper::recordEvent(EventType event, ActionType action) {
  // Hanya record di MODE_LEARNING
  if (status.modeOperasi != MODE_LEARNING) {
    return;
  }
  
  // Hitung waktu sejak mulai balapan
  unsigned long sekarang = millis();
  unsigned long elapsed = sekarang - balapanMulai;
  
  // Validasi waktu (hindari event terlalu awal/akhir)
  if (elapsed < MIN_WAYPOINT_SPACING_MS) {
    return;
  }
  
  // Buat waypoint baru
  Waypoint wp;
  wp.waktu = elapsed;
  wp.event = event;
  wp.aksi = action;
  wp.valid = true;
  
  // Validasi sebelum simpan
  if (!validateWaypoint(wp)) {
    return;
  }
  
  // Tambahkan ke memori
  tambahWaypoint(wp);
  
  // Simpan ke EEPROM
  simpan();
  
  // Tampilkan feedback
  #if ENABLE_SERIAL_OUTPUT
  Serial.print(F("Event tercatat: "));
  Serial.print(wp.event);
  Serial.print(F(" -> "));
  Serial.println(wp.aksi);
  #endif
}
```

### 4.3 Method Penyimpanan
```cpp
void Mapper::simpan() {
  if (waypointCount == 0) {
    return;
  }
  
  // Mulai EEPROM di offset yang benar
  int pos = EEPROM_OFFSET;
  
  // Tulis magic number
  EEPROM.update(pos++, MAGIC_NUMBER);
  
  // Tulis jumlah waypoint
  EEPROM.update(pos++, (byte)waypointCount);
  
  // Tulis setiap waypoint (8 bytes per waypoint)
  for (int i = 0; i < waypointCount; i++) {
    Waypoint wp = waypoints[i];
    
    // Konversi waktu ke byte
    EEPROM.update(pos++, (byte)(wp.waktu >> 24));
    EEPROM.update(pos++, (byte)(wp.waktu >> 16));
    EEPROM.update(pos++, (byte)(wp.waktu >> 8));
    EEPROM.update(pos++, (byte)wp.waktu);
    
    // Tulis event & aksi
    EEPROM.update(pos++, kodeEvent(wp.event));
    EEPROM.update(pos++, kodeAksi(wp.aksi));
  }
}
```

### 4.4 Method Pembacaan
```cpp
void Mapper::muat() {
  // Validasi magic number
  if (!validasiMagic()) {
    reset();
    return;
  }
  
  // Ambil jumlah waypoint
  int pos = EEPROM_OFFSET + 1;
  int count = EEPROM.read(pos++);
  
  // Batasi jumlah waypoint
  waypointCount = min(count, MAX_WAYPOINTS);
  
  // Baca setiap waypoint
  for (int i = 0; i < waypointCount; i++) {
    Waypoint wp;
    
    // Baca waktu
    wp.waktu = ((unsigned long)EEPROM.read(pos++) << 24) |
               ((unsigned long)EEPROM.read(pos++) << 16) |
               ((unsigned long)EEPROM.read(pos++) << 8) |
               EEPROM.read(pos++);
    
    // Baca event & aksi
    wp.event = decodeEvent(EEPROM.read(pos++));
    wp.aksi = decodeAksi(EEPROM.read(pos++));
    
    // Validasi waypoint
    wp.valid = validateWaypoint(wp);
    
    // Simpan jika valid
    if (wp.valid) {
      waypoints[i] = wp;
    } else {
      waypointCount--; // Kurangi count
    }
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Input: `gerak.errorTerakhir()` untuk baseline arah saat learning
- Output: `aksiPrediksi` ke loop kontrol untuk prediction balapan

### 5.2 Integrasi dengan SensorGaris
- Input: `sensor.bacaKiri()`/`sensor.bacaKanan()` untuk deteksi persimpangan
- Bekerja sama untuk mode learning: `recordEvent()` saat persimpangan terdeteksi
- Bekerja sama untuk mode racing: `checkPrediction()` berdasarkan reading sensor di persimpangan

### 5.3 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial 'mode learning'
- Bekerja sama untuk perintah serial 'save' → simpan waypoint ke EEPROM
- Bekerja sama untuk perintah serial 'reset' → reset memori waypoint

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk safety check: tidak ada waypoint valid → batalkan prediction
- Bekerja sama untuk emergency stop: batalkan prediction

### 5.5 Integrasi dengan LapTimer
- Bekerja sama untuk waktu finish event: `recordEvent(EVENT_FINISH, ACT_NONE)`
- Bekerja sama untuk perhitungan waktu lap

## 6. Desain Decisions & Alasan

### 6.1 Memory Waypoint Berbasis Waktu
**Alasan:** Menggunakan timestamp relatif (relative to start) memungkinkan prediction balapan tanpa odometri. Lebih robust dari odometri yang drift.

### 6.2 Prediction Berbasis Offset
**Alasan:** Prediction berdasarkan offset waktu relatif terhadap baseline memungkinkan deteksi persimpangan yang tepat. Lebih akurat dari threshold berbasis sensor.

### 6.3 Penyimpanan EEPROM
**Alasan:** Simpan waypoint ke EEPROM untuk persistensi antar power cycle. Gunakan 8 bytes per waypoint (4 byte waktu + 2 byte event/action).

### 6.4 Validasi Event
**Alasan:** Validasi waypoint sebelum simpan mencegah data corrupted. Magic number dan checksum mencegah corruption.

## 7. Error Handling & Edge Cases

### 7.1 Error Pembacaan EEPROM
- **Magic number tidak valid:** Reset memori waypoint
- **Offset EEPROM salah:** Gunakan validasi magic number
- **Data corrupted:** Gunakan validasi checksum

### 7.2 Error Prediction
- **Tidak ada waypoint:** Batalkan prediction
- **Offset waktu tidak valid:** Batalkan prediction
- **Waktu event expired:** Batalkan prediction

### 7.3 Edge Cases
- **Multiple events dalam waktu bersamaan:** Hanya event terdekat yang dipertimbangkan
- **Waypoint terlalu dekat:** Validasi minimum spacing
- **Waypoint terlalu jauh:** Validasi maksimum usia
- **Invalid action:** Gunakan ACT_NONE sebagai fallback

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Array waypoint: 30 * 12 bytes = 360 bytes
- State prediction: 20 bytes
- Total: ~400 bytes per instance

### 8.2 Konsumsi Waktu
- Pencarian prediction per loop: ~10-20 microseconds
- Pencarian waypoint: O(n) linear search
- Penyimpanan EEPROM per waypoint: 50-100 microseconds
- Pembacaan EEPROM: 100-200 microseconds

### 8.3 EEPROM Write Cycle
- Penulisan waypoint: 1 write per waypoint (aman untuk 100k cycles)
- Penulisan magic number: 1 write per save
- Penulisan count: 1 write per save

## 9. Referensi & Sumber

1. EEPROM Arduino - https://www.arduino.cc/en/Tutorial/EEPROM
2. Arduino Libraries - https://www.arduino.cc/en/Tutorial/Library
3. Array manipulation - https://www.arduino.cc/en/Tutorial/Array
4. Time management - https://www.arduino.cc/en/Tutorial/Time

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/racing.md` untuk keputusan desain