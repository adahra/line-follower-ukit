# Modul Antarmuka - Dokumentasi Teknis

## 1. Ikhtisar

Antarmuka (UI) adalah layer input manusia yang memungkinkan operator mengendalikan robot via tombol onboard, tepuk tangan, atau serial command. Ini menyediakan mode tuning, pembelajaran, racing, dan kontrol RC.

**File:** `LineFollowerFin/antarmuka.h/.cpp` (1.3KB / 27.0KB)
**Dependencies:** `Arduino.h`, `fwd.h`, `config.h`, `status.h`, `SensorGaris`, `Penggerak`, `Pengaman`, `LapTimer`, `Mapper`

## 2. Desain & Arsitektur

### 2.1 Input Manusia
- **Tombol:** Dua tombol onboard (B1, B2) dengan combo B1+B2 untuk kalibrasi otomatis
- **Tepuk:** Sensor suara (sound sensor ID 1) untuk toggle jalan/berhenti
- **Serial:** Parsing command serial (Kp/Ki/Kd/speed/ambang/calibrate/save/help)
- **Bluetooth:** Opsional (via BluetoothCtrl)
- **RC:** Opsional (via RcCtrl)

### 2.2 Arsitektur Input
- **Non-blocking:** Semua pengecekan punya cooldown untuk mencegah blocking loop
- **Atomic:** Combo tombol B1+B2 menggunakan flag volatile untuk menghindari race condition
- **Prioritas:** Tombol → tepuk → serial (jika diaktifkan)
- **Throttling:** Setiap input punya throttle untuk mencegah pembacaan berlebihan

### 2.3 Integrasi Sistem
```cpp
Antarmuka ui(status, sensor, gerak, aman, lapTimer, mapper);
```

**Dependency injection:** Semua reference status dan komponen dibagikan via reference

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cekTombol()` | `void` | Non-blocking pengecekan tombol dengan cooldown |
| `cekTepuk()` | `void` | Deteksi tepuk tangan untuk toggle jalan/berhenti |
| `cekSerial()` | `void` | Parsing command serial |

### 3.ację Method Private

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `mulai()` | `void(const char *via)` | Mulai robot (reset semua state) |
| `berhenti()` | `void(const char *via)` | Hentikan robot (reset state) |
| `resetSemua()` | `void` | Hentikan + reset semua modul |
| `parseCommand()` | `void(char *cmd)` | Parse string command |
| `tuningSequence()` | `void` | Combo tombol B1+B2 untuk auto-tune relay |
| `tampilkanStatusJalannya()` | `void` | Tampilkan status di serial |

### 3.3 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `status` | `Status &` | Reference status jalan |
| `sensor` | `SensorGaris &` | Reference pembacaan sensor |
| `gerak` | `Penggerak &` | Reference kontrol motor |
| `aman` | `Pengaman &` | Reference safety |
| `lapTimer` | `LapTimer &` | Reference lap timing |
| `mapper` | `Mapper &` | Reference waypoint memory |
| `cooldownB1` | `unsigned long` | Throttle tombol B1 (COOLDOWN_TOMBOL_MS) |
| `cooldownB2` | `unsigned long` | Throttle tombol B2 (COOLDOWN_TOMBOL_MS) |
| `comboActive` | `volatile bool` | Flag combo B1+B2 aktif |
| `comboMulai` | `unsigned long` | Waktu combo dimulai |
| `comboSelesai` | `bool` | True = auto-tune sudah berjalan |
| `cekSuaraTerakhir` | `unsigned long` | Throttle terakhir untuk sensor suara |
| `tepukTerakhir` | `unsigned long`山东 | Throttle terakhir untuk tepuk tangan |
| `tepukSiap` | `bool` | Siap untuk tepuk berikutnya |

## 4. Implementasi Detil

### 4.1 Tombol Input
```cpp
void Antarmuka::cekTombol() {
  if (status.modeSetelKd == true) {
    // Mode tuning Kd: B1 naik Kp/Kd
    if (digitalRead(B1_PIN) == LOW && millis() - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
      tambahKp(0.1);
      tambahKd(0.1);
      cooldownB1 = millis();
    }
  } else {
    // Mode normal: B1 toggle jalan/berhenti
    if (digitalRead(B1_PIN) == LOW && millis() - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
      if (status.jalan) {
        mulai("tombol");
      } else {
        berhenti("tombol");
      }
      cooldownB1 = millis();
    }
  }
}
```

### 4.2 Combo Tombol (B1+B2)
```cpp
void Antarmuka::tuningSequence() {
  if (comboActive && millis() - comboMulai >= CALIB_MANUAL_TIMEOUT_MS) {
    // Timeout: batalkan
    comboActive = false;
    comboSelesai = false;
    return;
  }
  
  if (comboActive && !comboSelesai) {
    // Jalankan relay feedback auto-tune
    // Gunakan relay PID untuk osilasi dan hitung Kp/Ki/Kd kritis
    // Terapkan Zinger-Nichols tuning
    float KpKritis = hitungKpKritis();
    float Ti = hitungPeriodeKritis();
    float Kd = 0.6 * KpKritis * Ti;
    float Ki = 1.2 * KpKritis / Ti;
    
    // Terapkan tuning (simpan ke EEPROM)
    simpanKpKiKd(KpKritis, Ki, Kd);
    
    comboSelesai = true;
    tampilkanStatusJalannya();
  }
}
```

### 4.3 Parsing Command Serial
```cpp
void Antarmuka::parseCommand(char *cmd) {
  if (strncmp(cmd, "kp ", 2) == 0) {
    float Kp = atof(cmd + 2);
    setKp(Kp);
  } else if (strncmp(cmd, "ki ", 2) == 0) {
    float Ki = atof(cmd + 2);
    setKi(Ki);
  } else if (strncmp(cmd, "kd ", 2) == 0) {
    float Kd = atof(cmd + 2);
    setKd(Kd);
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    int speed = atoi(cmd + 6);
    setKecepatanDasarMaks(speed);
  } else if (strncmp(cmd, "ambang ", 7) == 0) {
    int ambang = atoi(cmd + 7);
    setAmbang(ambang);
  } else if (strcmp(cmd, "calibrate") == 0) {
    sensor.kalibrasi(gerak, false);  // otomatis
  } else if (strcmp(cmd, "save") == 0) {
    simpanSemuaKe EEPROM;
  } else if (strcmp(cmd, "help") == 0) {
    tampilkanHelp();
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Status
- Write `status.jalan` (mulai/berhenti)
- Write `status.modeSetelKd` (toggle)
- Read `status.modeOperasi` (untuk LED indication)
- Read `status.subState` (untuk debugging)

### 5.2 Integrasi dengan SensorGaris
- Bekerja sama untuk kalibrasi: `sensor.kalibrasi()`
- Bekerja sama untuk simpan/muat: `sensor.simpan()`, `sensor.muat()`
- Bekerja sama untuk adaptive threshold: `sensor.setAdaptiveThreshold()`

### 5.3 Integrasi dengan Penggerak
- Bekerja sama untuk tuning PID: `gerak.hitung()` -> command PID
- Bekerja sama untuk kontrol manual: `gerak.maju()`, `gerak.putarKiri()`
- Bekerja sama untuk emergency stop: `gerak.berhenti()`

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk safety: `aman.bateraiKritis()`, `aman.miring()`
- Bekerja sama untuk lampu: `aman.lampuOtomatis()`
- Bekerja sama untuk reset semua: `aman.resetDeteksi()`

### 5.5 Integrasi dengan Mapper
- Bekerja sama untuk penyimpanan: `mapper.simpan()`, `mapper.muat()`
- Bekerja sama untuk mode: `status.modeOperasi = MODE_LEARNING`

### 5.6 Integrasi dengan Bluetooth & RC
- Bekerja sama untuk command processing: `bluetoothCtrl.setCommandCallback()`
- Bekerja sama untuk mode: `status.modeOperasi = MODE_RC`

## 6. Desain Decisions & Alasan

### 6.1 Non-blocking Input
**Alasan:** Loop robot real-time. Blocking pada pembacaan sensor input akan membuat robot kehilangan pelacakan garis. Cooldown millis() memastikan loop kontrol selesai tepat waktu.

### 6.2 Atomic Combo Input
**Alasan:** Combo tombol B1+B2 harus dideteksi tanpa race condition. Flag volatile memastikan atomicitas.

### 6.3 Parsing Command Serial
**Alasan:** Command serial harus di-parse cepat tanpa blocking. String comparison vs parsing.

### 6.4 Cooldown Tombol
**Alasan:** Mencegah false trigger dari bouncing tombol mekanis. COOLDOWN_TOMBOL_MS = 300ms.

### 6.5 Mode Tuning
**Alasan:** Separasi mode tuning (Kd) dari mode operasi normal. Tombol B1 behaves berbeda tergantung pada `modeSetelKd`.

## 7. Error Handling & Edge Cases

### 7.1 Error Parsing
- **String kosong:** Abaikan
- **Format salah:** Tampilkan error di serial
- **Value out of range:** Clamp ke batas yang valid
- **Command tidak valid:** Tampilkan error

### 7.2 Error Input
- **Tombol stuck:** Cooldown mencegah spam
- **Tepuk ganda:** Histeresis + cooldown mencegah duplicate toggle
- **Serial buffer overflow:** Gunakan queue atau buffer terbatas

### 7.3 Edge Cases
- **Startup:** Semua mode direset ke default
- **EEPROM invalid:** Kalibrasi otomatis (jika diaktifkan)
- **Bluetooth tidak connected:** Tetap terima command serial
- **RC signal lost:** Hentikan robot (melalui Pengaman)

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `COOLDOWN_TOMBOL_MS` | 300 | 100-500 | Cooldown tombol |
| `TEPUK_AMBANG` | 500 | 400-600 | Ambang sensor suara |
| `TEPUK_LEPAS` | 350 | 300-400 | Histeresis suara |
| `CEK_SUARA_MS` | 30 | 10-50 | Throttle pembacaan suara |
| `TEPUK_COOLDOWN_MS` | 800 | 500-1000 | Cooldown tepuk |
| `COMBO_TIMEOUT_MS` | 15000 | 10000-20000 | Timeout kalibrasi manual |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- State: ~14 bytes (references)
- Throttle counters: ~4 bytes
- Buffer: ~128 bytes (serial command)
- Total: ~150 bytes per instance

### 9.2 Konsumsi Waktu
- Cek tombol per loop: ~10-20 microseconds
- Cek tepuk per loop: ~5-10 microseconds
- Parsing serial (jika command): ~50-100 microseconds
- Total per loop: 0.1-0.2 milliseconds

### 9.3 Penggunaan Flash
- Implementation: ~1KB (header) + ~2KB (cpp)
- String constants: ~500 bytes
- Total: ~3.5KB

## 10. Referensi & Sumber

1. Arduino Digital Read - https://www.arduino.cc/en/Tutorial/DigitalReadSerial
2. Debounce - https://www.arduino.cc/en/Tutorial/Debounce
3. String Parsing - https://www.arduino.cc/en/Tutorial/StringManipulation
4. PID Tuning - https://www.arduino.cc/en/Tutorial/PIDLibrary

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar tombol & serial
- v1.1: Tambahkan tepuk tangan & combo input
- v1.2: Tambahkan parsing command serial
- v1.3: Tambahkan tuning sequence & auto-tune
- v1.4: Tambahkan error handling & validation
- v1.5: Perbaikan bug & performance

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/antarmuka.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/ui.md` untuk keputusan desain

