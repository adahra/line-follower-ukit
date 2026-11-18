# API Dokumentasi - Antarmuka

Dokumentasi API lengkap untuk modul Antarmuka, sistem kontrol manusia yang menyediakan antarmuka tombol, tepuk tangan, dan serial command untuk berinteraksi dengan robot.

## 1. Ikhtisar

Antarmuka (UI) adalah modul yang bertanggung jawab untuk kontrol manusia: tombol, tepuk tangan, dan serial command untuk berinteraksi dengan robot. Ini merupakan komponen antarmuka pengguna yang penting yang memungkinkan operator mengendalikan robot, mengakses tuning, dan memantau status.

**File:** `LineFollowerFin/antarmuka.h/.cpp` (1.3KB / 27.0KB)
**Dependencies:** `status.h`, `sensor_garis.h`, `penggerak.h`, `pengaman.h`, `lap_timer.h`, `mapping.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cekTombol()` | `void cekTombol()` | Periksa tombol uKit non-blocking (cooldown 300ms) |
| `cekTepuk()` | `void cekTepuk()` | Toggle jalan/berhenti via tepuk tangan (bystander) |
| `cekSerial()` | `void cekSerial()` | Parsing command serial (Kp/Ki/Kd/speed/ambang/calibrate/save/help) |
| `parseCommand()` | `bool parseCommand(String cmd, String arg)` | Parse dan eksekusi command serial |
| `tuningSequence()` | `void tuningSequence()` | Combo tombol B1+B2 untuk auto-tune relay |
| `getStatus()` | `const Status& getStatus() const` | Kembalikan reference ke status object |
| `setMode()` | `void setMode(ModeOperasi mode)` | Set mode operasi robot |
| `toggleJalan()` | `void toggleJalan()` | Toggle jalan/berhenti state |
| `resetSemua()` | `void resetSemua()` | Reset semua modul (melalui Bluetooth) |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `tampilkanHelp()` | `void tampilkanHelp()` | Tampilkan daftar command yang tersedia di serial |
| `validasiCommand()` | `bool validasiCommand(String cmd)` | Validasi command sebelum eksekusi |
| `getCommandArg()` | `String getCommandArg(int idx)` | Ambil argumen command |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `prosesTombol()` | `void prosesTombol()` | Proses status tombol dengan debounce |
| `prosesTepuk()` | `void prosesTepuk()` | Proses deteksi tepuk tangan dengan cooldown |
| `prosesSerial()` | `void prosesSerial()` | Parsing command serial dengan timeout |
| `tanganiKalibrasi()` | `void tanganiKalibrasi()` | Handle command kalibrasi otomatis |
| `tanganiSimpan()` | `void tanganiSimpan()` | Handle command simpan ke EEPROM |
| `tanganiTune()` | `void tanganiTune()` | Handle command tuning parameter |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class TombolAksi {
  TANPA_TOMBOL = 0,
  PENDEK,
  PANJANG,
  KOMBINASI
};

enum class TepukAksi {
  TANPA_TEPUK = 0,
  SINGLE,
  GANDA,
  TRIPLE
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `COOLDOWN_TOMBOL_MS` | unsigned long | 300 | Cooldown tombol |
| `TEPUK_COOLDOWN_MS` | unsigned long |  Forsyth    | Cooldown tepuk tangan |
| `TEPUK_DETEKSI_MS` | unsigned long | 500 | Durasi deteksi tepuk tangan |
| `MAX_TOMBOL_PENDEK` | unsigned long | 200 | Durasi tombol pendek |
| `MAX_TOMBOL_PANJANG` | unsigned long | 500 | Durasi tombol panjang |
| `TOMBOL_KOMBINASI_DELAY` | unsigned long | 100 | Delay antar tombol untuk combo |

## 4. Method Khusus

### 4.1 Method Periksa Tombol
```cpp
void Antarmuka::cekTombol() {
  // Non-blocking, cooldown COLDOD_TOMBOL_MS
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < COOLDOWN_TOMBOL_MS) {
    return;
  }
  lastCheck = millis();
  
  TombolState state = bacaStatusTombol();
  
  if (state.kanalB1 == TOMBOL_PENDEK && !state.latchPendeck) {
    // Tombol pendek: switch Kp/Kd tergantung modeSetelKd
    if (status.modeSetelKd) {
      // modeSetelKd aktif: toggle Kd
      status.Kd = !status.Kd;
    } else {
      // modeSetelKd tidak aktif: toggle Kp
      status.Kp = !status.Kp;
    }
    state.latchPendeck = true;
    // Tampilkan feedback LED
    setRgbledColor(255, 255, 0); // Kuning = tuning active
  }
  
  if (state.kanalB2 == TOMBOL_PANJANG && !state.latchPanjang) {
    if (status.jalan) {
      // Tombol panjang sambil jalan: berhenti
      status.jalan = false;
      gerak.berhenti();
    } else {
      // Tombol panjang sambil berhenti: simpan EEPROM + start
      simpanSemuaKeEEPROM();
      status.jalan = true;
    }
    state.latchPanjang = true;
    // Tampilkan feedback LED
    setRgbledColor(0, 255, 255); // Cyan = simpan & jalan
  }
  
  if (state.kanalB1 == TOMBOL_PENDEK && state.kanalB2 == TOMBOL_PENDEK && 
      millis() - state.tombolTerakhir < TOMBOL_KOMBINASI_DELAY) {
    // Kombinasi tombol: kalibrasi otomatis 360° berbasis IMU
    mulaiKalibrasiOtomatis();
    state.latchKombinasi = true;
    setRgbledColor(255, 0, 255); // Magenta = kalibrasi
  }
}
```

### 4.2 Method Periksa Tepuk Tangan
```cpp
void Antarmuka::cekTepuk() {
  // Deteksi tepuk tangan dengan cooldown TEPUK_COOLDOWN_MS
  static unsigned long lastTepuk = 0;
  if (millis() - lastTepuk < TEPUK_COOLDOWN_MS) {
    return;
  }
  
  TepukAksi aksi = deteksiTepuk();
  if (aksi != TANPA_TEPUK) {
    lastTepuk = millis();
    toggleJalan();
    
    // Tampilkan feedback LED
    if (status.jalan) {
      setRgbledColor(0, 255, 0); // Hijau = jalan
    } else {
      setRgbledColor(255, 0, 0); // Merah = berhenti
    }
  }
}
```

### 4.3 Method Parsing Serial
```cpp
void Antarmuka::cekSerial() {
#if ENABLE_SERIAL_MODE
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Split command dan argumen
    int spasiIndex = input.indexOf(' ');
    String cmd = (spasiIndex > 0) ? input.substring(0, spasiIndex) : input;
    String arg = (spasiIndex > 0) ? input.substring(spasiIndex + 1) : "";
    
    parseCommand(cmd, arg);
  }
#endif
}
```

### 4.4 Method Parsing Command
```cpp
bool Antarmuka::parseCommand(String cmd, String arg) {
  if (!validasiCommand(cmd)) {
    return false;
  }
  
  if (cmd == "start") {
    status.jalan = true;
    status.modeOperasi = MODE_NORMAL;
    lapTimer.begin();
    mapper.mulaiRun();  // reset baseline waktu balapan
    return true;
  }
  
  if (cmd == "stop") {
    status.jalan = false;
    return true;
  }
  
  if (cmd == "tune") {
    status.modeSetelKd = true;
    return true;
  }
  
  if (cmd == "help") {
    tampilkanHelp();
    return true;
  }
  
  if (cmd == "calibrate") {
    mulaiKalibrasiOtomatis();
    return true;
  }
  
  if (cmd == "save") {
    simpanSemuaKeEEPROM();
    return true;
  }
  
  if (cmd == "reset") {
    resetSemua();
    return true;
  }
  
  if (cmd == "mode") {
    if (arg == "learning") {
      status.modeOperasi = MODE_LEARNING;
    } else if (arg == "racing") {
      status.modeOperasi = MODE_RACING;
    } else if (arg == "rc") {
      status.modeOperasi = MODE_RC;
    }
    return true;
  }
  
  // Tuning parameters
  if (cmd == "kp" && arg.length() > 0) {
    status.Kp = arg.toFloat();
    return true;
  }
  
  if (cmd == "ki" && arg.length() > 0) {
    status.Ki = arg.toFloat();
    return true;
  }
  
  if (cmd == "kd" && arg.length() > 0) {
    status.Kd = arg.toFloat();
    return true;
  }
  
  if (cmd == "speed" && arg.length() > 0) {
    status.kecepatanDasarMaks = arg.toInt();
    return true;
  }
  
  if (cmd == "ambang" && arg.length() > 0) {
    status.ambangKiri = arg.toInt();
    status.ambangKanan = arg.toInt();
    return true;
  }
  
  return false;
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Penggerak
- Used oleh `ui.parseCommand()` untuk setel Kp/Ki/Kd/speed/ambang
- Bekerja sama untuk modeSetelKd (toggle Kd)
- Bekerja sama untuk tuningSequence (mode B1+B2 combo)

### 5.2 Integrasi dengan SensorGaris
- Used oleh `Antarmuka::parseCommand()` untuk perintah 'calibrate'
- Membaca penyimpanan EEPROM untuk tuning
- Menampilkan status sensor di serial output

### 5.3 Integrasi dengan Pengaman
- Bekerja sama untuk perintah kontrol lampu manual
- Bekerja sama untuk deteksi tepuk tangan dan toggle lampu
- Bekerja sama untuk kalibrasi tombol lampu otomatis

### 5.4 Integrasi dengan Mapper
- Bekerja sama untuk perintah serial mode learning/racing
- Digunakan saat pembelajaran waypoint

### 5.5 Integrasi dengan Bluetooth
- Menerima command serial melalui Bluetooth
- Sinkronisasi command antar antarmuka Bluetooth dan serial

## 6. Desain Decisions & Alasan

### 6.1 UI Non-blocking
**Alasan:** Gunakan cooldown millis() bukan delay() → robot tetap bisa melacak garis saat UI merespon. Mencegah robot tersangkut saat tuning.

### 6.2 Cooldown Tombol & Tepuk
**Alasan:** Mencegah false positive dari noise mekanis atau akustik. Cooldown yang berbeda untuk tombol vs tepuk tangan berdasarkan characteristic fisik.

### 6.3 Parsing Command Serial
**Alasan:** Parsing command serial fleksibel dengan syntax sederhana. Mendukung perintah tunggal dan perintah dengan argumen.

### 6.4 Combo Kalibrasi Tombol
**Alasan:** Kalibrasi otomatis berbasis IMU memerlukan koordinasi antar tombol. Menggunakan tombol bersamaan menghindari konflik dengan fungsi lain.

## 7. Error Handling & Edge Cases

### 7.1 Error Parsing Command
- **Command tidak valid:** Tampilkan help, log error
- **Argumen hilang:** Tampilkan usage command
- **Format argumen salah:** Tampilkan error, reset command

### 7.2 Error Input Hardware
- **Debounce tombol:** Multiple press dalam waktu singkat dianggap sebagai satu
- **False positive tepuk:** Cooldown mencegah noise akustik
- **Serial buffer overflow:** Reset input stream

### 7.3 Edge Cases
- **Tombol kombinasi:** Waktu antar tombol < threshold dianggap sebagai combo
- **Tepuk cepat:** Multiple tepuk dalam waktu singkat toggle jalan beberapa kali
- **Command tidak valid:** Abaikan, tampilkan help

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Object tombol/tepuk: ~20 bytes
- Buffer command: 64 bytes
- Total: ~100 bytes per instance

### 8.2 Konsumsi Waktu
- Pemeriksaan tombol per loop: ~10-20 microseconds
- Pemeriksaan tepuk per loop: ~5-10 microseconds
- Parsing command: 50-100 microseconds saat command diterima
- Kalibrasi: 5-10 milliseconds (saat tombol combo)

### 8.3 EEPROM Write Cycle
- Tuning: 1 write per perubahan parameter
- Kalibrasi: 1 write per kalibrasi (aman untuk 100k cycles)
- Simpan: 1 write per simpan manual

## 9. Referensi & Sumber

1. Arduino Library - https://www.arduino.cc/en/Tutorial/Library
2. Tombol analog - https://www.arduino.cc/en/Tutorial/Button
3. Sensor suara - https://www.arduino.cc/en/Tutorial/Sound
4. Parsing serial - https://www.arduino.cc/en/Tutorial/Serial

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/ui.md` untuk keputusan desain