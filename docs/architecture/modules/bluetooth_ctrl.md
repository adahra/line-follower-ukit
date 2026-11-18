# Modul BluetoothCtrl - Dokumentasi Teknis

## 1. Ikhtisar

BluetoothCtrl adalah modul kontrol Bluetooth/BLE untuk LineFollowerFin. Ini mendukung Serial Bluetooth (HM-10/ESP32) atau built-in uKit BT, menyediakan telemetry dan serial command.

**File:** `LineFollowerFin/bluetooth_ctrl.h/.cpp` (2.5KB / 11.8KB)
**Dependencies:** `Arduino.h`, `fwd.h`, `config.h`, `status.h`

## 2. Desain & Arsitektur

### 2.1 Tipe Modul Bluetooth
```cpp
enum BTModuleType {
  BT_NONE = 0,
  BT_HM10,        // HM-10/HC-08 BLE module via Serial
  BT_UKIT_BUILTIN // uKit built-in Bluetooth (jika tersedia)
};
```

### 2.2 Arsitektur Komunikasi
- **Serial:** HardwareSerial untuk HM-10 (Serial1 atau Serial2)
- **BLE:** Built-in uKit Bluetooth (jika tersedia)
- **Protocol:** Text-based serial (ASCII) untuk command dan telemetry
- **Buffer:** Circular buffer untuk command processing
- **Throttling:** Throttle untuk pembacaan & pengiriman data

### 2.3 Integrasi Sistem
```cpp
BluetoothCtrl bluetooth(status, gerak, sensor, aman, lapTimer);
```

**Dependency injection:** Semua reference status dan komponen dibagikan via reference

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void(unsigned long baud = 115200, BTModuleType type = BT_HM10)` | Inisialisasi Bluetooth |
| `update()` | `void` | Dipanggil setiap loop untuk membaca command & mengirim telemetry |
| `sendTelemetry()` | `void(const BTTelemetry &tel)` | Kirim struct telemetry |
| `sendStatus()` | `void` | Kirim status ringkas |
| `sendLapSummary()` | `void` | Kirim data lap summary |
| `sendEvent()` | `void(const char *event, const char *detail = "")` | Broadcast event |
| `setCommandCallback()` | `void setCommandCallback(void (*callback)(const char *cmd))` | Set callback untuk command processing |
| `isConnected()` | `bool isConnected() const` | Cek apakah Bluetooth terhubung |
| `setAutoTelemetry()` | `void setAutoTelemetry(bool enable, unsigned long intervalMs = 200)` | Aktifkan/telemetry otomatis |

### 3.2 Struktur Data

```cpp
struct BTTelemetry {
  unsigned long timestamp;
  float error;
  float pid;
  int speedDasar;
  int speedKiri;
  int speedKanan;
  int sensorKiri;
  int sensorKanan;
  float battery;
  int mode;
  bool jalan;
};
```

### 3.3 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `status` | `Status &` | Reference status jalan |
| `gerak` | `Penggerak &` | Reference kontrol motor |
| `sensor` | `SensorGaris &` | Reference pembacaan sensor |
| `aman` | `Pengaman &` | Reference safety |
| `lapTimer` | `LapTimer &` | Reference lap timing |
| `moduleType` | `BTModuleType` | Tipe modul Bluetooth |
| `connected` | `bool` | Status koneksi |
| `autoTelemetry` | `bool` | Flag telemetry otomatis |
| `telemetryInterval` | `unsigned long` | Interval telemetry otomatis |
| `lastTelemetry` | `unsigned long` | Waktu telemetry terakhir |
| `lastCommandCheck` | `unsigned long` | Waktu command terakhir |
| `commandCheckInterval` | `unsigned long` | Throttle command |
| `cmdBuffer` | `char cmdBuffer[CMD_BUFFER_SIZE]` | Buffer command |
| `cmdBufferPos` | `int` | Posisi buffer command |
| `commandCallback` | `void (*)(const char *cmd)` | Callback command |
| `btSerial` | `HardwareSerial *` | Pointer HardwareSerial untuk BT |

## 4. Implementasi Detil

### 4.1 Method Inisialisasi
```cpp
void BluetoothCtrl::begin(unsigned long baud, BTModuleType type) {
  moduleType = type;
  connected = false;
  autoTelemetry = false;
  telemetryInterval = 0;
  
  // Pilih HardwareSerial based on module type
  if (type == BT_HM10) {
    btSerial = &Serial1;  // Default: Serial1
  } else if (type == BT_UKIT_BUILTIN) {
    btSerial = &Serial2;  // uKit built-in BT
  }
  
  btSerial->begin(baud);
}
```

### 4.(如 Method Update
```cpp
void BluetoothCtrl::update() {
  unsigned long sekarang = millis();
  
  // Cek command dari Bluetooth
  if (connected && sekarang - lastCommandCheck >= commandCheckInterval) {
    char buffer[CMD_BUFFER_SIZE];
    if (readBTLine(buffer, sizeof(buffer))) {
      processCommand(buffer);
    }
    lastCommandCheck = sekarang;
  }
  
  // Kirim telemetry otomatis
  if (autoTelemetry && sekarang - lastTelemetry >= telemetryInterval) {
    BTTelemetry tel;
    tel.timestamp = sekarang;
    tel.error = gerak.errorTerakhir();
    tel.pid = gerak.PID_value;
    tel.speedDasar = gerak.kecepatanDasar;
    tel.speedKiri = gerak.kecepatanKiri;
    tel.speedKanan = gerak.kecepatanKanan;
    tel.sensorKiri = sensor.bacaKiri();
    tel.sensorKanan = sensor.bacaKanan();
    tel.battery = aman.tegangan();
    tel.mode = status.modeOperasi;
    tel.jalan = status.jalan;
    
    sendTelemetry(tel);
    lastTelemetry = sekarang;
  }
}
```

### 4.3 Method Command Processing
```cpp
void BluetoothCtrl::processCommand(const char *cmd) {
  if (strcmp(cmd, "help") == 0) {
    sendResponse("Available commands:\n");
    sendResponse("  help\n");
    sendResponse("  reset\n");
    sendResponse("  calibrate\n");
    sendResponse("  save\n");
  } else if (strcmp(cmd, "reset") == 0) {
    // Reset semua modul
    gerak.reset();
    sensor.resetFilter();
    mapper.reset();
    lapTimer.reset();
    sendOK("Reset complete");
  } else if (strcmp(cmd, "calibrate") == 0) {
    // Kalibrasi otomatis sensor
    sensor.kalibrasi(gerak, false);
    sendOK("Calibration complete");
  } else if (strcmp(cmd, "save") == 0) {
    // Simpan semua ke EEPROM
    sensor.simpan();
    gerak.simpanHeading();
    mapper.simpan();
    sendOK("Save complete");
  } else {
    sendError("Unknown command");
  }
}
```

### 4.4 Method Telemetry
```cpp
void BluetoothCtrl::sendTelemetry(const BTTelemetry &tel) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), 
           "T:%lu,E:%.2f,P:%.2f,S:%d,SL:%d,SR:%d,C:%.2f,M:%d,J:%d\n",
           tel.timestamp, tel.error, tel.pid, tel.speedDasar,
           tel.speedKiri, tel.speedKanan, tel.battery,
           tel.mode, tel.jalan ? 1 : 0);
  
  btSerial->print(buffer);
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Status
- Bekerja sama untuk status jalan: `status.jalan`
- Bekerja sama untuk mode operasi: `status.modeOperasi`
- Bekerja sama untuk callback command: `setCommandCallback`

### 5.2 Integrasi dengan Penggerak
- Bekerja sama untuk error PID: `gerak.errorTerakhir()`
- Bekerja sama untuk gains PID: `gerak.PID_value`
- Bekerja sama untuk kecepatan motor: `gerak.kecepatanKiri`, `gerak.kecepatanKanan`
- Bekerja sama untuk penyimpanan: `gerak.simpanHeading()`

### 5.3 Integrasi dengan SensorGaris
- Bekerja sama untuk reading sensor: `sensor.bacaKiri()`, `sensor.bacaKanan()`
- Bekerja sama untuk kalibrasi: `sensor.kalibrasi()`

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk tegangan baterai: `aman.tegangan()`
- Bekerja sama untuk status safety: `aman.bateraiKritis()`, `aman.miring()`

### 5.5 Integrasi dengan LapTimer
- Bekerja sama untuk data lap: `lapTimer.getLapCount()`, `lapTimer.getLastLapTime()`
- Bekerja sama untuk telemetry: `sendLapSummary()`

### 5.6 Integrasi dengan Antarmuka
- Bekerja sama untuk command parsing: `parseAndExecute()`
- Bekerja sama untuk event broadcasting: `sendEvent()`

## 6. Desain Decisions & Alasan

### 6.1 Serial Bluetooth
**Alasan:** Serial Bluetooth (HM-10) mudah diintegrasikan dengan Arduino. String-based command mudah di-parse.

### 6.2 Auto Telemetry
**Alasan:** Telemetry otomatis untuk debugging & monitoring. Throttle mencegah pembacaan sensor berlebihan.

### 6.3 Circular Buffer
**Alasan:** Hindari overflow buffer saat command diterima berturut-turut. Circular buffer hemat memori.

### 6.4 Telemetry Interval
**Alasan:** Throttle telemetry (BT_TELEMETRY_MS = 200ms) mencegah pengiriman data berlebihan.

### 6.5 Callback Command
**Alasan:** Memisahkan command processing dari Bluetooth update. Lebih fleksibel untuk callback.

## 7. Error Handling & Edge Cases

### 7.1 Error Koneksi
- **Tidak terhubung:** Tetap terima command
- **Koneksi terputus:** Coba reconnect
- **Error serial:** Reset buffer

### 7.2 Error Parsing
- **String kosong:** Abaikan
- **Format salah:** Tampilkan error
- **Value out of range:** Clamp ke batas yang valid

### 7.3 Edge Cases
- **Startup:** Semua state direset
- **Kalibrasi:** Reset semua modul
- **Power cycle:** Muat semua state
- **Full buffer:** Hapus oldest command

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `BT_BAUD` | 115200 | 9600-115200 | Baud rate Bluetooth |
| `BT_TELEMETRY_MS` | 200 | 100-500 | Interval telemetry otomatis |
| `BT_MODULE_TYPE` | 1 | 0-2 | Tipe modul Bluetooth |
| `commandCheckInterval` | 50 | 10-100 | Throttle command |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- State: ~20 bytes (tipe data primitive)
- Buffer: 128 bytes (cmd buffer)
- Total: ~150 bytes per instance

### 9.2 Konsumsi Waktu
- Update per loop: ~50-100 microseconds
- Send telemetry: ~100-200 microseconds
- Parse command: ~50-150 microseconds
- Total per loop: 0.2-0.5 milliseconds

### 9.3 Penggunaan Flash
- Implementation: ~5KB
- Total: ~5.5KB

## 10. Referensi & Sumber

1. Serial Communication - https://www.arduino.cc/en/Reference/Serial
2. String Functions - https://www.arduino.cc/en/Reference/String
3. snprintf - https://www.cplusplus.com/reference/cstdio/snprintf/
4. Callback - https://www.geeksforgeeks.org/function-pointers-c/

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar Serial Bluetooth
- v1.1: Tambahkan auto telemetry
- v1.2: Tambahkan command processing & callback
- v1.3: Tambahkan error handling & validation
- v1.4: Perbaikan bug & tuning

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/bluetooth_ctrl.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/bluetooth.md` untuk keputusan desain

