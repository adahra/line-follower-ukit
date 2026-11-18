# API Dokumentasi - BluetoothCtrl

Dokumentasi API lengkap untuk modul BluetoothCtrl, sistem kontrol Bluetooth dan telemetri serial untuk monitoring jarak jauh dan debugging.

## 1. Ikhtisar

BluetoothCtrl (Bluetooth Control) adalah modul yang bertanggung jawab untuk Serial Bluetooth (HM-10/ESP32) atau built-in uKit BT, dump CSV telemetri (interval BT_TELEMETRY_MS), dan parsing command untuk kontrol robot.

**File:** `LineFollowerFin/bluetooth_ctrl.h/.cpp` (2.5KB / 11.8KB)
**Dependencies:** `config.h`, `status.h`, `sensor_garis.h`, `penggerak.h`, `pengaman.h`, `lap_timer.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void begin(long baud, BTModuleType moduleType)` | Inisialisasi Bluetooth |
| `update()` | `void update()` | Update Bluetooth setiap loop |
| `sendTelemetry()` | `void sendTelemetry()` | Kirim CSV telemetry |
| `setCommandCallback()` | `void setCommandCallback(void(*callback)(String))` | Set callback command Bluetooth |
| `isConnected()` | `bool isConnected()` | Kembalikan true jika Bluetooth terhubung |
| `getState()` | `BTState getState() const` | Kembalikan state Bluetooth |
| `getModuleType()` | `BTModuleType getModuleType() const` | Kembalikan tipe module Bluetooth |
| `reset()` | `void reset()` | Reset Bluetooth |

### 2.2 Method Telemetry

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setAutoTelemetry()` | `void setAutoTelemetry(bool enabled, unsigned long interval)` | Aktifkan/deaktifkan telemetry otomatis |
| `cetakTelemetry()` | `void cetakTelemetry()` | Tampilkan debug telemetry |
| `getTelemetryData()` | `String getTelemetryData() const` | Kembalikan data telemetry |

### 2.3 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `cetakStatus()` | `void cetakStatus()` | Tampilkan status Bluetooth |
| `validasiKoneksi()` | `bool validasiKoneksi() const` | Validasi koneksi Bluetooth |
| `resetChannel()` | `void resetChannel()` | Reset channel Bluetooth |

### 2. tiendaPrivasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `parsecmd()` | `bool parsecmd(String input)` | Parse command Bluetooth |
| `prosescmd()` | `void prosescmd(String cmd, String arg)` | Proses command Bluetooth |
| `tanganiSerial()` | `void tanganiSerial()` | Handled serial input |
| `tanganiKalibrasi()` | `void tanganiKalibrasi()` | Handled command kalibrasi |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class BTModuleType {
  BT_NONE = 0,
  BT_HM10,
  BT_UKIT_BUILTIN
};

enum class BTState {
  OFF = 0,
  ON,
  CONNECTED,
  ERROR
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `BT_BAUD` | long | 115200 | Baud rate Bluetooth |
| `BT_TELEMETRY_MS` | unsigned long | 1000 | Interval telemetri Bluetooth |
| `BT_COMMAND_TIMEOUT_MS` | unsigned long | 100 | Timeout command Bluetooth |
| `BT_MAX_CONNECTION_AGE_MS` | unsigned long | 30000 | Usia koneksi maksimum |
| `BT_RECONNECT_INTERVAL_MS` | unsigned long | 5000 | Interval reconnect Bluetooth |
| `TELEM_MAX_LINE_LENGTH` | int | 64 | Panjang maksimum baris telemetri |

## 4. Method Khusus

### 4.1 Method Inisialisasi
```cpp
void BluetoothCtrl::begin(long baud, BTModuleType moduleType) {
  // Set baud rate
  Serial1.begin(baud);
  
  // Set tipe module
  this->moduleType = moduleType;
  
  // Inisialisasi state
  state = BT_STATE_OFF;
  
  // Atur callback command
  setCommandCallback(parsecmd);
  
  // Tampilkan debug
  #if ENABLE_SERIAL_OUTPUT
  Serial.println(F("BluetoothCtrl: Inisialisasi tipe module = "));
  Serial.println(moduleType);
  #endif
}
```

### 4.2 Method Update
```cpp
void BluetoothCtrl::update() {
  // Periksa ketersediaan data
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    
    // Parse command
    parsecmd(input);
    
    // Update waktu activity
    lastActivity = millis();
  }
  
  // Periksa koneksi timeout
  if (state == BT_STATE_CONNECTED && 
      millis() - lastActivity > BT_MAX_CONNECTION_AGE_MS) {
    state = BT_STATE_OFF;
    Serial.println(F("Bluetooth: Koneksi timeout"));
  }
}
```

### 4.3 Method Telemetri
```cpp
void BluetoothCtrl::sendTelemetry() {
  if (!autoTelemetryEnabled) {
    return;
  }
  
  String csv = getTelemetryData();
  
  // Kirim telemetry
  Serial1.println(csv);
  
  // Tampilkan debug
  #if ENABLE_SERIAL_OUTPUT
  Serial.println(F("BluetoothTelemetry: "));
  Serial.println(csv);
  #endif
}
```

### 4.4 Method Parsing Command
```cpp
bool BluetoothCtrl::parsecmd(String input) {
  // Split command dan argumen
  int spasiIndex = input.indexOf(' ');
  String cmd = (spasiIndex > 0) ? input.substring(: spasiIndex) : input;
  String arg = (spasiIndex > 0) ? input.substring(spasiIndex + 1) : "";
  
  return prosescmd(cmd, arg);
}

bool BluetoothCtrl::prosescmd(String cmd, String arg) {
  if (cmd == "start") {
    status.jalan = true;
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
  
  if (cmd == "reset") {
    resetSemua();
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
  
  if (cmd == "save") {
    simpanSemuaKeEEPROM();
    return true;
  }
  
  if (cmd == "calibrate") {
    mulaiKalibrasiOtomatis();
    return true;
  }
  
  if (cmd == "help") {
    tampilkanHelp();
    return true;
  }
  
  return false;
}
```

### 4.5 Method Telemetry
```cpp
String BluetoothCtrl::getTelemetryData() const {
  String csv = "";
  
  // Header telemetry
  csv += F("Time(ms),Kp,Ki,Kd,SpeedLeft,SpeedRight,Error,Heading");
  csv += ',';
  
  // Data telemetry
  csv += millis();
  csv += ',';
  csv += status.Kp;
  csv += ',';
  csv += status.Ki;
  csv += ',';
  csv += status.Kd;
  csv += ',';
  
  // Get current speeds
  int speedKiri, speedKanan;
  gerak.getSpeeds(speedKiri, speedKanan);
  
  csv += speedKiri;
  csv += ',';
  csv += speedKanan;
  csv += ',';
  
  // Get current error
  int error = sensor.bacaKiri() - sensor.bacaKanan();
  csv += error;
  csv += ',';
  
  // Get current heading (jika diaktifkan)
  #if ENABLE_HEADING_CORRECTION
  csv += gerak.getHeading();
  #else
  csv += 0;
  #endif
  
  return csv;
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Antarmuka
- Sync command Bluetooth dengan command serial
- Menerima command kontrol robot melalui Bluetooth
- Sinkronisasi state robot antar Bluetooth dan serial

### 5.2 Integrasi dengan Pengaman
- Menerima command keselamatan melalui Bluetooth
- Sinkronisasi state safety antar Bluetooth dan serial

### 5.3 Integrasi dengan Penggerak
- Kirim telemetry kecepatan motor melalui Bluetooth
- Menerima command kontrol motor melalui Bluetooth
- Sinkronisasi kontrol motor antar Bluetooth dan serial

### 5.4 Integrasi dengan SensorGaris
- Kirim data sensor melalui Bluetooth telemetry
- Menerima command kalibrasi sensor melalui Bluetooth

### 5.5 Integrasi dengan LapTimer
- Kirim data timing lap melalui Bluetooth telemetry
- Menerima command kontrol lap timer melalui Bluetooth

## 6. Desain Decisions & Alasan

### 6.1 Bluetooth Serial
**Alasan:** Menggunakan Serial1 untuk komunikasi Bluetooth. Lebih mudah dari library Bluetooth eksternal.

### 6.2 Telemetry CSV
**Alasan:** Format CSV mudah di-parse dan kompatibel dengan spreadsheet. Mendukung telemetry real-time.

### 6.3 Parsing Command
**Alasan:** Parsing command serial fleksibel dengan syntax sederhana. Mendukung command tunggal dan command dengan argumen.

### 6.4 Connection Handling
**Alasan:** Timeout koneksi dan reconnect memastikan robot tetap responsif. Mencegah hanging connection.

## 7. Error Handling & Edge Cases

### 7..eachErrorParsing
- **Command tidak valid:** Log error, kembalikan false
- **Argumen hilang:** Tampilkan usage command
- **Format argumen salah:** Tampilkan error, reset command

### 7.2 Error Bluetooth
- **Koneksi timeout:** Set state OFF
- **Error parsing:** Reset koneksi
- **CRC error:** Kembalikan false

### 7.3 Edge Cases
- **Command duplik:** Abaikan duplik
- **Timeout tel
- **Checksum tidak valid:** Reset koneksi

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Buffer command: 128 bytes
- Buffer telemetry: 64 bytes
- State Bluetooth: 20 bytes
- Total: ~180 bytes per instance

### 8.2 Konsumsi Waktu
- Update Bluetooth per loop: ~10-20 microseconds
- Parsing command: 50-100 microseconds saat command diterima
- Telemetri: 50-100 microseconds saat interval
- Inisialisasi: 100-200 microseconds

### 8.3 EEPROM Write Cycle
- Tuning: 1 write per perubahan parameter
- Kalibrasi: 序列化 Command: 1 write per kalibrasi (aman untuk 100k cycles)
- Simpan: 1 write per simpan

## 9. Referensi & Sumber

1. Bluetooth Arduino - https://www.arduino.cc/en/Tutorial/Bluetooth
2. Serial Communication - https://www.arduino.cc/en/Tutorial/Serial
3. EEPROM - https://www.arduino.cc/en/Tutorial/EEPROM
4. CSV format - https://www.arduino.cc/en/Tutorial/CSV

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/bluetooth.md` untuk keputusan desain