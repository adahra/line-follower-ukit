# BUILD.md — Compile & Upload dengan arduino-cli

Panduan lengkap compile dan upload LineFollowerFin ke **uKit Explore 2** menggunakan `arduino-cli` (tanpa Arduino IDE GUI).

---

## 📋 Prasyarat

### 1. Install arduino-cli
```bash
# Linux/macOS
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Atau download binary dari GitHub Releases
# https://github.com/arduino/arduino-cli/releases

# Verifikasi
arduino-cli version
```

### 2. Konfigurasi awal
```bash
# Buat config default
arduino-cli config init

# Tambahkan board manager URL untuk uKit (jika ada)
# Cek apakah uKitExplore2 tersedia di board manager resmi
arduino-cli board listall | grep -i ukit
```

---

## 🔧 Setup Board uKit Explore 2

### Opsi A: Jika uKitExplore2 ada di Board Manager
```bash
# Tambah URL board manager (ganti dengan URL resmi uKit jika ada)
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/ubtrobot/ukit-board-manager/main/package_ubtrobot_index.json

# Update index
arduino-cli core update-index

# Install core uKit
arduino-cli core install ubtrobot:avr
```

### Opsi B: Manual Install (Jika tidak di Board Manager)
```bash
# 1. Download library uKitExplore2 dari sumber resmi
# 2. Extract ke ~/Arduino/libraries/uKitExplore2/
# 3. Pastikan struktur:
#    ~/Arduino/libraries/uKitExplore2/
#    ├── uKitExplore2.h
#    ├── uKitMotor.h
#    ├── uKitServo.h
#    ├── uKitSensor.h
#    ├── uKitId.h
#    ├── onBoardHW.h
#    ├── TransforRobot.h
#    ├── VisionDiscrim.h
#    ├── ClickButton.h
#    └── src/ (file .cpp)

# 4. Install core AVR (untuk ATMega2560)
arduino-cli core install arduino:avr
```

---

## 📦 Install Library Dependencies

```bash
# Library yang dibutuhkan (sudah built-in di Arduino IDE):
# - EEPROM.h (built-in, tidak perlu install)

# Jika pakai library tambahan:
arduino-cli lib install "EEPROM"  # biasanya built-in
```

---

## ⚙️ Konfigurasi Project

### 1. Buat file `sketch.yaml` (opsional, untuk config project)
```yaml
# LineFollowerFin/sketch.yaml
board: arduino:avr:mega
programmer: arduino:avrispmkii  # atau usbasp, tergantung programmer
port: /dev/ttyUSB0  # atau /dev/ttyACM0, cek dengan `arduino-cli board list`
build_flags:
  - "-DARDUINO_UKIT_EXPLORE2"
  - "-Wall"
  - "-Wextra"
```

### 2. Atau gunakan flag langsung di command line

---

## 🏗️ Compile (Verify)

### Compile utama (LineFollowerFin.ino)
```bash
# Masuk ke folder project
cd LineFollowerFin

# Compile dengan board ATMega2560 (uKit Explore 2 berbasis Mega2560)
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin.ino

# Atau dengan konfigurasi spesifik:
arduino-cli compile \
  --fqbn arduino:avr:mega:cpu=atmega2560 \
  --build-property "compiler.cpp.extra_flags=-DARDUINO_UKIT_EXPLORE2" \
  LineFollowerFin.ino
```

### Compile test sketches
Test sketch berada di subfolder `tests/`; nama folder harus sama dengan nama `.ino`.

```bash
# Test Lap Timer
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestLapTimer

# Test Bluetooth
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestBluetooth

# Test Path Planner
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestPathPlanner
```

### Output compile sukses (default semua fitur opsional OFF):
```
Sketch uses 90448 bytes (35%) of program storage space. Maximum is 253952 bytes.
Global variables use 5813 bytes (70%) of dynamic memory, leaving 2379 bytes for local variables. Maximum is 8192 bytes.
```

---

## 📤 Upload ke Board

### 1. Cek port & board terdeteksi
```bash
# List board terhubung
arduino-cli board list

# Output contoh:
# PORT         TYPE              BOARD NAME          FQBN
# /dev/ttyUSB0 Serial Port       Arduino Mega or Mega 2560  arduino:avr:mega
```

### 2. Upload
```bash
# Upload ke port yang terdeteksi
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega LineFollowerFin.ino

# Atau auto-detect port (jika hanya 1 board)
arduino-cli upload --fqbn arduino:avr:mega LineFollowerFin.ino
```

### 3. Upload dengan programmer eksternal (USBasp, AVRISP mkII, dll)
```bash
# Jika pakai programmer (bukan bootloader USB)
arduino-cli upload -p /dev/ttyUSB0 \
  --fqbn arduino:avr:mega \
  --programmer arduino:avrispmkii \
  LineFollowerFin.ino
```

---

## 🔍 Monitor Serial

```bash
# Buka serial monitor (115200 baud)
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200

# Atau dengan format timestamp
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200 --timestamp

# Keluar: Ctrl+C
```

---

## 📋 One-Liner Lengkap (Compile + Upload + Monitor)

```bash
# Compile, upload, dan monitor berturut-turut
cd LineFollowerFin && \
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin.ino && \
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega LineFollowerFin.ino && \
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

---

## ⚙️ Konfigurasi Khusus uKit Explore 2

### Board Definition (jika butuh custom board)
Buat file `~/.arduino15/packages/ubtrobot/hardware/avr/1.0.0/boards.txt`:
```ini
ukit_explore2.name=uKit Explore 2
ukit_explore2.upload.tool=avrdude
ukit_explore2.upload.protocol=wiring
ukit_explore2.upload.maximum_size=253952
ukit_explore2.upload.maximum_data_size=8192
ukit_explore2.upload.speed=115200
ukit_explore2.bootloader.tool=avrdude
ukit_explore2.bootloader.low_fuses=0xFF
ukit_explore2.bootloader.high_fuses=0xD8
ukit_explore2.bootloader.extended_fuses=0xFD
ukit_explore2.bootloader.file=stk500v2/stk500boot_v2_mega2560.hex
ukit_explore2.build.mcu=atmega2560
ukit_explore2.build.f_cpu=16000000L
ukit_explore2.build.core=arduino
ukit_explore2.build.variant=mega
ukit_explore2.build.board=AVR_UKIT_EXPLORE2
```

Lalu compile dengan:
```bash
arduino-cli compile --fqbn ubtrobot:avr:ukit_explore2 LineFollowerFin.ino
```

---

## 🐛 Troubleshooting

### Error: "uKitExplore2.h: No such file or directory"
```bash
# Pastikan library terinstall di ~/Arduino/libraries/uKitExplore2/
ls ~/Arduino/libraries/uKitExplore2/

# Atau copy library ke folder sketch
cp -r ~/Arduino/libraries/uKitExplore2 LineFollowerFin/lib/
```

### Error: "Serial1 not declared" (untuk Bluetooth test)
```cpp
// Di TestBluetooth.ino, tambahkan di awal:
#define Serial1 Serial1  // Pastikan core AVR support Serial1
// Atau pakai HardwareSerial Serial1(1);
```

### Error: "avrdude: stk500v2_ReceiveMessage(): timeout"
```bash
# Coba:
# 1. Tekan tombol reset di board sebelum upload
# 2. Kurangi baud rate upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega --config upload.speed=57600 LineFollowerFin.ino

# 3. Cek kabel USB & driver CH340/CP2102
ls /dev/ttyUSB*
dmesg | grep -i usb
```

### Error: "Sketch too big" / "Text section exceeds"
```bash
# Optimasi size:
arduino-cli compile --fqbn arduino:avr:mega \
  --build-property "compiler.cpp.extra_flags=-Os -flto" \
  LineFollowerFin.ino

# Atau matikan fitur yg tidak dipakai di config.h:
# const bool ENABLE_PATH_PLANNER = false;
# const bool ENABLE_BLUETOOTH = false;
```

### Upload berhasil tapi tidak jalan
```bash
# 1. Cek Serial Monitor apakah ada output boot
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200

# 2. Pastikan Initialization() dipanggil di setup()
# 3. Cek apakah board benar-benar ATMega2560 (bukan 1280)
```

---

## 🤖 CI/CD (GitHub Actions) - Otomatis Compile

Workflow yang dipakai repo ada di [`.github/workflows/compile.yml`](.github/workflows/compile.yml). Inti langkahnya:

```yaml
- uses: actions/checkout@v4
- uses: arduino/setup-arduino-cli@v1
- run: arduino-cli core install arduino:avr
- name: Install uKit Explore library
  run: |
    mkdir -p "$HOME/Arduino/libraries"
    git clone --depth 1 https://github.com/UBTEDU/uKitExplore-library.git \
      "$HOME/Arduino/libraries/uKitExplore-library"
    git clone --depth 1 --branch v6.19.4 \
      https://github.com/bblanchon/ArduinoJson.git \
      "$HOME/Arduino/libraries/ArduinoJson"
- name: Compile main sketch
  run: |
    arduino-cli compile --fqbn arduino:avr:mega \
      --libraries "$HOME/Arduino/libraries" LineFollowerFin
- name: Compile test sketches
  run: |
    for test in TestLapTimer TestBluetooth TestPathPlanner; do
      arduino-cli compile --fqbn arduino:avr:mega \
        --libraries "$HOME/Arduino/libraries" "LineFollowerFin/tests/$test"
    done
```

Workflow juga mengompilasi salinan sketch dengan fitur opsional aktif
(`ENABLE_RC`, `ENABLE_HEADING_CORRECTION`, `ENABLE_ADAPTIVE_THRESHOLD`,
`ENABLE_GAIN_SCHEDULING`, `ENABLE_LAP_TIMER`) untuk menangkap error API yang
tidak terlihat pada build default.

Catatan: library uKit tidak di-vendor. Sumber resminya
[UBTEDU/uKitExplore-library](https://github.com/UBTEDU/uKitExplore-library).

---

## 📝 Quick Reference Card

| Perintah | Fungsi |
|----------|--------|
| `arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin.ino` | Compile utama |
| `arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega LineFollowerFin.ino` | Upload |
| `arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200` | Serial monitor |
| `arduino-cli board list` | List board terhubung |
| `arduino-cli lib list` | List library terinstall |
| `arduino-cli core list` | List core terinstall |

---

## 📁 Struktur Folder untuk arduino-cli

```
line-follower-ukit/
├── LineFollowerFin/
│   ├── LineFollowerFin.ino      # Sketch utama
│   ├── *.h / *.cpp              # Source code modular
│   ├── sketch.yaml              # (opsional) config project
│   ├── lib/                     # (opsional) library lokal
│   │   └── uKitExplore2/
│   └── tests/
│       ├── TestLapTimer/TestLapTimer.ino
│       ├── TestBluetooth/TestBluetooth.ino
│       └── TestPathPlanner/TestPathPlanner.ino
└── BUILD.md                     # File ini
```

---

## ✅ Checklist Sebelum Upload

- [ ] `arduino-cli compile` sukses tanpa error
- [ ] Board terdeteksi: `arduino-cli board list` → `/dev/ttyUSB0`
- [ ] Library `uKitExplore2` terinstall di `~/Arduino/libraries/`
- [ ] Port tidak dipakai program lain (close Arduino IDE/Serial Monitor)
- [ ] Board dalam mode bootloader (tekan reset jika perlu)
- [ ] Baud rate serial monitor = 115200 (sesuai code)

---

## 🔗 Referensi

- [arduino-cli Documentation](https://arduino.github.io/arduino-cli/latest/)
- [Board Manager JSON Format](https://github.com/arduino/arduino-cli/blob/master/BOARD_MANAGER.md)
- [uKit Explore 2 Datasheet](https://ubtechedu.gitbook.io/ukit-explore) - cek dokumentasi resmi
- [AVRDUDE Manual](https://www.nongnu.org/avrdude/user-manual/avrdude.html)

---

*File ini dibuat untuk memudahkan development headless/CI tanpa Arduino IDE GUI.*