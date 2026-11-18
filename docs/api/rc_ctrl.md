# API Dokumentasi - RcCtrl

Dokumentasi API lengkap untuk modul RcCtrl, sistem kontrol Remote Control (RC) untuk robot, mendukung PWM/PPM (dua channel) atau Serial ESP-based.

## 1. Ikhtisar

RcCtrl (Remote Control) adalah modul yang bertanggung jawab untuk PWM/PPM (dua channel) atau Serial ESP-based, pengambil alih penuh kontrol robot saat diaktifkan. Ini merupakan komponen kontrol eksternal yang penting yang memungkinkan operator mengendalikan robot secara manual.

**File:** `LineFollowerFin/rc_ctrl.h/.cpp` (2.3KB / 6.0KB)
**Dependencies:** `config.h`, `status.h`, `penggerak.h`, `sensor_garis.h`, `pengaman.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void begin(long baud, int pin, int pwm, int steerPin)` | Inisialisasi RC |
| `update()` | `void update()` | Update RC setiap loop |
| `getSpeeds()` | `void getSpeeds(int& speedKiri, int& speedKanan)` | Ambil kecepatan motor kiri/kanan |
| `isSignalLost()` | `bool isSignalLost()` | Kembalikan true jika sinyal RC hilang |
| `setMode()` | `void setMode(RCSource mode)` | Set mode RC (PWM/PPM atau Serial) |
| `reset()` | `void reset()` | Reset RC |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `setChannel()` | `void setChannel(int channel)` | Set channel RC (PWM/PPM) |
| `setCalibration()` | `void setCalibration(int offset, int scale)` | Set offset dan scale RC |
| `resetChannel()` | `void resetChannel()` | Reset channel RC |
| `cetakStatus()` | `void cetakStatus()` | Tampilkan status RC |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `parsecmd()` | `bool parsecmd(String input)` | Parse command RC |
| `prosescmd()` | `void prosescmd(String cmd, String arg)` | Proses command RC |
| `cekPWM()` | `bool cekPWM()` | Periksa PWM RC |
| `cekPPM()` | `bool cekPPM()` | Periksa PPM RC |
| `cekSerial()` | `bool cekSerial()` | Periksa Serial RC |
| `tanganiSignal()` | `void tanganiSignal()` | Handled signal RC |
| `cekSignalLost()` | `bool cekSignalLost()` | Cek sinyal hilang |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class RCSource {
  RC_NONE = 0,
  RC_PWM,
  RC_SERIAL
};

enum class RCState {
  OFF = 0,
  ON,
  CONNECTED,
  ERROR
};
```

### 3.2 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `RC_BAUD` | long | 115200 | Baud rate RC |
| `RC_PIN` | int | 2 | Pin RC (PWM/PPM) |
| `RC_STEER_PIN` | int | 3 | Pin steer RC |
| `RC_PWM_MAX` | int | 255 | PWM maksimum |
| `RC_PPM_MIN` | int | 1000 | PPM minimum (mikrodetik) |
| `RC_PPM_MAX` | int | 2000 | PPM maksimum (mikrodetik) |
| `RC_SERIAL_TIMEOUT_MS` | unsigned long | 100 | Timeout serial RC |
| `RC_SIGNAL_LOST_TIMEOUT_MS` | unsigned long | 500 | Timeout sinyal hilang |
| `RC_CALIBRATION_TIMEOUT_MS` | unsigned long | 1000 | Timeout kalibrasi RC |

## 4. Method Khusus

### 4.1 Method Inisialisasi
```cpp
void RcCtrl::begin(long baud, int pin, int pwm, int steerPin) {
  // Set pin mode
  pinMode(pin, INPUT);
  pinMode(pwm, OUTPUT);
  pinMode(steerPin, OUTPUT);
  
  // Set baud rate
  Serial2.begin(baud);
  
  // Inisialisasi state
  state = RC_STATE_OFF;
  source = RC_SOURCE_NONE;
  
  // Set pin
  rcPin = pin;
  pwmPin = pwm;
  steerPin = steerPin;
  
  // Tampilkan debug
  #if ENABLE_SERIAL_OUTPUT
  Serial.println(F("RcCtrl: Inisialisasi pin = "));
  Serial.println(pin);
  Serial.println(F("Steer pin = "));
  Serial.println(steerPin);
  #endif
}
```

### 4.2 Method Update
```cpp
void RcCtrl::update() {
  // Periksa signal RC
  if (source == RC_SOURCE_PWM) {
    cekPWM();
  } else if (source == RC_SOURCE_PPM) {
    cekPPM();
  } else if (source == RC_SOURCE_SERIAL) {
    cekSerial();
  }
  
  // Cek sinyal hilang
  if (isSignalLost()) {
    gerak.berhenti();
    setRgbledColor(255, 0, 0);  // Merah = sinyal hilang
  }
  
  // Update sinyal
  tanganiSignal();
}
```

### 4.3 Method PWM
```cpp
void RcCtrl::cekPWM() {
  // Baca PWM dari pin
  int pwmKiri = pulseIn(rcPin, HIGH, RC_PWM_MAX * 2);
  int pwmKanan = pulseIn(steerPin, HIGH, RC_PWM_MAX * 2);
  
  // Kalibrasi PWM
  int speedKiri = kalibrasiPWM(pwmKiri);
  int speedKanan = kalibrasiPWM(pwmKanan);
  
  // Terapkan ke motor
  gerak.jalankanArah(speedKiri, speedKanan);
}
```

### 4.4 Method PPM
```cpp
void RcCtrl::cekPPM() {
  // Baca PPM dari pin
  int ppmKiri = bacaPPM(rcPin);
  int ppmKanan = bacaPPM(steerPin);
  
  // Kalibrasi PPM
  int speedKiri = kalibrasiPPM(ppmKiri);
  int speedKanan = kalibrasiPPM(ppmKanan);
  
  // Terapkan ke motor
  gerak.jalankanArah(speedKiri, speedKanan);
}
```

### 4.5 Method Serial
```cpp
void RcCtrl::cekSerial() {
  if (Serial2.available()) {
    String input = Serial2.readStringUntil('\n');
    input.trim();
    
    // Parse command
    parsecmd(input);
    
    // Update waktu activity
    lastActivity = millis();
  }
  
  // Periksa koneksi timeout
  if (state == RC_STATE_CONNECTED && 
      millis() - lastActivity > RC_SERIAL_TIMEOUT_MS) {
    state = RC_STATE_OFF;
    Serial.println(F("RC: Koneksi serial timeout"));
  }
}
```

### 4.6 Method Kalibrasi
```cpp
calibrasiRc() {
  // Kalibrasi PWM/PPM
  int pwmKiri = bacaPWM(rcPin);
  int ppmKiri = bacaPPM(rcPin);
  int speedKiri = (pwmKiri + ppmKiri) / 2;
  
  int pwmKanan = bacaPWM(steerPin);
  int ppmKanan = bacaPPM(steerPin);
  int speedKanan = (pwmKanan + ppmKanan) / 2;
  
  // Set offset dan scale
  offsetKiri = speedKiri;
  scaleKiri = 255 / (pwmKiriMax - pwmKiriMin);
  
  offsetKanan = speedKanan;
  scaleKanan = 255 / (ppmKananMax - ppmKananMin);
  
  // Tampilkan debug
  #if ENABLE_SERIAL_OUTPUT
  Serial.println(F("RcCtrl: Kalibrasi selesai"));
  Serial.print(F("OffsetKiri = "));
  Serial.println(offsetKiri);
  Serial.print(F("ScaleKiri = "));
  Serial.println(scaleKiri);
  #endif
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Antarmuka
- Sinkronisasi command RC dengan command serial
- Menerima command kontrol robot melalui RC
- Sinkronisasi state robot antar RC dan serial

### 5.2 Integrasi dengan Pengaman
- Menerima command keselamatan melalui RC
- Sinkronisasi state safety antar RC dan serial
- Menerima command emergency stop melalui RC

### 5.3 Integrasi dengan Penggerak
- Kirim telemetry kecepatan motor melalui RC
- Menerima command kontrol motor melalui RC
- Sinkronisasi kontrol motor antar RC dan serial

### 5.4 Integrasi dengan SensorGaris
- Kirim data sensor melalui RC telemetry
- Menerima command kalibrasi sensor melalui RC

### 5.5 Integrasi dengan LapTimer
- Kirim data timing lap melalui RC telemetry
- Menerima command kontrol lap timer melalui RC

## 6. Desain Decisions & Alasan

### 6.1 PWM/PPM vs Serial
**Alasan:** PWM/PPM lebih real-time dan presisi dibandingkan serial. Serial lebih mudah untuk setup dan debugging.

### 6.2 Pengambil alih Penuh
**Alasan:** RC mengambil alih sepenuhnya saat diaktifkan (status.modeOperasi == MODE_RC). Ini mencegah konflik dengan kontrol tracer otomatis.

### 6.3 Kalibrasi RC
**Alasan:** Kalibrasi PWM/PPM memastikan mapping yang tepat dari joystick ke motor. Offset dan scale memastikan kontrol yang presisi.

### 6.4 Timeout Sinyal Hilang
**Alasan:** Timeout sinyal hilang mencegah robot bergerak tanpa kontrol. Merah LED menunjukkan kehilangan sinyal.

## 7. Error Handling & Edge Cases

### 7.1 Error Parsing Signal
- **Signal tidak valid:** Log error, kembalikan false
- **Timeout signal:** Reset RC
- **Checksum tidak valid:** Kembalikan false

### 7.2 Error Kalibrasi
- **Offset tidak valid:** Gunakan default offset
- **Scale tidak valid:** Gunakan default scale
- **Kalibrasi timeout:** Gagal kalibrasi

### 7.3 Edge Cases
- **Signal noise:** Filter noise dengan average beberapa pembacaan
- **Signal interference:** Gunakan channel yang berbeda
- **Kalibrasi overlap:** Pastikan tidak ada kalibrasi berturut-turut
- **Reset RC:** Hentikan motor, reset state

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Buffer command: 128 bytes
- Buffer signal: 64 bytes
- State RC: 20 bytes
- Total: ~180 bytes per instance

### 8.2 Konsumsi Waktu
- Update RC per loop: ~10-20 microseconds
- Parsing command: 50-100 microseconds saat command diterima
- Kalibrasi: 100-200 microseconds saat kalibrasi
- Inisialisasi: 50-100 microseconds

### 8.3 EEPROM Write Cycle
- Tuning: 1 write per perubahan parameter
- Kalibrasi: 1 write per kalibrasi (aman untuk 100k cycles)
- Simpan: 1 write per simpan

## 9. Referensi & Sumber

1. PWM Arduino - https://www.arduino.cc/en/Tutorial/PWM
2. PPM Arduino - https://www.arduino.cc/en/Tutorial/PPM
3. Serial Communication - https://www.arduino.cc/en/Tutorial/Serial
4. RC Controller - https://www.arduino.cc/en/Tutorial/RC

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/rc.md` untuk keputusan desain