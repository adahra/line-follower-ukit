# Modul RcCtrl - Dokumentasi Teknis

## 1. Ikhtisar

RcCtrl adalah modul kontrol Remote Control (RC) untuk LineFollowerFin. Ini membaca sinyal RC receiver (PWM/PPM pada digital pin) atau Serial (ESP-based), mengubahnya ke command motor, dan mengambil alih kontrol saat mode RC diaktifkan.

**File:** `LineFollowerFin/rc_ctrl.h/.cpp` (2.3KB / 6.0KB)
**Dependencies:** `Arduino.h`, `fwd.h`, `config.h`, `status.h`

## 2. Desain & Arsitektur

### 2.1 Tipe Sumber RC
```cpp
enum RCSource {
  RC_NONE = 0,
  RC_PWM,        // pulseIn pada digital pin
  RC_SERIAL      // Serial ESP-based receiver
};
```

### 2.2 Arsitektur Kontrol
- **Two channels:** Throttle (channel 1), Steering (channel 2)
- **Mapping:** PWM/PPM → kecepatan bertanda (-100..100)
- **Deadband:** Center zone (RC_DEADBAND_US = 50μs)
- **Timeout:** Hilang sinyal → hentikan robot
- **Prioritas:** Mode RC mengambil alih semua kontrol tracer saat diaktifkan

### 2.3 Integrasi Sistem
```cpp
RcCtrl rc(status, gerak, sensor, aman, lapTimer);
```

**Dependency injection:** Semua reference status dan komponen dibagikan via reference

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void(unsigned long baud = 115200, int pin = 2, RCSource src = RC_PWM, int steerPin = RC_STEER_PIN)` | Inisialisasi RC |
| `update()` | `void` | Dipanggil setiap loop untuk membaca RC |
| `readPulseWidth_us()` | `unsigned long readPulseWidth_us(int pin)` | Baca pulse width dari satu channel |
| `isEnabled()` | `bool isEnabled() const` | Cek apakah RC diaktifkan |
| `setEnabled()` | `void setEnabled(bool on)` | Aktifkan/deaktifkan RC control |
| `setMode()` | `void setMode(ModeOperasi mode)` | Set mode operasi ke MODE_RC |
| `getThrottle()` | `int getThrottle() const` | Kembalikan nilai throttle (-100..100) |
| `getSteering()` | `int getSteering() const` | Kembalikan nilai steering (-100..100) |
| `isSignalLost()` | `bool isSignalLost() const` | Cek apakah sinyal RC hilang |
| `reset()` | `void` | Reset state RC |
| `getSpeeds()` | `void getSpeeds(int &outSpeedKiri, int &outSpeedKanan) const` | Hitung speed dari throttle/steering |

### 3.2 State Private

| State | Type | Keterangan |
|-------|------|-------------|
| `status` | `Status &` | Reference status jalan |
| `gerak` | `Penggerak &` | Reference kontrol motor |
| `sensor` | `SensorGaris &` | Reference pembacaan sensor |
| `aman` | `Pengaman &` | Reference safety |
| `lapTimer` | `LapTimer &` | Reference lap timing |
| `rcSource` | `RCSource` | Tipe sumber RC |
| `rcPin` | `int` | Pin throttle |
| `rcSteerPin` | `int` | Pin steering |
| `rcBaud` | `unsigned long` | Baud rate untuk RC Serial |
| `lastPulseTime` | `unsigned long` | Waktu pulse terakhir |
| `lastThrottleUs` | `unsigned long` | Pulse throttle terakhir |
| `lastSteeringUs` | `unsigned long` | Pulse steering terakhir |
| `signalLost` | `bool` | Flag sinyal hilang |
| `throttlePct` | `int` | Throttle dalam persen (-100..100) |
| `steeringPct` | `int` | Steering dalam persen (-100..100) |
| `rcCmdBuffer` | `char rcCmdBuffer[RC_CMD_BUFFER_SIZE]` | Buffer command Serial |
| `rcCmdBufferPos` | `int` | Posisi buffer command |
| `lastSerialCheck` | `unsigned long` | Waktu check serial terakhir |
| `serialCheckInterval` | `unsigned long` | Throttle check serial |

## 4. Implementasi Detil

### 4.1 Method Inisialisasi
```cpp
void RcCtrl::begin(unsigned long baud, int pin, RCSource src, int steerPin) {
  rcSource = src;
  rcPin = pin;
  rcSteerPin = steerPin;
  rcBaud = baud;
  
  if (src == RC_PWM) {
    pinMode(pin, INPUT);
    pinMode(steerPin, INPUT);
  } else if (src == RC_SERIAL) {
    // Inisialisasi serial untuk ESP-based receiver
    Serial.begin(baud);
  }
}
```

### 4.2 Method Update
```cpp
void RcCtrl::update() {
  if (!isEnabled()) return;
  
  if (rcSource == RC_PWM) {
    readPwmChannel();
  } else if (rcSource == RC_SERIAL) {
    processSerialLine();
  }
  
  mapToSpeeds();
  
  // Periksa sinyal hilang
  if (millis() - lastPulseTime > RC_PULSE_TIMEOUT_MS) {
    signalLost = true;
  } else {
    signalLost = false;
  }
}
```

### 4.3 Method Read PWM
```cpp
unsigned long RcCtrl::readPulseWidth_us(int pin) {
  // Gunakan pulseIn untuk membaca pulse width
  unsigned long width = pulseIn(pin, HIGH, RC_PULSE_TIMEOUT_US);
  return width;
}
```

### 4.4 Method Map to Speeds
```cpp
void RcCtrl::mapToSpeeds() {
  // Mapping throttle: RC_MIN_US -> 0%, RC_CENTER_US -> 0%, RC_MAX_US -> 100%
  // Mapping steering: serupa, tapi steering left/right
  
  float throttle = mapValue(lastThrottleUs, RC_MIN_US, RC_MAX_US, RC_SCALE_MIN, RC_SCALE_MAX, RC_CENTER_US);
  float steering = mapValue(lastSteeringUs, RC_MIN_US, RC_MAX_US, -RC_SCALE_MAX, RC_SCALE_MAX, RC_CENTER_US);
  
  // Aplikasikan gain steering
  steering *= RC_STEER_GAIN;
  
  // Clamp ke -100..100
  throttlePct = constrain(throttle, -100, 100);
  steeringPct = constrain(steering, -100, 100);
}
```

### 4.5 Method Process Serial
```cpp
void RcCtrl::processSerialLine(const char *line) {
  if (strcmp(line, "ready") == 0) {
    // Siap untuk command
  } else if (strncmp(line, "set ", 4) == 0) {
    // Command set throttle/steering
    float throttle = atof(line + 4);
    float steering = atof(strchr(line, ' ') + 1);
    
    throttlePct = constrain(throttle, -100, 100);
    steeringPct = constrain(steering, -100, 100);
  }
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan Status
- Bekerja sama untuk `status.modeOperasi = MODE_RC`
- Bekerja sama untuk `status.jalan` (mulai/berhenti)
- Bekerja sama untuk `status.subState` (untuk debugging)

### 5.2 Integrasi dengan Penggerak
- Bekerja sama untuk kontrol motor: `gerak.jalankanArah(throttle, steering)`
- Bekerja sama untuk safety: `gerak.berhenti()` saat sinyal hilang
- Bekerja sama untuk emergency stop: `gerak.berhenti()` saat hilang sinyal

### 5.3 Integrasi dengan SensorGaris
- Bekerja sama untuk calibration: Tidak langsung (opsional)
- Bekerja sama untuk telemetry: Kirim reading sensor via RC

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk safety: `halangan()` menghentikan semua kontrol
- Bekerja sama untuk emergency: `berhenti()` saat sinyal hilang

### 5.5 Integrasi dengan Antarmuka
- Bekerja sama untuk command: `setEnabled()` dari command serial
- Bekerja sama untuk mode: `setMode()` dari command serial

## 6. Desain Decisions & Alasan

### 6.1 Two Channel RC
**Alasan:** Kontrol throttle dan steering terpisah. Implementasi lama membaca dua channel dari satu pin membuat steering selalu sama dengan throttle.

### 6.2 Deadband
**Alasan:** Center zone (RC_CENTER_US ± RC_DEADBAND_US) mencegah gerakan kecil saat RC tidak dipegang.

### 6.3 Timeout Sinyal
**Alasan:** Hilang sinyal selama > RC_PULSE_TIMEOUT_MS (500ms) dianggap hilang sinyal. Hentikan robot untuk safety.

### 6.4 Mapping PWM/PPM
**Alasan:** Standard RC receiver menggunakan PWM/PPM. Mapping ke -100..100 memudahkan control motor.

### 6.5 RC Steer Gain
**Alasan:** RC_STEER_GAIN = 1.0 memberikan steering linier. Gain bisa tune untuk respons yang lebih tinggi/rendah.

## 7. Error Handling & Edge Cases

### 7.1 Error RC
- **Tidak ada sinyal:** Timeout → sinyal hilang
- **Noise:** Deadband mencegah noise kecil
- **Invalid command:** Abaikan atau tampilkan error

### 7.2 Error Mapping
- **Pulse out of range:** Clamp ke batas yang valid
- **Gain terlalu tinggi:** Clamp steering
- **Timeout serial:** Reset buffer

### 7.3 Edge Cases
- **Startup:** Semua state direset
- **Kalibrasi:** Reset ke center (1500μs)
- **Multiple command:** Process satu per satu
- **Serial buffer overflow:** Circular buffer

## 8. Tuning Parameters

| Parameter | Default | Range | Pengaruh |
|-----------|---------|-------|------------|
| `RC_BAUD` | 115200 | 9600-115200 | Baud rate serial |
| `RC_PIN` | 2 | 0-5 | Pin throttle |
| `RC_STEER_PIN` | 3 | 0-5 | Pin steering |
| `RC_MIN_US` | 1000 | 800-1200 | Pulse minimum |
| `RC_MAX_US` | 2000 | 1800-2200 | Pulse maximum |
| `RC_CENTER_US` | 1500 | 1400-1600 | Pulse center |
| `RC_DEADBAND_US` | 50 | 25-100 | Deadband center |
| `RC_STEER_GAIN` | 1.0 | 0.5-2.0 | Gain steering |
| `RC_PULSE_TIMEOUT_MS` | 500 | 200-1000 | Timeout sinyal hilang |

## 9. Kinerja & Sumber Daya

### 9.1 Penggunaan RAM
- State: ~30 bytes (tipe data primitive)
- Buffer: 64 bytes (rcCmdBuffer)
- Total: ~100 bytes per instance

### 9.2 Konsumsi Waktu
- Update per loop: ~50-100 microseconds
- Read PWM: ~100-200 microseconds
- Mapping speed: ~10-20 microseconds
- Total per loop: 0.2-0.3 milliseconds

### 9.3 Penggunaan Flash
- Implementation: ~2KB
- Total: ~2.3KB

## 10. Referensi & Sumber

1. pulseIn - https://www.arduino.cc/en/Reference/pulseIn
2. Arduino Map - https://www.arduino.cc/en/Reference/map
3. PWM Control - https://www.arduino.cc/en/Tutorial/PWMSpeedControl
4. RC Receiver - https://www.robotmarketplace.com/products/pwm-rc-receiver-4-channel

## 11. Catatan Perkembangan

- v1.0: Implementasi dasar PWM/PPM RC
- v1.1: Tambahkan mapping throttle/steering
- v1.2: Tambahkan deadband & timeout
- v1.3: Tambahkan serial RC
- v1.4: Tambahkan error handling & validation
- v1.5: Perbaikan bug & tuning

## 12. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/rc_ctrl.md` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/rc.md` untuk keputusan desain

