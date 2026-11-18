# Keputusan Desain - PID Controller

Dokumentasi keputusan desain dan alasan untuk PID controller, komponen kontrol utama yang mengubah error pelacakan garis menjadi command motor.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk PID controller di LineFollowerFin, yang menerapkan kontrol PID anti windup, deadband servo, dan fitur kontrol motor canggih.

**File:** `docs/design-decisions/pid.md`

## 2. Tujuan PID Controller

### 2.1 Kontrol Error
**Alasan:** PID controller mengubah error pelacakan garis (selisih sensor kiri/kanan) menjadi command motor. Ini memberikan kontrol yang presisi untuk robot line-following.

**Implementasi:**
- Error = (bacaKiri - bacaKanan) / 2.0
- Error positif = robot condong ke kiri, perlu belok kanan
- Error negatif = robot condong ke kanan, perlu belok kiri

### 2.2 Anti Windup
**Alasan:** Tanpa anti windup, I term bisa melambung saat actuator saturasi (mis. motor berhenti). Clamping ±50 mencegah integral melambung.

**Implementasi:**
- I term dijepit ±BATAS_INTEGRAL (50)
- Reset integral setelah recovery exit
- I term hanya diupdate saat PID interval selesai

### 2.3 Deadband Servo
**Alasan:** Hardware servo punya deadband: command speed kecil hanya berdengung, roda tidak bergerak. 25 adalah minimum efektif.

**Implementasi:**
- Speed 1-24 dipaksa ke 25 atau 0 (SERVO_MIN_EFEKTIF = 25)
- Mencegah joystick drift saat speed rendah

### 2.4 PID Interval Tetap
**Alasan:** Delay antar loop membuat D melonjak acak. Interval tetap (10ms) memastikan dt konstan.

**Implementasi:**
- PID_INTERVAL_MS = 10ms (pastikan dt konstan)
- Gunakan millis() untuk interval yang akurat
- Reset PID timer saat forced compute

## 3. Pemilihan Parameter PID

### 3.1 Kp (Proportional Gain)
**Alasan:** Kp memberikan responsivitas error. Kp tinggi membuat robot lebih responsif tapi bisa menyebabkan overshoot.

**Implementasi:**
- Default: Kp = 1.2
- Range: 0.5-3
- Saat speed rendah: Kp tinggi (responsif)
- Saat speed tinggi: Kp rendah (anti overshoot)

### 3.2 Ki (Integral Gain)
**Alasan:** Ki menghilangkan steady-state error. Ki tinggi membuat robot lebih stabil tapi bisa menyebabkan integral windup.

**Implementasi:**
- Default: Ki = 1.0
- Range: 0-5
- Saat speed rendah: Ki menengah
- Saat speed tinggi: Ki rendah

### 3.3 Kd (Derivative Gain)
**Alasan:** Kd memberikan damping untuk mencegah overshoot. Kd tinggi membuat robot lebih stabil tapi bisa menyebabkan noise amplification.

**Implementasi:**
- Default: Kd = 0.8
- Range: 0-2
- Saat speed rendah: Kd rendah (tidak butuh damping keras)
- Saat speed tinggi: Kd tinggi (stabil)

## 4. Desain Decision Key

### 4.1 Anti Windup Implementation
**Alasan:** Implementasi anti windup yang tepat mencegah integral melambung saat actuator saturasi.

**Implementasi:**
```cpp
float I = I + error * Ki;
I = constrain(I, -BATAS_INTEGRAL, BATAS_INTEGRAL);  // anti windup
```

### 4.2 Reset Integral Setelah Recovery
**Alasan:** Setelah recovery dari lost-line, integral hutang tidak relevan dengan garis yang baru ditemukan.

**Implementasi:**
```cpp
bool dariRecovery = (status.recoveryMulai != 0);
status.recoveryMulai = 0;
if (dariRecovery) {
  gerak.resetIntegralSetelahRecovery((bacaKiri - bacaKanan) / 2.0);
}
```

### 4.3 Gain Scheduling
**Alasan:** Karakteristik respon PID berbeda saat speed rendah vs tinggi. Penjadwalan speed memberikan gain yang optimal.

**Implementasi:**
```cpp
if (kecepatanDasar <= GS_SPEED_LOW_MAX) {
  Kp = GS_KP_LOW; Ki = GS_KI_LOW; Kd = GS_KD_LOW;
} else if (kecepatanDasar <= GS_SPEED_MID_MAX) {
  Kp = GS_KP_MID; Ki = GS_KI_MID; Kd = GS_KD_MID;
} else {
  Kp = GS_KP_HIGH; Ki = GS_KI_HIGH; Kd = GS_KD_HIGH;
}
```

### 4.4 Heading Correction Sebagai Lapisan Atas
**Alasan:** Heading correction berbasis gyro Z komplementer dengan PID berbasis error sensor. Ini adalah koreksi steer tambahan, bukan replacement.

**Implementasi:**
```cpp
float headingCorrection = getHeadingError() * HEADING_GAIN;
headingCorrection = constrain(headingCorrection, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
PID_value += headingCorrection;
```

## 5. Desain Trade-offs

### 5.1 PID Interval Tetap vs Variable
**Trade-off:** Interval tetap (10ms) memastikan dt konstan tapi mengurangi fleksibilitas. Variable interval lebih fleksibel tapi menyebabkan D yang tidak stabil. Memilih tetap untuk stability.

### 5.2 Anti Windup vs Tanpa Anti Windup
**Trade-off:** Anti windup mencegah integral melambung tapi menambah kompleksitas komputasi. Tanpa anti windup lebih sederhana tapi rentan terhadap integral windup. Memilih anti windup untuk robustness.

### 5.3 Deadband vs Tanpa Deadband
**Trade-off:** Deadband mencegah joystick drift tapi bisa menyebabkan jitter. Tanpa deadband lebih presisi tapi bisa menyebabkan jitter. Memilih deadband untuk kontrol yang lebih halus.

## 6. Validasi Desain

### 6.1 Tuning Parameter
**Validasi:** Parameter PID diuji dengan robot pada kecepatan yang berbeda.

**Implementasi:**
- Speed rendah (≤60): Kp tinggi, Kd rendah
- Speed tinggi (>100): Kp rendah, Kd tinggi
- Speed menengah: gain menengah

### 6.2 Gain Scheduling
**Validasi:** Gain scheduling memberikan respons yang optimal di setiap kecepatan.

**Implementasi:**
- GS_SPEED_LOW_MAX = 60
- GS_SPEED_MID_MAX = 100
- GS_KP_LOW = 1.2, GS_KP_MID = 1.0, GS_KP_HIGH = 0.85
- GS_KI_LOW = 1.0, GS_KI_MID = 0.8, GS_KI_HIGH = 0.6
- GS_KD_LOW = 0.5, GS_KD_MID = 0.8, GS_KD_HIGH = 1.2

### 6.3 Heading Correction
**Validasi:** Heading correction berbasis gyro Z memberikan koreksi yang akurat.

**Implementasi:**
- HEADING_GAIN = 0.5 (gain rendah untuk correction yang halus)
- HEADING_MAX_OFFSET = 15 (offset maksimum)

## 7. Risiko & Mitigasi

### 7.1 Integral Windup
**Risiko:** I term melambung saat actuator saturasi. **Mitigasi:** Anti windup dengan clamping ±50.

### 7.2 Derivative Noise
**Risiko:** Kd bisa meningkatkan noise sensor. **Mitigasi:** Gunakan interval PID tetap, batasi gain.

### 7.3 Overshoot
**Risiko:** Kp tinggi menyebabkan overshoot. **Mitigasi:** Gain scheduling, heading correction, deadband.

## 8. Referensi

1. Arduino PID Library - https://www.arduino.cc/en/Tutorial/PIDLibrary
2. Anti-windup - https://www.controleng.com/fundamentals/anti-windup-control/
3. Gain Scheduling - https://www.arduino.cc/en/Tutorial/GainScheduling
4. Derivative Noise - https://www.arduino.cc/en/Tutorial/Derivative
5. Deadband - https://www.arduino.cc/en/Tutorial/Deadband

## 9. Catatan Perkembangan

### 9.1 Iterasi Tuning
- v1.0: PID biasa, Ki tinggi, Kd rendah
- v1.1: Anti windup, gain scheduling
- v1.2: Heading correction sebagai lapisan atas
- v1.3: Deadband servo, interval tetap
- v1.4: Adaptive threshold, tuning via antarmuka

### 9.2 Validasi Tuning
- Manual tuning dengan robot pada lintasan yang diketahui
- Uji pertigaan (belok kiri, lurus, belok kanan)
- Uji kecepatan tinggi dan rendah
- Uji kalibrasi sensor

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/penggerak.md` untuk dokumentasi API
- Lihat `docs/design-decisions/optional_features.md` untuk fitur opsional lainnya
- Lihat `docs/design-decisions/performance.md` untuk optimasi kinerja