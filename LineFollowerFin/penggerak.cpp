#include "penggerak.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include <EEPROM.h>
#include <string.h>

void Penggerak::reset() {
  error = 0;
  lastError = 0;
  P = 0;
  I = 0;
  D = 0;
  PID_value = 0;
  prevKiri = 0;
  prevKanan = 0;
  // Turbo ikut direset: tanpa ini, waktuLurusMulai/kecepatanTurboAktif
  // nyangkut antar run dan turbo bisa langsung aktif dari nilai lama.
  waktuLurusMulai = 0;
  kecepatanTurboAktif = 0;
  pidTerakhir = 0;
  telemHeader = false;  // cetak header CSV lagi di run berikutnya
  telemTerakhir = 0;
#if ENABLE_HEADING_CORRECTION
  // Mulai/berhenti = baseline arah baru. Bias gyro tetap dipertahankan.
  headingTracker.reset();
#endif
}

void Penggerak::resetIntegralSetelahRecovery(float errorBaru) {
  I = 0;
  lastError = errorBaru;  // cegah spike D di loop pertama
}

bool Penggerak::hitung(int bacaKiri, int bacaKanan, bool paksa) {
  // PID jalan pada interval tetap agar D konsisten.
  unsigned long nowPid = millis();
  if (!paksa && nowPid - pidTerakhir < PID_INTERVAL_MS) {
    return false;
  }

  pidTerakhir = nowPid;

  // 1. Hitung Nilai Error (pakai 2.0 agar tidak terpotong integer)
  error = (bacaKiri - bacaKanan) / 2.0;

  // 2. Kalkulasi PID Standar
  P = error;
  I = I + error;
  I = constrain(I, -BATAS_INTEGRAL, BATAS_INTEGRAL);
  D = error - lastError;

  // --- FUZZY ADAPTIVE PID ---
  // Jika fuzzy aktif, hitung delta Kp/Kd/Ki dari error & delta_error,
  // lalu gunakan gain efektif (base + delta) untuk PID.
#if ENABLE_FUZZY_PID
  float KpEff = Kp, KdEff = Kd, KiEff = Ki;
  if (fuzzyPid.isEnabled()) {
    float dKp = 0.0, dKd = 0.0, dKi = 0.0;
    fuzzyPid.compute(error, D, dKp, dKd, dKi);  // D = delta error
    KpEff = Kp + dKp;
    KdEff = Kd + dKd;
    KiEff = Ki + dKi;
    // Clamp: gain efektif tidak boleh negatif
    if (KpEff < 0.0) KpEff = 0.0;
    if (KdEff < 0.0) KdEff = 0.0;
    if (KiEff < 0.0) KiEff = 0.0;
  }
#else
  float KpEff = Kp, KdEff = Kd, KiEff = Ki;
#endif

  // --- GAIN SCHEDULING ---
  // Gain berbeda per kecepatan: rendah (tikungan) vs tinggi (lurus).
  // Low speed: Kp naik (responsif), Kd turun (tidak perlu damping keras).
  // High speed: Kp turun (anti overshoot), Kd naik (stabil).
#if ENABLE_GAIN_SCHEDULING
  if (gainSchedulingEnabled) {
    KpEff *= getGainScheduleKp();
    KdEff *= getGainScheduleKd();
    KiEff *= getGainScheduleKi();
  }
#endif

  PID_value = (KpEff * P) + (KiEff * I) + (KdEff * D);

  lastError = error;

  // --- IMU HEADING CORRECTION ---
  // Tambahkan steering offset dari heading error ke PID_value.
  // Koreksi ini berlaku sebagai "perekam arah": mencegah robot drift
  // di lintasan lurus panjang tanpa garis kuat di kedua sensor.
  // Hanya dihitung saat PID interval tiba (dt konstan, konsisten).
#if ENABLE_HEADING_CORRECTION
  if (headingTracker.isEnabled()) {
    // updateHeading() dipanggil dari loop() tiap iterasi (termasuk saat
    // berhenti/kalibrasi). Di sini cukup terapkan koreksinya saja.
    float headingErr = headingTracker.getHeadingError();
    float headingCorrection = headingErr * headingGain;
    headingCorrection = constrain(headingCorrection, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
    PID_value += headingCorrection;
  }
#endif

  // --- DYNAMIC SPEED PROFILE (Turbo Mode) ---
  // Jika error kecil (jalan lurus) dalam waktu lama, naikkan kecepatan dasar.
  if (abs(error) < LURUS_THRESHOLD) {
    if (waktuLurusMulai == 0) {
      waktuLurusMulai = millis();
    } else if (millis() - waktuLurusMulai > LURUS_TIME_REQUIRED) {
      // Masuk mode Turbo: naikkan kecepatan secara gradual agar tidak slip
      kecepatanTurboAktif += TURBO_RAMP_SPEED;
      if (kecepatanTurboAktif > KECEPATAN_TURBO) {
        kecepatanTurboAktif = KECEPATAN_TURBO;
      }
    }
  } else {
    // Tikungan terdeteksi: reset turbo segera
    waktuLurusMulai = 0;
    kecepatanTurboAktif = 0;
  }

  // 3. Kecepatan dasar adaptif: makin melenceng makin direm.
  int faktorPengurang =
    abs(PID_value) * FAKTOR_REM_ADAPTIF;  // agresivitas pengereman (bisa di-tuning)
  
  // Base speed adalah gabungan dari kecepatanDasarMaks + bonus Turbo - pengereman adaptif
  int baseSpeedTarget = kecepatanDasarMaks + kecepatanTurboAktif - faktorPengurang;
  kecepatanDasar = constrain(baseSpeedTarget, kecepatanDasarMin, SERVO_MAKS_DIPAKAI);

  // 4. Kecepatan motor akhir dari kecepatan dasar yang sudah beradaptasi
  kecepatanKiri = kecepatanDasar - PID_value;
  kecepatanKanan = kecepatanDasar + PID_value;

  // 5. Batasi Output Akhir (derate servo 0-150, bukan 0-255 docs)
  kecepatanKiri = constrain(kecepatanKiri, 0, SERVO_MAKS_DIPAKAI);
  kecepatanKanan = constrain(kecepatanKanan, 0, SERVO_MAKS_DIPAKAI);

  // 6. Deadband compensation: servo diam di speed 1-24, paksa ke
  // SERVO_MIN_EFEKTIF agar koreksi PID benar-benar menggerakkan roda.
  kecepatanKiri = terapkanDeadband(kecepatanKiri);
  kecepatanKanan = terapkanDeadband(kecepatanKanan);

  // 7. Slew-rate limiter: tahan lompatan speed per loop agar roda
  // servo tidak slip (open-loop, tanpa encoder).
  kecepatanKiri = batasiSlew(kecepatanKiri, prevKiri);
  kecepatanKanan = batasiSlew(kecepatanKanan, prevKanan);
  prevKiri = kecepatanKiri;
  prevKanan = kecepatanKanan;

  jalankan(kecepatanKiri, kecepatanKanan);
  telemetri(bacaKiri, bacaKanan);
  return true;
}

void Penggerak::terapkanTerakhir() {
  jalankan(prevKiri, prevKanan);
}

void Penggerak::jalankan(int speedKiri, int speedKanan) {
  setServoTurn(4, 1, speedKiri);
  setServoTurn(2, 1, speedKiri);
  setServoTurn(1, 0, speedKanan);
  setServoTurn(3, 0, speedKanan);
}

void Penggerak::jalankanArah(int speedKiri, int speedKanan) {
  int kiri = constrain(abs(speedKiri), 0, SERVO_MAKS_DIPAKAI);
  int kanan = constrain(abs(speedKanan), 0, SERVO_MAKS_DIPAKAI);
  kiri = terapkanDeadband(kiri);
  kanan = terapkanDeadband(kanan);

  if (kiri == 0) {
    setServoStop(4);
    setServoStop(2);
  } else {
    int dirKiri = (speedKiri > 0) ? 1 : 0;
    setServoTurn(4, dirKiri, kiri);
    setServoTurn(2, dirKiri, kiri);
  }

  if (kanan == 0) {
    setServoStop(1);
    setServoStop(3);
  } else {
    int dirKanan = (speedKanan > 0) ? 0 : 1;
    setServoTurn(1, dirKanan, kanan);
    setServoTurn(3, dirKanan, kanan);
  }
}

void Penggerak::berhenti() {
  setServoStop(4);
  setServoStop(2);
  setServoStop(1);
  setServoStop(3);
}

void Penggerak::saatBerhenti() {
  berhenti();
  prevKiri = 0;
  prevKanan = 0;
}

void Penggerak::majuManual(int speed, int durasi) {
  setServoTurn(4, 1, speed);
  setServoTurn(2, 1, speed);
  setServoTurn(1, 0, speed);
  setServoTurn(3, 0, speed);
  delay(durasi);
}

void Penggerak::majuTerus() {
  majuManual(kecepatanDasarMin, 100);
}

void Penggerak::maju(int speed) {
  setServoTurn(4, 1, speed);
  setServoTurn(2, 1, speed);
  setServoTurn(1, 0, speed);
  setServoTurn(3, 0, speed);
}

void Penggerak::putarKiri(int speed) {
  setServoTurn(4, 0, speed);
  setServoTurn(2, 0, speed);
  setServoTurn(1, 0, speed);
  setServoTurn(3, 0, speed);
}

void Penggerak::putarKanan(int speed) {
  setServoTurn(4, 1, speed);
  setServoTurn(2, 1, speed);
  setServoTurn(1, 1, speed);
  setServoTurn(3, 1, speed);
}

void Penggerak::belokKiri90(int ambang) {
  setServoTurn(4, 0, kecepatanDasarMin);
  setServoTurn(2, 0, kecepatanDasarMin);
  setServoTurn(1, 0, kecepatanDasarMin);
  setServoTurn(3, 0, kecepatanDasarMin);
  delay(300);
  unsigned long t0 = millis();
  while ((int)readInfraredDistance(2) < ambang) {
    if (millis() - t0 > (unsigned long)BELOK_TIMEOUT_MS) {
      break;  // garis tak ketemu: jangan macet, kembali ke loop utama
    }

    delay(1);
  }
}

void Penggerak::belokKanan90(int ambang) {
  setServoTurn(4, 1, kecepatanDasarMin);
  setServoTurn(2, 1, kecepatanDasarMin);
  setServoTurn(1, 1, kecepatanDasarMin);
  setServoTurn(3, 1, kecepatanDasarMin);
  delay(300);
  unsigned long t0 = millis();
  while ((int)readInfraredDistance(1) < ambang) {
    if (millis() - t0 > (unsigned long)BELOK_TIMEOUT_MS) {
      break;  // garis tak ketemu: jangan macet, kembali ke loop utama
    }

    delay(1);
  }
}

void Penggerak::putarKalibrasi() {
  // Speed 50 sering terlalu lemah untuk satu putaran penuh (terutama di
  // permukaan beralas karet); 90 memberi torsi cukup tanpa saturasi gyro.
  setServoTurn(4, 0, CALIB_SPIN_SPEED);
  setServoTurn(2, 0, CALIB_SPIN_SPEED);
  setServoTurn(1, 0, CALIB_SPIN_SPEED);
  setServoTurn(3, 0, CALIB_SPIN_SPEED);
}

void Penggerak::putarCariGaris(float lastErr) {
  int kecCariGaris = kecepatanDasarMin;
  if (lastErr > 0) {
    setServoTurn(4, 0, kecCariGaris);
    setServoTurn(2, 0, kecCariGaris);
    setServoTurn(1, 0, kecCariGaris);
    setServoTurn(3, 0, kecCariGaris);
  } else {
    setServoTurn(4, 1, kecCariGaris);
    setServoTurn(2, 1, kecCariGaris);
    setServoTurn(1, 1, kecCariGaris);
    setServoTurn(3, 1, kecCariGaris);
  }
}

float Penggerak::errorTerakhir() const {
  return lastError;
}

int Penggerak::terapkanDeadband(int speed) {
  if (speed == 0) {
    return 0;
  }

  if (speed < SERVO_MIN_EFEKTIF) {
    return SERVO_MIN_EFEKTIF;
  }

  return speed;
}

int Penggerak::batasiSlew(int target, int prev) {
  int selisih = target - prev;
  if (selisih > SLEW_MAKS_PER_LOOP) {
    return prev + SLEW_MAKS_PER_LOOP;
  }

  if (selisih < -SLEW_MAKS_PER_LOOP) {
    return prev - SLEW_MAKS_PER_LOOP;
  }

  return target;
}

// t,err,pid,dasar,kiri,kanan,senK,senN — throttle 100 ms, non-blocking.
void Penggerak::telemetri(int bacaKiri, int bacaKanan) {
#if ENABLE_TELEMETRI
  unsigned long now = millis();
  if (now - telemTerakhir < TELEM_MS) {
    return;
  }

  telemTerakhir = now;
  if (!telemHeader) {
    telemHeader = true;
    Serial.println(F("t,err,pid,dasar,kiri,kanan,senK,senN"));
  }

  Serial.print(now);
  Serial.print(F(","));
  Serial.print(error);
  Serial.print(F(","));
  Serial.print(PID_value);
  Serial.print(F(","));
  Serial.print(kecepatanDasar);
  Serial.print(F(","));
  Serial.print(kecepatanKiri);
  Serial.print(F(","));
  Serial.print(kecepatanKanan);
  Serial.print(F(","));
  Serial.print(bacaKiri);
  Serial.print(F(","));
  Serial.println(bacaKanan);
#else
  (void)bacaKiri;
  (void)bacaKanan;
#endif
}

// --- Fuzzy PID methods ---
#if ENABLE_FUZZY_PID
void Penggerak::setFuzzyEnabled(bool on) {
  fuzzyPid.setEnabled(on);
}

bool Penggerak::isFuzzyEnabled() const {
  return fuzzyPid.isEnabled();
}
#endif

// --- Gain Scheduling methods ---
#if ENABLE_GAIN_SCHEDULING
void Penggerak::setGainScheduling(bool on) {
  gainSchedulingEnabled = on;
}

bool Penggerak::isGainSchedulingEnabled() const {
  return gainSchedulingEnabled;
}

float Penggerak::getGainScheduleKp() const {
  if (kecepatanDasar < GS_SPEED_LOW_MAX) return GS_KP_LOW;
  if (kecepatanDasar < GS_SPEED_MID_MAX) return GS_KP_MID;
  return GS_KP_HIGH;
}

float Penggerak::getGainScheduleKd() const {
  if (kecepatanDasar < GS_SPEED_LOW_MAX) return GS_KD_LOW;
  if (kecepatanDasar < GS_SPEED_MID_MAX) return GS_KD_MID;
  return GS_KD_HIGH;
}

float Penggerak::getGainScheduleKi() const {
  if (kecepatanDasar < GS_SPEED_LOW_MAX) return GS_KI_LOW;
  if (kecepatanDasar < GS_SPEED_MID_MAX) return GS_KI_MID;
  return GS_KI_HIGH;
}
#endif

// --- IMU Heading Correction methods ---
#if ENABLE_HEADING_CORRECTION
void Penggerak::setHeadingCorrection(bool on) {
  if (on) {
    // begin() mereset integrasi/heading, tapi mempertahankan bias & target.
    // Dulu urutannya setEnabled(true) lalu begin() yang mengeset enabled=false,
    // sehingga fitur langsung mati begitu diaktifkan.
    if (!headingTracker.isEnabled()) {
      headingTracker.begin();
    }
    headingTracker.setEnabled(true);
  } else {
    headingTracker.setEnabled(false);
  }
}

bool Penggerak::isHeadingCorrectionEnabled() const {
  return headingTracker.isEnabled();
}

void Penggerak::updateHeading() {
  headingTracker.update();
}

void Penggerak::setHeadingTarget(float t) {
  headingTracker.setTarget(t);
}

float Penggerak::getHeading() const {
  return headingTracker.getHeading();
}

float Penggerak::getTargetHeading() const {
  return headingTracker.getTargetHeading();
}

float Penggerak::getHeadingError() const {
  return headingTracker.getHeadingError();
}

void Penggerak::resetHeading() {
  headingTracker.reset();
}

void Penggerak::calibrateHeadingBias() {
  headingTracker.calibrateBias();
}

bool Penggerak::isHeadingCalibrating() const {
  return headingTracker.isCalibrating();
}

void Penggerak::simpanHeading() {
  EEPROM.put(ADDR_HEADING_GAIN, headingGain);
  EEPROM.put(ADDR_HEADING_TARGET, headingTracker.getTargetHeading());
  EEPROM.put(ADDR_HEADING_BIAS, headingTracker.getBias());
  // Magic ditulis terakhir: data setengah jadi tidak dianggap valid.
  EEPROM.update(ADDR_HEADING_MAGIC, 88);
}

void Penggerak::muatHeading() {
  if (EEPROM.read(ADDR_HEADING_MAGIC) != 88) {
    return;  // belum pernah disimpan: pakai default config
  }

  float savedGain, savedTarget, savedBias;
  EEPROM.get(ADDR_HEADING_GAIN, savedGain);
  EEPROM.get(ADDR_HEADING_TARGET, savedTarget);
  EEPROM.get(ADDR_HEADING_BIAS, savedBias);

  // Validasi: EEPROM kosong terbaca NaN/0xFF, jangan sampai dipakai.
  if (savedGain >= 0.0 && savedGain <= 2.0) {
    headingGain = savedGain;
  }
  if (savedTarget >= -180.0 && savedTarget <= 180.0) {
    headingTracker.setTarget(savedTarget);
  }
  if (savedBias >= -20.0 && savedBias <= 20.0) {
    headingTracker.setBias(savedBias);
  }
}
#endif
