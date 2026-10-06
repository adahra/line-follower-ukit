
#include "penggerak.h"

void Penggerak::reset() {
  error = 0;
  lastError = 0;
  P = 0;
  I = 0;
  D = 0;
  PID_value = 0;
  prevKiri = 0;
  prevKanan = 0;
  pidTerakhir = 0;
  telemHeader = false;  // cetak header CSV lagi di run berikutnya
  telemTerakhir = 0;
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

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;

  // 3. Kecepatan dasar adaptif: makin melenceng makin direm.
  int faktorPengurang =
    abs(PID_value) * FAKTOR_REM_ADAPTIF;  // agresivitas pengereman (bisa di-tuning)
  kecepatanDasar = kecepatanDasarMaks - faktorPengurang;
  kecepatanDasar =
    constrain(kecepatanDasar, kecepatanDasarMin, kecepatanDasarMaks);

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

void Penggerak::belokKiri90(int ambang) {
  setServoTurn(4, 0, kecepatanDasarMin);
  setServoTurn(2, 0, kecepatanDasarMin);
  setServoTurn(1, 0, kecepatanDasarMin);
  setServoTurn(3, 0, kecepatanDasarMin);
  delay(300);
  unsigned long t0 = millis();
  while (readInfraredDistance(2) < ambang) {
    if (millis() - t0 > BELOK_TIMEOUT_MS) {
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
  while (readInfraredDistance(1) < ambang) {
    if (millis() - t0 > BELOK_TIMEOUT_MS) {
      break;  // garis tak ketemu: jangan macet, kembali ke loop utama
    }

    delay(1);
  }
}

void Penggerak::putarKalibrasi() {
  setServoTurn(4, 0, 50);
  setServoTurn(2, 0, 50);
  setServoTurn(1, 0, 50);
  setServoTurn(3, 0, 50);
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
  unsigned long now = millis();
  if (now - telemTerakhir < TELEM_MS) {
    return;
  }

  telemTerakhir = now;
  if (!telemHeader) {
    telemHeader = true;
    Serial.println("t,err,pid,dasar,kiri,kanan,senK,senN");
  }

  Serial.print(now);
  Serial.print(',');
  Serial.print(error);
  Serial.print(',');
  Serial.print(PID_value);
  Serial.print(',');
  Serial.print(kecepatanDasar);
  Serial.print(',');
  Serial.print(kecepatanKiri);
  Serial.print(',');
  Serial.print(kecepatanKanan);
  Serial.print(',');
  Serial.print(bacaKiri);
  Serial.print(',');
  Serial.println(bacaKanan);
}
