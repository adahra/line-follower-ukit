#include "sensor_garis.h"
#include "penggerak.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include "KalmanMPU6050.h"  // IMU untuk kalibrasi putaran 360°

void SensorGaris::resetFilter() {
  filtKiri = -1;
  filtKanan = -1;
}

// Filter eksponensial: redam 1 bacaan liar tanpa menambah delay.
int SensorGaris::bacaHalus(int id) {
  float &filt = (id == 2) ? filtKiri : filtKanan;
  int mentah = readInfraredDistance(id);
  if (filt < 0) {
    filt = mentah;
  } else {
    filt += ALPHA_SENSOR * (mentah - filt);
  }

  return (int)(filt + 0.5);
}

int SensorGaris::bacaKiri() {
  return bacaHalus(2);
}

int SensorGaris::bacaKanan() {
  return bacaHalus(1);
}

void SensorGaris::kalibrasi(Penggerak &gerak, bool manual) {
  Serial.println(manual ? F("--- MEMULAI KALIBRASI MANUAL ---")
                        : F("--- MEMULAI KALIBRASI SENSOR ---"));
  int maksKiri = 0, minKiri = 20;
  int maksKanan = 0, minKanan = 20;

  if (manual) {
    Serial.println(F("Mode MANUAL: motor diam, putar robot 360 derajat dengan tangan."));
  }
  setRgbledColor(255, 255, 0);
  delay(200);  // beri waktu robot tenang sebelum sampling bias gyro

#if CALIB_USE_IMU
  // Bias gyro harus diukur saat robot benar-benar diam. Tanpa ini,
  // integrasi sudut bisa berhenti terlalu cepat atau terlambat.
  float gyroBias = 0.0;
  int biasSamples = 0;
  unsigned long biasStart = millis();
  while (millis() - biasStart < CALIB_GYRO_BIAS_MS) {
    IMU::read();
    gyroBias += IMU::getRawGyroZ();
    biasSamples++;
    delay(5);
  }
  if (biasSamples > 0) {
    gyroBias /= (float)biasSamples;
  }
  if (manual) {
    Serial.println(F("Bias gyro siap. Sekarang putar robot 360 derajat dengan tangan."));
  } else {
    Serial.println(F("Kalibrasi: bias gyro siap, target putaran 360 derajat"));
  }
#endif

  if (manual) {
    gerak.berhenti();  // mode manual: roda tidak boleh ikut menggerakkan robot
  } else {
    gerak.putarKalibrasi();
  }

  unsigned long spinStart = millis();
#if CALIB_USE_IMU
  unsigned long lastUpdate = spinStart;
  float angle = 0.0;
  unsigned long batasWaktu = manual ? CALIB_MANUAL_TIMEOUT_MS : CALIB_SPIN_TIMEOUT_MS;
#else
  unsigned long batasWaktu = manual ? CALIB_MANUAL_TIMEOUT_MS : CALIB_FALLBACK_MS;
#endif
  bool selesai = false;

  while (!selesai) {
#if CALIB_USE_IMU
    IMU::read();
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;
    if (dt > 0.0) {
      angle += (IMU::getRawGyroZ() - gyroBias) * dt;
    }

    if (fabs(angle) >= CALIB_TARGET_DERAJAT) {
      selesai = true;
    } else if (now - spinStart >= batasWaktu) {
      selesai = true;
    }
#else
    if (millis() - spinStart >= batasWaktu) {
      selesai = true;
    }
#endif

    if (selesai) {
      break;
    }

    // Sampling mentah (bukan filter) agar min/max mencakup hitam dan putih.
    int sensorKanan = readInfraredDistance(1);
    int sensorKiri = readInfraredDistance(2);
    if (sensorKanan > maksKanan && sensorKanan <= 20) {
      maksKanan = sensorKanan;
    }

    if (sensorKiri > maksKiri && sensorKiri <= 20) {
      maksKiri = sensorKiri;
    }

    if (sensorKanan < minKanan && sensorKanan > 0) {
      minKanan = sensorKanan;
    }

    if (sensorKiri < minKiri && sensorKiri > 0) {
      minKiri = sensorKiri;
    }

    delay(CALIB_SAMPLE_MS);
  }

  gerak.berhenti();

#if CALIB_USE_IMU
  Serial.print(F("Kalibrasi: sudut terdeteksi "));
  Serial.print(angle, 1);
  Serial.println(F(" derajat"));

  if (fabs(angle) < CALIB_MIN_ROTASI_DERAJAT) {
    Serial.println(F("Kalibrasi GAGAL: putaran tidak mencapai minimum."));
    Serial.println(F("Ambang lama dipertahankan; ulangi kalibrasi."));
    setRgbledColor(255, 0, 0);
    delay(1000);
    setRgbledColor(0, 0, 0);
    return;
  }
#endif

  // Pastikan kedua sensor benar-benar melihat kontras hitam/putih.
  if (maksKiri <= minKiri || maksKanan <= minKanan) {
    Serial.println(F("Kalibrasi GAGAL: kontras sensor tidak cukup."));
    Serial.println(F("Ambang lama dipertahankan; cek posisi/lighting."));
    setRgbledColor(255, 0, 0);
    delay(1000);
    setRgbledColor(0, 0, 0);
    return;
  }

  AMBANG_KANAN = constrain((maksKanan + minKanan) / 2, 3, 18);
  AMBANG_KIRI = constrain((maksKiri + minKiri) / 2, 3, 18);
  Serial.print(F("Ambang kiri: "));
  Serial.print(AMBANG_KIRI);
  Serial.print(F(" kanan: "));
  Serial.println(AMBANG_KANAN);
  simpan();

  for (int i = 0; i < 3; i++) {
    setRgbledColor(0, 255, 0);
    delay(200);
    setRgbledColor(0, 0, 0);
    delay(200);
  }
}

void SensorGaris::simpan() {
  // Use new versioned EEPROM layout
  eepromWritePID(Kp, Ki, Kd);
  eepromWriteSpeed(kecepatanDasarMaks, kecepatanDasarMin);
  eepromWriteThresholds(AMBANG_KIRI, AMBANG_KANAN);
#if ENABLE_ADAPTIVE_THRESHOLD
  // Adaptive threshold base values
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KIRI, (uint16_t)AMBANG_KIRI);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KANAN, (uint16_t)AMBANG_KANAN);
  EEPROM.update(EEPROM_ADDR::EE_ADDR_ADAPT_MAGIC, MAGIC_ADAPT_VALID);
  EEPROM.update(EEPROM_ADDR::EE_ADDR_ADAPT_ENABLED, adaptiveThresholdEnabled ? 1 : 0);
#endif
  EEPROM.update(EEPROM_ADDR::EE_ADDR_MAGIC, MAGIC_EEPROM_VALID);
  EEPROM.update(EEPROM_ADDR::EE_ADDR_VERSION, EEPROM_FMT_VERSION);
}

void SensorGaris::muat() {
  // Use new versioned EEPROM layout
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID &&
      EEPROM.read(EEPROM_ADDR::EE_ADDR_VERSION) == EEPROM_FMT_VERSION) {
    eepromReadPID(Kp, Ki, Kd);
    eepromReadSpeed(kecepatanDasarMaks, kecepatanDasarMin);
    eepromReadThresholds(AMBANG_KIRI, AMBANG_KANAN);
    // Clamp thresholds to calibration range
    AMBANG_KIRI = constrain(AMBANG_KIRI, 3, 18);
    AMBANG_KANAN = constrain(AMBANG_KANAN, 3, 18);
#if ENABLE_ADAPTIVE_THRESHOLD
    // Load adaptive base if magic valid
    if (EEPROM.read(EEPROM_ADDR::EE_ADDR_ADAPT_MAGIC) == MAGIC_ADAPT_VALID) {
      uint16_t baseKiri, baseKanan;
      EEPROM.get(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KIRI, baseKiri);
      EEPROM.get(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KANAN, baseKanan);
      if (baseKiri >= 1 && baseKiri <= 20 && baseKanan >= 1 && baseKanan <= 20) {
        AMBANG_KIRI = constrain(baseKiri, 3, 18);
        AMBANG_KANAN = constrain(baseKanan, 3, 18);
      }
      adaptAmbangKiri = AMBANG_KIRI;
      adaptAmbangKanan = AMBANG_KANAN;
      adaptiveThresholdEnabled = EEPROM.read(EEPROM_ADDR::EE_ADDR_ADAPT_ENABLED) != 0;
    }
#endif
  }
}

// --- Adaptive Threshold methods ---
#if ENABLE_ADAPTIVE_THRESHOLD
int SensorGaris::ambangDinamis(int id) const {
  return (id == 2) ? adaptAmbangKiri : adaptAmbangKanan;
}

void SensorGaris::setAdaptiveThreshold(bool on) {
  adaptiveThresholdEnabled = on;
  if (on) resetAdaptive();
}

bool SensorGaris::isAdaptiveThresholdEnabled() const {
  return adaptiveThresholdEnabled;
}

void SensorGaris::updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar) {
  // Ambang dinamis = base * (1 + speedFactor), di-clamp.
  float speedFactor = 1.0 + (kecepatanDasar / 150.0) * ADAPT_SPEED_FACTOR;
  adaptAmbangKiri = constrain((int)(AMBANG_KIRI * speedFactor), ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
  adaptAmbangKanan = constrain((int)(AMBANG_KANAN * speedFactor), ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
  (void)bacaKiri;
  (void)bacaKanan;
}

void SensorGaris::resetAdaptive() {
  adaptAmbangKiri = AMBANG_KIRI;
  adaptAmbangKanan = AMBANG_KANAN;
}

void SensorGaris::simpanAdaptive() {
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KIRI, (uint16_t)AMBANG_KIRI);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KANAN, (uint16_t)AMBANG_KANAN);
  EEPROM.update(EEPROM_ADDR::EE_ADDR_ADAPT_MAGIC, MAGIC_ADAPT_VALID);
  EEPROM.update(EEPROM_ADDR::EE_ADDR_ADAPT_ENABLED, adaptiveThresholdEnabled ? 1 : 0);
}

void SensorGaris::muatAdaptive() {
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_ADAPT_MAGIC) != MAGIC_ADAPT_VALID) {
    resetAdaptive();  // belum pernah disimpan: pakai ambang saat ini
    return;
  }

  uint16_t baseKiri = 0, baseKanan = 0;
  EEPROM.get(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KIRI, baseKiri);
  EEPROM.get(EEPROM_ADDR::EE_ADDR_ADAPT_AMBANG_KANAN, baseKanan);
  if (baseKiri >= 1 && baseKiri <= 20 && baseKanan >= 1 && baseKanan <= 20) {
    AMBANG_KIRI = constrain(baseKiri, 3, 18);
    AMBANG_KANAN = constrain(baseKanan, 3, 18);
  }
  adaptAmbangKiri = AMBANG_KIRI;
  adaptAmbangKanan = AMBANG_KANAN;
  adaptiveThresholdEnabled = EEPROM.read(EEPROM_ADDR::EE_ADDR_ADAPT_ENABLED) != 0;
}
#endif

void SensorGaris::cetak() {
  Serial.println(F("--- NILAI TERSIMPAN ---"));
  bool valid = (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID &&
                EEPROM.read(EEPROM_ADDR::EE_ADDR_VERSION) == EEPROM_FMT_VERSION);
  Serial.print(F("EEPROM valid: "));
  Serial.println(valid ? F("YA") : F("TIDAK (pakai default kode)"));
  Serial.print(F("EEPROM version: "));
  Serial.println(EEPROM.read(EEPROM_ADDR::EE_ADDR_VERSION));
  Serial.print(F("Kp: "));
  Serial.println(Kp);
  Serial.print(F("Ki: "));
  Serial.println(Ki);
  Serial.print(F("Kd: "));
  Serial.println(Kd);
  Serial.print(F("Speed maks: "));
  Serial.println(kecepatanDasarMaks);
  Serial.print(F("Speed min: "));
  Serial.println(kecepatanDasarMin);
  Serial.print(F("Ambang kiri: "));
  Serial.println(AMBANG_KIRI);
  Serial.print(F("Ambang kanan: "));
  Serial.println(AMBANG_KANAN);
#if ENABLE_ADAPTIVE_THRESHOLD
  Serial.print(F("Adaptive: "));
  Serial.println(adaptiveThresholdEnabled ? F("ON") : F("OFF"));
  Serial.print(F("Adapt base kiri: "));
  Serial.println(adaptAmbangKiri);
  Serial.print(F("Adapt base kanan: "));
  Serial.println(adaptAmbangKanan);
#endif
  Serial.println(F("-----------------------"));
}
