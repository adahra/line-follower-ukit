// heading_tracker.cpp — IMU yaw integration + heading correction.
// Menggabungkan gyro Z (derajat/detik) menjadi absolute heading (derajat)
// dengan kalibrasi bias gyro untuk kompensasi drift.
#include "heading_tracker.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include "KalmanMPU6050.h"  // class IMU (di luar ukit_extern.h)

void HeadingTracker::begin() {
  heading = 0.0;
  gyroZ = 0.0;
  lastUpdateTime = 0;
  enabled = false;
  calibrating = false;
  calibStart = 0;
  calibSamples = 0;
  calibSum = 0.0;
  // gyroBias, biasCalibrated, dan targetHeading SENGAJA tidak direset:
  // begin() dipanggil ulang saat fitur diaktifkan, kalibrasi/ target yang
  // sudah ada tidak boleh hilang.
}

void HeadingTracker::reset() {
  heading = 0.0;  // arah saat ini dianggap referensi (lurus)
  lastUpdateTime = 0;
}

void HeadingTracker::setTarget(float targetHeading) {
  this->targetHeading = targetHeading;
}

void HeadingTracker::setBias(float b) {
  gyroBias = b;
  biasCalibrated = true;
}

void HeadingTracker::calibrateBias(unsigned long durationMs) {
  calibrating = true;
  biasCalibrated = false;
  calibStart = millis();
  calibSamples = 0;
  calibSum = 0.0;
  calibDuration = (durationMs > 0) ? durationMs : HEADING_CALIB_MS;
}

float HeadingTracker::getHeadingError() const {
  float err = targetHeading - heading;
  // Wrap ke [-180, 180]
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return err;
}

void HeadingTracker::update() {
  // Kalibrasi bias harus tetap bisa jalan walau koreksi heading OFF
  // (perintah serial 'heading cal' dijalankan saat robot berhenti).
  if (!enabled && !calibrating) return;

  unsigned long now = millis();
  if (now - lastUpdateTime < HEADING_UPDATE_MS) return;

  // Hitung dt SEBELUM lastUpdateTime di-overwrite. Versi lama menghitung
  // (now - lastUpdateTime) setelah lastUpdateTime = now sehingga dt selalu 0.
  float dt = (now - lastUpdateTime) / 1000.0;
  bool punyaSampelSebelumnya = (lastUpdateTime != 0);
  lastUpdateTime = now;

  // --- Kalibrasi bias gyro (non-blocking) ---
  if (calibrating) {
    IMU::read();
    float rawZ = IMU::getRawGyroZ();
    calibSum += rawZ;
    calibSamples++;
    if (now - calibStart >= calibDuration) {
      if (calibSamples > 0) {
        gyroBias = calibSum / (float)calibSamples;
        biasCalibrated = true;
      }
      calibrating = false;
      Serial.print(F("Heading: gyro bias kalibrasi = "));
      Serial.print(gyroBias, 3);
      Serial.println(F(" °/s"));
    }
    return;
  }

  if (!enabled) return;

  // Jangan integrasikan celah waktu besar (mis. while-loop belok 90° atau
  // delay serial): laju gyro selama celah itu tidak pernah tersampel.
  if (!punyaSampelSebelumnya || dt <= 0.0 || dt > 0.25) return;

  // --- Integrasi gyro Z menjadi heading ---
  IMU::read();
  float rawZ = IMU::getRawGyroZ();
  gyroZ = rawZ - gyroBias;  // kompensasi bias (drift nol)

  heading += gyroZ * dt;

  // Wrap heading ke [-180, 180]
  while (heading > 180.0) heading -= 360.0;
  while (heading < -180.0) heading += 360.0;
}