// heading_tracker.h — IMU yaw integration + heading correction.
// Menggabungkan gyro Z (derajat/detik) menjadi absolute heading (derajat)
// dengan kalibrasi bias gyro untuk kompensasi drift. Dipakai oleh Penggerak
// sebagai lapisan koreksi steering di atas PID.
//
// Catatan: gyro Z mengukur yaw rate RELATIF terhadap tubuh robot. Tanpa
// magnetometer, heading absolut tidak bisa diketahui dari gyro saja, tapi
// untuk line follower kita hanya butuh heading RELATIF (sejak start) untuk
// mendeteksi drift/slip dan mempertahankan arah lurus.
#ifndef HEADING_TRACKER_H
#define HEADING_TRACKER_H

#include <Arduino.h>
#include "config.h"  // agar HEADING_CALIB_MS terdefinisi sebelum #if guard

class HeadingTracker {
 public:
  HeadingTracker() = default;

  void begin();
  void update();
  void reset();
  void setTarget(float targetHeading);
  void setBias(float b);
  void calibrateBias(unsigned long durationMs = HEADING_CALIB_MS);

  float getHeading() const { return heading; }
  float getTargetHeading() const { return targetHeading; }
  float getHeadingError() const;
  float getGyroZ() const { return gyroZ; }
  float getBias() const { return gyroBias; }
  bool isBiasCalibrated() const { return biasCalibrated; }
  bool isEnabled() const { return enabled; }
  void setEnabled(bool on) { enabled = on; }
  bool isCalibrating() const { return calibrating; }

 private:
  float heading = 0.0;
  float targetHeading = 0.0;
  float gyroBias = 0.0;
  float gyroZ = 0.0;
  float lastUpdateTime = 0;
  bool enabled = false;
  bool biasCalibrated = false;
  bool calibrating = false;
  unsigned long calibStart = 0;
  unsigned long calibDuration = HEADING_CALIB_MS;
  int calibSamples = 0;
  float calibSum = 0.0;
};

#endif  // HEADING_TRACKER_H