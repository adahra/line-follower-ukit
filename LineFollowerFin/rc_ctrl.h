// rc_ctrl.h — Remote Control (RC) mode untuk LineFollowerFin
// Membaca sinyal RC receiver (PWM/PPM pada digital pin) atau Serial (ESP-based).
// Channel 1 = throttle, channel 2 = steering. Dua channel cukup untuk dasar.
#ifndef RC_CTRL_H
#define RC_CTRL_H

#include <Arduino.h>
#include "fwd.h"
#include "config.h"
#include "status.h"

// Tipe sumber RC
enum RCSource {
  RC_NONE = 0,
  RC_PWM,        // pulseIn pada digital pin
  RC_SERIAL      // Serial ESP-based receiver
};

class RcCtrl {
 public:
  RcCtrl(Status &st, Penggerak &g, SensorGaris &sen, Pengaman &am, LapTimer &lt);

  // Inisialisasi
  void begin(unsigned long baud = 115200, int pin = 2, RCSource src = RC_PWM);

  // Dipanggil setiap loop
  void update();

  // Baca lebar pulse (us) dari channel throttle/steering
  unsigned long readPulseWidth_us(int pin);

  // Map pulse ke speed kiri/kanan
  void mapToSpeed(int pulseUs, int &outSpeedKiri, int &outSpeedKanan) const;

  // Enable/disable RC control
  bool isEnabled() const;
  void setEnabled(bool on);

  // Set mode operasi ke MODE_RC saat RC aktif
  void setMode(ModeOperasi mode);

  // Akses nilai throttle/steering (0-100 throttle, -100..100 steering)
  int getThrottle() const;
  int getSteering() const;

  // Cek apakah sinyal RC masih hidup
  bool isSignalLost() const;

  // Reset state RC
  void reset();

  // Hitung speed yang diinginkan dari throttle/steering
  void getSpeeds(int &outSpeedKiri, int &outSpeedKanan) const;

 private:
  Status &status;
  Penggerak &gerak;
  SensorGaris &sensor;
  Pengaman &aman;
  LapTimer &lapTimer;

  RCSource rcSource = RC_NONE;
  int rcPin = 2;
  unsigned long rcBaud = 115200;

  // Pulse measurement state
  unsigned long lastPulseTime = 0;
  unsigned long lastThrottleUs = RC_CENTER_US;
  unsigned long lastSteeringUs = RC_CENTER_US;
  bool signalLost = true;

  // Cached command values
  int throttlePct = 0;        // 0-100
  int steeringPct = 0;        // -100..100

  // Serial buffer (jika RC_SERIAL)
  static const int RC_CMD_BUFFER_SIZE = 64;
  char rcCmdBuffer[RC_CMD_BUFFER_SIZE];
  int rcCmdBufferPos = 0;
  unsigned long lastSerialCheck = 0;
  unsigned long serialCheckInterval = 50;

  // Helper
  void readPwmChannel();
  void mapToSpeeds();
  void processSerialLine(const char *line);
};

#endif