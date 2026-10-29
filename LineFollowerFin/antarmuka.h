// antarmuka.h — input manusia: tombol onboard + tepuk tangan + serial command.
#ifndef ANTARMUKA_H
#define ANTARMUKA_H

#include <Arduino.h>
#include "fwd.h"
#include "config.h"
#include "status.h"

class Antarmuka {
 public:
  Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am);
  void cekTombol();   // non-blocking: tanpa delay, pakai cooldown
  void cekTepuk();    // 1x tepuk = toggle jalan/berhenti
  void cekSerial();   // parsing command serial: Kp/Ki/Kd/speed/ambang/calibrate/save/help

 private:
  void mulai(const char *via);
  void berhenti(const char *via);
  void resetSemua();
  void parseCommand(char *cmd);
  Status &status;
  SensorGaris &sensor;
  Penggerak &gerak;
  Pengaman &aman;

  unsigned long cooldownB1 = 0;
  unsigned long cooldownB2 = 0;
  unsigned long cekSuaraTerakhir = 0;
  unsigned long tepukTerakhir = 0;
  bool tepukSiap = true;
};

#endif
