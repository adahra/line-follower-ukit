// antarmuka.h — input manusia: tombol onboard + tepuk tangan.
#ifndef ANTARMUKA_H
#define ANTARMUKA_H

#include "ucode.h"
#include "config.h"
#include "status.h"

class SensorGaris;
class Penggerak;
class Pengaman;

class Antarmuka {
 public:
  Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am);
  void cekTombol();  // non-blocking: tanpa delay, pakai cooldown
  void cekTepuk();   // 1x tepuk = toggle jalan/berhenti

 private:
  void mulai(const char *via);  // start bersih dari tombol/tepuk
  void berhenti(const char *via);
  void resetSemua();
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
