// pengaman.h — keselamatan: baterai, miring, halangan, macet, lampu gelap.
#ifndef PENGAMAN_H
#define PENGAMAN_H

#include "ucode.h"
#include "config.h"
#include "status.h"

class SensorGaris;
class Penggerak;

class Pengaman {
 public:
  Pengaman(Status &st, Penggerak &g, SensorGaris &sen);
  // true = semua fungsi harus berhenti (pemanggil: return dari loop).
  bool bateraiKritis();
  bool miring();
  bool halangan();
  bool macet(int bacaKiri, int bacaKanan);
  void lampuOtomatis();
  void resetDeteksi();  // referensi macet adaptasi ulang
  float tegangan();

 private:
  void bacaIMU();
  void resetSemua();  // stop + buang state semua modul
  Status &status;
  Penggerak &gerak;
  SensorGaris &sensor;

  unsigned long cekCahayaTerakhir = 0;
  bool lampuGelapNyala = false;
  unsigned long cekHalanganTerakhir = 0;
  bool halanganDepan = false;
  unsigned long cekBateraiTerakhir = 0;
  bool bateraiLemah = false;
  unsigned long imuTerakhir = 0;
  float imuRoll = 0, imuPitch = 0, imuAccelMag = 0;
  unsigned long irBekuSejak = 0;
  int irRefKiri = 0, irRefKanan = 0;
  float accelRef = -1;
};

#endif
