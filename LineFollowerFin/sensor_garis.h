// sensor_garis.h — sensor garis IR: filter, ambang, kalibrasi, EEPROM.
#ifndef SENSOR_GARIS_H
#define SENSOR_GARIS_H

#include "ucode.h"
#include <EEPROM.h>
#include <string.h>  // memcmp untuk eepromPutHemat
#include "config.h"

// EEPROM hemat tulis: tulis hanya bila byte berubah
// (EEPROM ~100rb siklus; kalibrasi + save tiap start aman).
// Template wajib di header (definisi harus terlihat saat dipakai).
template <typename T> void eepromPutHemat(int addr, const T &nilai) {
  T lama;
  EEPROM.get(addr, lama);
  if (memcmp(&lama, &nilai, sizeof(T)) != 0) {
    EEPROM.put(addr, nilai);
  }
}

class Penggerak;  // forward: kalibrasi butuh gerak untuk berputar

class SensorGaris {
 public:
  void resetFilter();  // adaptasi ulang ke garis baru
  int bacaKiri();      // ID 2, sudah difilter
  int bacaKanan();     // ID 1, sudah difilter
  void kalibrasi(Penggerak &gerak);  // putar 3 dtk, hitung ambang per sensor
  void simpan();                     // Kp/Kd/speed/ambang -> EEPROM
  void muat();                       // EEPROM -> variabel (dijepit valid)
  void cetak();                      // dump Serial saat boot

 private:
  int bacaHalus(int id);
  float filtKiri = -1;  // <0 = belum diinisialisasi
  float filtKanan = -1;
};

#endif
