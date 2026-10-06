// deklarasi.h — deklarasi maju seluruh fungsi + helper template.
// Wajib: generator prototipe otomatis Arduino gagal bila ada fungsi template
// di file .ino, sehingga fungsi di bawah loop() tak dikenal tanpa ini.
#ifndef DEKLARASI_H
#define DEKLARASI_H

#include <EEPROM.h>
#include <string.h>  // memcmp untuk eepromPutHemat

void cekTombol();
void cekTepukTangan();
void cekLampuGelap();
bool cekHalanganDepan();
bool cekBateraiLemah();
void bacaIMU();
bool cekMiring();
bool cekMacet(int bacaKiri, int bacaKanan);
void telemetri(int bacaKiri, int bacaKanan);
int bacaSensorHalus(int id, float &filt);
void resetStatePID();
int terapkanDeadband(int speed);
int batasiSlew(int target, int prev);
void majuManual(int speed, int durasi);
void eksekusiBelokKiri90();
void eksekusiBelokKanan90();
void eksekusiMajuTerus();
void jalankanMotorPID(int speedKiri, int speedKanan);
void berhenti();
void mulaiKalibrasiSensor();
void simpanNilaiKeEEPROM();
void bacaNilaiDariEEPROM();
void cetakNilaiEEPROM();

// --- EEPROM hemat tulis: tulis hanya bila byte berubah ---
// (EEPROM ~100rb siklus; kalibrasi tiap boot + save tiap start aman.)
template <typename T> void eepromPutHemat(int addr, const T &nilai) {
  T lama;
  EEPROM.get(addr, lama);
  if (memcmp(&lama, &nilai, sizeof(T)) != 0) {
    EEPROM.put(addr, nilai);
  }
}

#endif
