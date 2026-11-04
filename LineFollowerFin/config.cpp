// config.cpp — definisi variabel global (hanya satu translation unit).
#include "config.h"

// Default awal untuk arena loop dengan S ganda:
// - Kp sedikit diturunkan (8.0 -> 7.0) agar tidak zigzag.
// - Kd dinaikkan (1.5 -> 2.2) untuk meredam osilasi di S.
// - Ki diturunkan (0.1 -> 0.05) agar integral tidak menumpuk di tikungan.
float Kp = 7.0;
float Ki = 0.05;
float Kd = 2.2;

int kecepatanDasarMaks = 85;  // tuning awal; naikkan 5 poin per iterasi
int kecepatanDasarMin = 40;   // di atas deadband 25, tikungan tak mati total
int kecepatanDasar = 40;      // nilai awal sebelum PID menghitung

// Default ambang 6 (tengah rentang valid 3-18). Wajib dikalibrasi ulang di
// permukaan arena; nilai EEPROM yang valid akan menggantikan default ini.
int AMBANG_KIRI = 6;
int AMBANG_KANAN = 6;