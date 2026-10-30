// config.cpp — definisi variabel global (hanya satu translation unit).
#include "config.h"

// Zigzag di lurus + mengikuti di tikungan = Kp/Kd masih terlalu tinggi di
// daerah error kecil (overcorrect). Tikungan OK berarti Kp cukup; jangan
// turun terlalu jauh. Kd turun paling banyak (D noise pemicu zigzag).
float Kp = 6.0;
float Ki = 0.03;
float Kd = 2.0;

int kecepatanDasarMaks = 110;
int kecepatanDasarMin = 55;  // 50->55: di atas deadband 25, tikungan tak mati total
int kecepatanDasar = 90;

// Default ambang: 8 (tengah rentang valid 5-14). 3 di bawah rentang -> sensor
// selalu baca di bawah ambang -> dianggap "hilang garis" terus-menerus.
int AMBANG_KIRI = 8;
int AMBANG_KANAN = 8;