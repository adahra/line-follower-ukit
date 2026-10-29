// config.cpp — definisi variabel global (hanya satu translation unit).
#include "config.h"

// Zigzag di lurus = Kp/Kd terlalu tinggi (error=1 -> PID=32 -> overcorrect).
// Turunkan Kp agar koreksi kecil halus; Kd diturunkan agar D noise tak dipaku.
float Kp = 6.0;
float Ki = 0.15;
float Kd = 3.0;

int kecepatanDasarMaks = 110;
int kecepatanDasarMin = 55;  // 50->55: di atas deadband 25, tikungan tak mati total
int kecepatanDasar = 90;

// Default ambang: 8 (tengah rentang valid 5-14). 3 di bawah rentang -> sensor
// selalu baca di bawah ambang -> dianggap "hilang garis" terus-menerus.
int AMBANG_KIRI = 8;
int AMBANG_KANAN = 8;