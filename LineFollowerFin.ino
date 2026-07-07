#include "uKitExplore2.h"
#include <EEPROM.h> 

// --- PENGATURAN PID ---
float Kp = 21.7;  
float Ki = 0.01; 
float Kd = 1.5;  

int kecepatanDasarMaks = 110; // Batas kecepatan tertinggi saat di jalan lurus
int kecepatanDasarMin  = 50;  // Batas kecepatan terendah saat menikung tajam
int kecepatanDasar     = 90;  // Variabel dinamis yang akan dikalibrasi oleh PID

int kecepatanKiri = 0;
int kecepatanKanan = 0;

// Variabel internal PID
float error = 0, lastError = 0;
float P, I, D, PID_value;

// --- VARIABEL KALIBRASI & STATUS ---
int AMBANG_BATAS = 8; 
bool robotJalan = false; 
bool modeSetelKd = false; 

// --- ALAMAT ADDRESS EEPROM ---
const int ADDR_EEPROM_CHECK = 0; 
const int ADDR_KP = 4;           
const int ADDR_KD = 8;           
const int ADDR_SPEED = 12;       
const int ADDR_AMBANG = 16;      

void setup() {
  Initialization();

  bacaNilaiDariEEPROM();
  mulaiKalibrasiSensor(); // Kalibrasi ambang batas sensor saat menyala
}

void loop() {
  cekTombol();

  if (robotJalan == true) {
    int bacaKiri = readInfraredDistance(2);  
    int bacaKanan = readInfraredDistance(1); 

    // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
    if (bacaKiri >= AMBANG_BATAS && bacaKanan >= AMBANG_BATAS) {
      setRgbledColor(255, 255, 255); 
      majuManual(kecepatanDasarMin, 150); // Lewati perempatan dengan kecepatan aman

      if (bacaKiri > bacaKanan) {
        eksekusiBelokKanan90();
      } else if (bacaKanan > bacaKiri) {
        eksekusiBelokKiri90();
      } else {
        eksekusiMajuTerus();
      }
      setRgbledColor(0, 0, 0); 
    } 
    
    // --- KONDISI B: LOST LINE RECOVERY ---
    else if (bacaKiri < AMBANG_BATAS && bacaKanan < AMBANG_BATAS) {
      setRgbledColor(255, 0, 255); 
      int kecCariGaris = kecepatanDasarMin; 

      if (lastError > 0) {
        setServoTurn(4, 0, kecCariGaris); setServoTurn(2, 0, kecCariGaris);
        setServoTurn(1, 0, kecCariGaris); setServoTurn(3, 0, kecCariGaris);
      } else {
        setServoTurn(4, 1, kecCariGaris); setServoTurn(2, 1, kecCariGaris);
        setServoTurn(1, 1, kecCariGaris); setServoTurn(3, 1, kecCariGaris);
      }
    }
    
    // --- KONDISI C: MODE TRACER PID NORMAL DENGAN ADAPTIVE SPEED ---
    else {
      setRgbledColor(0, 0, 0); 
      
      // 1. Hitung Nilai Error
      error = (bacaKiri - bacaKanan) / 2;

      // 2. Kalkulasi PID Standar
      P = error;
      I = I + error;
      I = constrain(I, -50, 50); 
      D = error - lastError;
      
      PID_value = (Kp * P) + (Ki * I) + (Kd * D);
      lastError = error; 

      // 3. --- LOGIKA KALIBRASI KECEPATAN DASAR BERBASIS PID ---
      // Semakin besar nilai PID_value (semakin melenceng), kecepatan dasar akan semakin dikurangi.
      // abs() digunakan agar nilai koreksi negatif/positif tetap dihitung sebagai nilai mutlak pengurangan.
      int faktorPengurang = abs(PID_value) * 0.6; // Pengali 0.6 adalah keagresifan pengereman (bisa di-tuning)
      
      kecepatanDasar = kecepatanDasarMaks - faktorPengurang;
      kecepatanDasar = constrain(kecepatanDasar, kecepatanDasarMin, kecepatanDasarMaks); 

      // 4. Hitung Kecepatan Motor Akhir Menggunakan Kecepatan Dasar yang Sudah Beradaptasi
      kecepatanKiri = kecepatanDasar - PID_value;
      kecepatanKanan = kecepatanDasar + PID_value;

      // 5. Batasi Output Akhir Kecepatan Motor ke Servo uKit (0 - 150)
      kecepatanKiri = constrain(kecepatanKiri, 0, 150);
      kecepatanKanan = constrain(kecepatanKanan, 0, 150);

      jalankanMotorPID(kecepatanKiri, kecepatanKanan);
    }
  } 
  else {
    berhenti();
    if (modeSetelKd == true) setRgbledColor(255, 0, 255); 
    else setRgbledColor(0, 0, 0);     
  }
  delay(2); 
}

// --- FUNGSI KALIBRASI SENSOR ---
void mulaiKalibrasiSensor() {
  Serial.println("--- MEMULAI KALIBRASI SENSOR ---");
  int nilaiMaks = 0;   
  int nilaiMin = 20;  

  setRgbledColor(255, 255, 0); delay(500);
  setServoTurn(4, 0, 50); setServoTurn(2, 0, 50);
  setServoTurn(1, 0, 50); setServoTurn(3, 0, 50);

  unsigned long waktuMulai = millis();
  while (millis() - waktuMulai < 3000) {
    int sensor1 = readInfraredDistance(1); int sensor2 = readInfraredDistance(2);
    if (sensor1 > nilaiMaks && sensor1 <= 20) nilaiMaks = sensor1;
    if (sensor2 > nilaiMaks && sensor2 <= 20) nilaiMaks = sensor2;
    if (sensor1 < nilaiMin && sensor1 > 0) nilaiMin = sensor1;
    if (sensor2 < nilaiMin && sensor2 > 0) nilaiMin = sensor2;
    delay(10); 
  }
  berhenti();

  AMBANG_BATAS = (nilaiMaks + nilaiMin) / 2;
  AMBANG_BATAS = constrain(AMBANG_BATAS, 5, 14); 
  simpanNilaiKeEEPROM();

  for (int i = 0; i < 3; i++) { setRgbledColor(0, 255, 0); delay(200); setRgbledColor(0, 0, 0); delay(200); }
}

// --- MANAJEMEN TOMBOL ---
void cekTombol() {
  int buttonState1 = readButtonValue(1);    
  int buttonState2 = readButtonValue(2); 
  
  if (buttonState1 == 1) {        
    if (!modeSetelKd) { Kp += 1.0; Serial.print("Kp: "); Serial.println(Kp); setRgbledColor(255, 0, 0); } 
    else { Kd += 0.5; Serial.print("Kd: "); Serial.println(Kd); setRgbledColor(0, 0, 255); }
    delay(300); setRgbledColor(0, 0, 0);
  } 
  else if (buttonState1 == 2) { 
    if (!modeSetelKd) { Kp -= 1.0; if(Kp < 0) Kp = 0; Serial.print("Kp: "); Serial.println(Kp); setRgbledColor(0, 255, 0); } 
    else { Kd -= 0.5; if(Kd < 0) Kd = 0; Serial.print("Kd: "); Serial.println(Kd); setRgbledColor(0, 255, 255); }
    delay(300); setRgbledColor(0, 0, 0);
  } 
  else if (buttonState1 == 3) { 
    if (robotJalan) {
      robotJalan = false;      
      setEyelightLook(1,0,5,254,0,0); setEyelightLook(2,0,5,254,0,0); 
      delay(1000);
    } else {
      modeSetelKd = !modeSetelKd; 
      setRgbledColor(255, 255, 255); delay(150); setRgbledColor(0, 0, 0); delay(150);
    }
  }

  if (buttonState2 == 1) {
    kecepatanDasarMaks += 10; if (kecepatanDasarMaks > 150) kecepatanDasarMaks = 150;
    setRgbledColor(255, 255, 0); delay(400); setRgbledColor(0, 0, 0);
    Serial.print("Target Speed Maks: "); Serial.println(kecepatanDasarMaks);
  }
  else if (buttonState2 == 2) {
    kecepatanDasarMaks -= 10; if (kecepatanDasarMaks < 60) kecepatanDasarMaks = 60;
    setRgbledColor(0, 255, 255); delay(400); setRgbledColor(0, 0, 0);
    Serial.print("Target Speed Maks: "); Serial.println(kecepatanDasarMaks);
  }
  else if (buttonState2 == 3) {
    if (!robotJalan) {
      simpanNilaiKeEEPROM();
      robotJalan = true;
      setEyelightLook(1,0,7,0,0,254); setEyelightLook(2,0,7,0,0,254); 
      setRgbledColor(255, 255, 255); delay(1000); setRgbledColor(0, 0, 0);
    }
  }
}

// --- DRIVER PERGERAKAN ---
void majuManual(int speed, int durasi) {
  setServoTurn(4, 1, speed); setServoTurn(2, 1, speed);
  setServoTurn(1, 0, speed); setServoTurn(3, 0, speed);
  delay(durasi);
}
void eksekusiBelokKiri90() {
  setServoTurn(4, 0, kecepatanDasarMin); setServoTurn(2, 0, kecepatanDasarMin);
  setServoTurn(1, 0, kecepatanDasarMin); setServoTurn(3, 0, kecepatanDasarMin);
  delay(300); while (readInfraredDistance(2) < AMBANG_BATAS) { delay(1); }
}
void eksekusiBelokKanan90() {
  setServoTurn(4, 1, kecepatanDasarMin); setServoTurn(2, 1, kecepatanDasarMin);
  setServoTurn(1, 1, kecepatanDasarMin); setServoTurn(3, 1, kecepatanDasarMin);
  delay(300); while (readInfraredDistance(1) < AMBANG_BATAS) { delay(1); }
}
void eksekusiMajuTerus() { majuManual(kecepatanDasarMin, 100); }
void jalankanMotorPID(int speedKiri, int speedKanan) {
  setServoTurn(4, 1, speedKiri); setServoTurn(2, 1, speedKiri);  
  setServoTurn(1, 0, speedKanan); setServoTurn(3, 0, speedKanan); 
}
void berhenti() { setServoStop(4); setServoStop(2); setServoStop(1); setServoStop(3); }

// --- EEPROM ---
void simpanNilaiKeEEPROM() {
  EEPROM.put(ADDR_KP, Kp); EEPROM.put(ADDR_KD, Kd);
  EEPROM.put(ADDR_SPEED, kecepatanDasarMaks); EEPROM.put(ADDR_AMBANG, AMBANG_BATAS);
  EEPROM.write(ADDR_EEPROM_CHECK, 123);
}
void bacaNilaiDariEEPROM() {
  if (EEPROM.read(ADDR_EEPROM_CHECK) == 123) {
    EEPROM.get(ADDR_KP, Kp); EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SPEED, kecepatanDasarMaks); EEPROM.get(ADDR_AMBANG, AMBANG_BATAS);
  }
}
