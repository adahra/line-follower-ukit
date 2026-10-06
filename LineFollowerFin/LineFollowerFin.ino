#include "ucode.h"
#include <EEPROM.h>

// --- PENGATURAN PID ---
float Kp = 21.7;
float Ki = 0.01;
float Kd = 1.5;

int kecepatanDasarMaks = 110;  // Batas kecepatan tertinggi saat di jalan lurus
int kecepatanDasarMin = 50;    // Batas kecepatan terendah saat menikung tajam
int kecepatanDasar = 90;       // Variabel dinamis yang akan dikalibrasi oleh PID

// --- BATAS KHUSUS SERVO 360 (open-loop, lihat docs setServoTurn) ---
// Docs resmi: speed 0-255. 150 adalah derate yang disengaja agar robot stabil.
// Servo punya deadband: speed kecil hanya berdengung, roda tidak bergerak.
const int SERVO_MIN_EFEKTIF = 25;  // speed 1-24 dipaksa ke 25 atau 0
const int SERVO_MAKS_DIPAKAI =
  150;                                        // derate dari 255, jangan naikkan tanpa uji lintasan
const int SLEW_MAKS_PER_LOOP = 10;            // batas perubahan speed per loop anti slip
const unsigned long BELOK_TIMEOUT_MS = 1500;  // anti macet di while belok 90°
const unsigned long RECOVERY_TIMEOUT_MS =
  2000;                           // batas putar cari garis, lalu berhenti
unsigned long recoveryMulai = 0;  // 0 = tidak dalam mode recovery

// --- ANTI GOYANG SENSOR: filter eksponensial agar 1 bacaan liar
// tidak langsung jadi koreksi PID penuh (error range kecil, ±10).
float filtKiri = -1;  // <0 = belum diinisialisasi
float filtKanan = -1;
const float ALPHA_SENSOR =
  0.5;  // bobot bacaan baru 0-1, makin besar makin responsif

// --- PID interval tetap: D = error-lastError hanya valid bila dt konstan.
// Tanpa ini, delay tombol (300-1000ms) membuat D melonjak acak.
const unsigned long PID_INTERVAL_MS = 10;
unsigned long pidTerakhir = 0;

// --- TOMBOL NON-BLOCKING: cooldown millis() + edge-detect.
// delay() di sini membuat robot buta garis selama ratusan ms.
unsigned long cooldownB1 = 0;
unsigned long cooldownB2 = 0;
const unsigned long COOLDOWN_TOMBOL_MS = 300;

// --- LAMPU OTOMATIS SAAT GELAP (sensor cahaya + eye lamp) ---
// Docs: readLightValue(id) -> 0-4000 lux; setEyelightAllPetals(id,r,g,b).
// Sesuaikan ID sensor cahaya dengan port yang terpasang di robot.
const int LIGHT_SENSOR_ID = 1;
const int EYELIGHT_KIRI = 1;
const int EYELIGHT_KANAN = 2;
const int AMBANG_GELAP_NYALA = 100;       // lux: di bawah ini lampu menyala
const int AMBANG_GELAP_MATI = 150;        // lux: histeresis, di atas ini lampu mati
const unsigned long CEK_CAHAYA_MS = 200;  // throttle agar tak baca tiap loop
unsigned long cekCahayaTerakhir = 0;
bool lampuGelapNyala = false;

// --- HALANGAN DEPAN: berhenti bila objek <= 5 cm (sensor ultrasonic) ---
// Docs: readUltrasonicDistance(id) -> jarak 0-400 cm.
// Sesuaikan ULTRASONIC_ID dengan port sensor yang menghadap depan.
const int ULTRASONIC_ID = 1;
const int JARAK_BERHENTI_CM = 5;
const unsigned long CEK_HALANGAN_MS = 100;  // ultrasonic butuh jeda antar ping
unsigned long cekHalanganTerakhir = 0;
bool halanganDepan = false;

int prevKiri = 0;
int prevKanan = 0;

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

// --- DEKLARASI MAJU (wajib: generator prototipe Arduino gagal bila ada
// fungsi template di file, sehingga fungsi di bawah loop() tak dikenal) ---
void cekTombol();
void cekLampuGelap();
bool cekHalanganDepan();
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

void setup() {
  Initialization();
  if (protocolRunState == false) {
    bacaNilaiDariEEPROM();
    mulaiKalibrasiSensor();  // Kalibrasi ambang batas sensor saat menyala
  }
}

void loop() {
  protocol();
  if (protocolRunState == false) {
    cekTombol();
    cekLampuGelap();  // headlight otomatis, non-blocking (throttle 200ms)

    if (robotJalan == true) {
      // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
      if (cekHalanganDepan()) {
        return;
      }

      int bacaKiri = bacaSensorHalus(2, filtKiri);
      int bacaKanan = bacaSensorHalus(1, filtKanan);

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
      if (bacaKiri >= AMBANG_BATAS && bacaKanan >= AMBANG_BATAS) {
        setRgbledColor(255, 255, 255);
        majuManual(kecepatanDasarMin,
                   150);  // Lewati perempatan dengan kecepatan aman

        // Baca ulang setelah maju: keputusan belok harus pakai data segar,
        // bukan nilai sebelum majuManual yang sudah basi.
        bacaKiri = readInfraredDistance(2);
        bacaKanan = readInfraredDistance(1);

        if (bacaKiri > bacaKanan) {
          eksekusiBelokKanan90();
        } else if (bacaKanan > bacaKiri) {
          eksekusiBelokKiri90();
        } else {
          eksekusiMajuTerus();
        }

        setRgbledColor(0, 0, 0);
      }

      // --- KONDISI B: LOST LINE RECOVERY (dengan timeout) ---
      else if (bacaKiri < AMBANG_BATAS && bacaKanan < AMBANG_BATAS) {
        if (recoveryMulai == 0) {
          recoveryMulai = millis();
        }

        if (millis() - recoveryMulai > RECOVERY_TIMEOUT_MS) {
          // Garis tak ketemu >2 detik: berhenti + LED merah, tunggu operator.
          // Jangan berputar liar sampai baterai habis.
          berhenti();
          setRgbledColor(255, 0, 0);
        } else {
          setRgbledColor(255, 0, 255);
          int kecCariGaris = kecepatanDasarMin;

          if (lastError > 0) {
            setServoTurn(4, 0, kecCariGaris);
            setServoTurn(2, 0, kecCariGaris);
            setServoTurn(1, 0, kecCariGaris);
            setServoTurn(3, 0, kecCariGaris);
          } else {
            setServoTurn(4, 1, kecCariGaris);
            setServoTurn(2, 1, kecCariGaris);
            setServoTurn(1, 1, kecCariGaris);
            setServoTurn(3, 1, kecCariGaris);
          }
        }
      }

      // --- KONDISI C: MODE TRACER PID NORMAL DENGAN ADAPTIVE SPEED ---
      else {
        setRgbledColor(0, 0, 0);

        // PID jalan pada interval tetap agar D konsisten.
        // Di antara interval, pertahankan speed terakhir (tanpa hitung ulang).
        unsigned long nowPid = millis();
        if (nowPid - pidTerakhir < PID_INTERVAL_MS && recoveryMulai == 0) {
          jalankanMotorPID(prevKiri, prevKanan);
          return;
        }

        pidTerakhir = nowPid;

        // 1. Hitung Nilai Error (pakai 2.0 agar tidak terpotong integer)
        error = (bacaKiri - bacaKanan) / 2.0;

        // Reset state integral saat kembali dari recovery/persimpangan:
        // hutang koreksi lama tidak relevan dengan garis yang baru ditemukan.
        if (recoveryMulai != 0) {
          I = 0;
          lastError = error;  // cegah spike D di loop pertama
          recoveryMulai = 0;
        }

        // 2. Kalkulasi PID Standar
        P = error;
        I = I + error;
        I = constrain(I, -50, 50);
        D = error - lastError;

        PID_value = (Kp * P) + (Ki * I) + (Kd * D);
        lastError = error;

        // 3. --- LOGIKA KALIBRASI KECEPATAN DASAR BERBASIS PID ---
        // Semakin besar nilai PID_value (semakin melenceng), kecepatan dasar akan
        // semakin dikurangi. abs() digunakan agar nilai koreksi negatif/positif
        // tetap dihitung sebagai nilai mutlak pengurangan.
        int faktorPengurang =
          abs(PID_value) * 0.6;  // Pengali 0.6 adalah keagresifan pengereman (bisa di-tuning)

        kecepatanDasar = kecepatanDasarMaks - faktorPengurang;
        kecepatanDasar =
          constrain(kecepatanDasar, kecepatanDasarMin, kecepatanDasarMaks);

        // 4. Hitung Kecepatan Motor Akhir Menggunakan Kecepatan Dasar yang Sudah
        // Beradaptasi
        kecepatanKiri = kecepatanDasar - PID_value;
        kecepatanKanan = kecepatanDasar + PID_value;

        // 5. Batasi Output Akhir (derate servo 0-150, bukan 0-255 docs)
        kecepatanKiri = constrain(kecepatanKiri, 0, SERVO_MAKS_DIPAKAI);
        kecepatanKanan = constrain(kecepatanKanan, 0, SERVO_MAKS_DIPAKAI);

        // 6. Deadband compensation: servo diam di speed 1-24, paksa ke
        // SERVO_MIN_EFEKTIF agar koreksi PID benar-benar menggerakkan roda.
        kecepatanKiri = terapkanDeadband(kecepatanKiri);
        kecepatanKanan = terapkanDeadband(kecepatanKanan);

        // 7. Slew-rate limiter: tahan lompatan speed per loop agar roda
        // servo tidak slip (open-loop, tanpa encoder).
        kecepatanKiri = batasiSlew(kecepatanKiri, prevKiri);
        kecepatanKanan = batasiSlew(kecepatanKanan, prevKanan);
        prevKiri = kecepatanKiri;
        prevKanan = kecepatanKanan;

        jalankanMotorPID(kecepatanKiri, kecepatanKanan);
      }
    } else {
      berhenti();
      prevKiri = 0;
      prevKanan = 0;
      if (modeSetelKd == true) {
        setRgbledColor(255, 0, 255);
      } else {
        setRgbledColor(0, 0, 0);
      }
    }

    delay(2);
  }
}

// --- FUNGSI KALIBRASI SENSOR ---
void mulaiKalibrasiSensor() {
  Serial.println("--- MEMULAI KALIBRASI SENSOR ---");
  int nilaiMaks = 0;
  int nilaiMin = 20;

  setRgbledColor(255, 255, 0);
  delay(500);
  setServoTurn(4, 0, 50);
  setServoTurn(2, 0, 50);
  setServoTurn(1, 0, 50);
  setServoTurn(3, 0, 50);

  unsigned long waktuMulai = millis();
  while (millis() - waktuMulai < 3000) {
    int sensor1 = readInfraredDistance(1);
    int sensor2 = readInfraredDistance(2);
    if (sensor1 > nilaiMaks && sensor1 <= 20) {
      nilaiMaks = sensor1;
    }

    if (sensor2 > nilaiMaks && sensor2 <= 20) {
      nilaiMaks = sensor2;
    }

    if (sensor1 < nilaiMin && sensor1 > 0) {
      nilaiMin = sensor1;
    }

    if (sensor2 < nilaiMin && sensor2 > 0) {
      nilaiMin = sensor2;
    }

    delay(10);
  }

  berhenti();

  AMBANG_BATAS = (nilaiMaks + nilaiMin) / 2;
  AMBANG_BATAS = constrain(AMBANG_BATAS, 5, 14);
  simpanNilaiKeEEPROM();

  for (int i = 0; i < 3; i++) {
    setRgbledColor(0, 255, 0);
    delay(200);
    setRgbledColor(0, 0, 0);
    delay(200);
  }
}

// --- MANAJEMEN TOMBOL (non-blocking: tanpa delay, pakai cooldown) ---
void cekTombol() {
  unsigned long now = millis();
  int buttonState1 = readButtonValue(1);
  int buttonState2 = readButtonValue(2);

  // Feedback LED kilat tuning dihapus: loop() menimpa LED tiap iterasi,
  // Serial print tetap jadi umpan balik tuning.
  if (buttonState1 != 0 && now - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    if (buttonState1 == 1) {
      if (!modeSetelKd) {
        Kp += 1.0;
      } else {
        Kd += 0.5;
      }

      Serial.print(!modeSetelKd ? "Kp: " : "Kd: ");
      Serial.println(!modeSetelKd ? Kp : Kd);
    } else if (buttonState1 == 2) {
      if (!modeSetelKd) {
        Kp -= 1.0;
        if (Kp < 0) {
          Kp = 0;
        }
      } else {
        Kd -= 0.5;
        if (Kd < 0) {
          Kd = 0;
        }
      }

      Serial.print(!modeSetelKd ? "Kp: " : "Kd: ");
      Serial.println(!modeSetelKd ? Kp : Kd);
    } else if (buttonState1 == 3) {
      if (robotJalan) {
        robotJalan = false;
        resetStatePID();
        setEyelightLook(1, 0, 5, 254, 0, 0);
        setEyelightLook(2, 0, 5, 254, 0, 0);
      } else {
        modeSetelKd = !modeSetelKd;
        Serial.print("Mode setel Kd: ");
        Serial.println(modeSetelKd ? "YA" : "TIDAK");
      }
    }
  }

  if (buttonState2 != 0 && now - cooldownB2 >= COOLDOWN_TOMBOL_MS) {
    cooldownB2 = now;
    if (buttonState2 == 1) {
      kecepatanDasarMaks += 10;
      if (kecepatanDasarMaks > 150) {
        kecepatanDasarMaks = 150;
      }

      Serial.print("Target Speed Maks: ");
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 2) {
      kecepatanDasarMaks -= 10;
      if (kecepatanDasarMaks < 60) {
        kecepatanDasarMaks = 60;
      }

      Serial.print("Target Speed Maks: ");
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 3) {
      if (!robotJalan) {
        simpanNilaiKeEEPROM();
        resetStatePID();  // start bersih: tanpa hutang integral/derivatif run
                          // lama
        robotJalan = true;
        setEyelightLook(1, 0, 7, 0, 0, 254);
        setEyelightLook(2, 0, 7, 0, 0, 254);
      }
    }
  }
}

// --- LAMPU OTOMATIS: nyala putih saat gelap, mati saat terang ---
// Histeresis 2 ambang mencegah lampu kedip di batas terang/gelap.
// Catatan: saat menyala, ini menimpa ekspresi wajah setEyelightLook;
// wajah start/stop tetap terlihat saat ruangan terang.
void cekLampuGelap() {
  unsigned long now = millis();
  if (now - cekCahayaTerakhir < CEK_CAHAYA_MS) {
    return;
  }

  cekCahayaTerakhir = now;

  int lux = readLightValue(LIGHT_SENSOR_ID);
  if (!lampuGelapNyala && lux < AMBANG_GELAP_NYALA) {
    lampuGelapNyala = true;
    setEyelightAllPetals(EYELIGHT_KIRI, 255, 255, 255);
    setEyelightAllPetals(EYELIGHT_KANAN, 255, 255, 255);
  } else if (lampuGelapNyala && lux > AMBANG_GELAP_MATI) {
    lampuGelapNyala = false;
    setEyelightOff(EYELIGHT_KIRI);
    setEyelightOff(EYELIGHT_KANAN);
  }
}

// --- HALANGAN DEPAN: return true bila harus berhenti. Non-blocking. ---
bool cekHalanganDepan() {
  unsigned long now = millis();
  if (now - cekHalanganTerakhir >= CEK_HALANGAN_MS) {
    cekHalanganTerakhir = now;
    int jarak = readUltrasonicDistance(ULTRASONIC_ID);
    // 0 = bacaan tak valid (di luar jangkauan) -> abaikan, seperti contoh docs.
    bool baru = (jarak > 0 && jarak <= JARAK_BERHENTI_CM);
    if (baru != halanganDepan) {
      halanganDepan = baru;
      Serial.print("Halangan depan: ");
      Serial.println(halanganDepan ? "BERHENTI" : "JALAN");
    }
  }

  if (halanganDepan) {
    berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

// --- DRIVER PERGERAKAN ---
void majuManual(int speed, int durasi) {
  setServoTurn(4, 1, speed);
  setServoTurn(2, 1, speed);
  setServoTurn(1, 0, speed);
  setServoTurn(3, 0, speed);
  delay(durasi);
}

void eksekusiBelokKiri90() {
  setServoTurn(4, 0, kecepatanDasarMin);
  setServoTurn(2, 0, kecepatanDasarMin);
  setServoTurn(1, 0, kecepatanDasarMin);
  setServoTurn(3, 0, kecepatanDasarMin);
  delay(300);
  unsigned long t0 = millis();
  while (readInfraredDistance(2) < AMBANG_BATAS) {
    if (millis() - t0 > BELOK_TIMEOUT_MS) {
      break;  // garis tak ketemu: jangan macet, kembali ke loop utama
    }

    delay(1);
  }
}

void eksekusiBelokKanan90() {
  setServoTurn(4, 1, kecepatanDasarMin);
  setServoTurn(2, 1, kecepatanDasarMin);
  setServoTurn(1, 1, kecepatanDasarMin);
  setServoTurn(3, 1, kecepatanDasarMin);
  delay(300);
  unsigned long t0 = millis();
  while (readInfraredDistance(1) < AMBANG_BATAS) {
    if (millis() - t0 > BELOK_TIMEOUT_MS) {
      break;  // garis tak ketemu: jangan macet, kembali ke loop utama
    }

    delay(1);
  }
}

void eksekusiMajuTerus() {
  majuManual(kecepatanDasarMin, 100);
}

void jalankanMotorPID(int speedKiri, int speedKanan) {
  setServoTurn(4, 1, speedKiri);
  setServoTurn(2, 1, speedKiri);
  setServoTurn(1, 0, speedKanan);
  setServoTurn(3, 0, speedKanan);
}

// --- HELPER KHUSUS SERVO ---
// Filter eksponensial: redam 1 bacaan liar tanpa menambah delay.
int bacaSensorHalus(int id, float &filt) {
  int mentah = readInfraredDistance(id);
  if (filt < 0) {
    filt = mentah;
  } else {
    filt += ALPHA_SENSOR * (mentah - filt);
  }

  return (int)(filt + 0.5);
}

void resetStatePID() {
  error = 0;
  lastError = 0;
  P = 0;
  I = 0;
  D = 0;
  PID_value = 0;
  prevKiri = 0;
  prevKanan = 0;
  recoveryMulai = 0;
  filtKiri = -1;  // filter adaptasi ulang ke garis baru
  filtKanan = -1;
  pidTerakhir = 0;
}

int terapkanDeadband(int speed) {
  if (speed == 0) {
    return 0;
  }

  if (speed < SERVO_MIN_EFEKTIF) {
    return SERVO_MIN_EFEKTIF;
  }

  return speed;
}

int batasiSlew(int target, int prev) {
  int selisih = target - prev;
  if (selisih > SLEW_MAKS_PER_LOOP) {
    return prev + SLEW_MAKS_PER_LOOP;
  }

  if (selisih < -SLEW_MAKS_PER_LOOP) {
    return prev - SLEW_MAKS_PER_LOOP;
  }

  return target;
}

void berhenti() {
  setServoStop(4);
  setServoStop(2);
  setServoStop(1);
  setServoStop(3);
}

// --- EEPROM (hemat tulis: EEPROM ~100rb siklus, jangan tulis tiap boot
// bila nilainya sama) ---
template<typename T> void eepromPutHemat(int addr, const T &nilai) {
  T lama;
  EEPROM.get(addr, lama);
  if (memcmp(&lama, &nilai, sizeof(T)) != 0) {
    EEPROM.put(addr, nilai);
  }
}

void simpanNilaiKeEEPROM() {
  eepromPutHemat(ADDR_KP, Kp);
  eepromPutHemat(ADDR_KD, Kd);
  eepromPutHemat(ADDR_SPEED, kecepatanDasarMaks);
  eepromPutHemat(ADDR_AMBANG, AMBANG_BATAS);
  EEPROM.update(ADDR_EEPROM_CHECK, 123);
}

void bacaNilaiDariEEPROM() {
  if (EEPROM.read(ADDR_EEPROM_CHECK) == 123) {
    EEPROM.get(ADDR_KP, Kp);
    EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SPEED, kecepatanDasarMaks);
    EEPROM.get(ADDR_AMBANG, AMBANG_BATAS);
  }
}
