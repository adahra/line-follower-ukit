#include "ucode.h"
#include <EEPROM.h>
#include "config.h"     // seluruh nilai konfigurasi
#include "deklarasi.h"  // deklarasi maju + helper template EEPROM

// (Konfigurasi PID, speed, dan konstanta tune pindah ke config.h)

// --- STATE RUNTIME (bukan config; direset via resetStatePID) ---
unsigned long recoveryMulai = 0;  // 0 = tidak dalam mode recovery
float filtKiri = -1;              // filter sensor, <0 = belum diinisialisasi
float filtKanan = -1;
unsigned long pidTerakhir = 0;
unsigned long cooldownB1 = 0;
unsigned long cooldownB2 = 0;

// --- STATE LAMPU OTOMATIS ---
unsigned long cekCahayaTerakhir = 0;
bool lampuGelapNyala = false;

// --- STATE HALANGAN DEPAN ---
unsigned long cekHalanganTerakhir = 0;
bool halanganDepan = false;

// --- STATE TEPUK TANGAN ---
unsigned long cekSuaraTerakhir = 0;
unsigned long tepukTerakhir = 0;
bool tepukSiap = true; // true = menunggu lonjakan suara berikutnya

// --- STATE BATERAI ---
unsigned long cekBateraiTerakhir = 0;
bool bateraiLemah = false;

// --- STATE TELEMETRI ---
unsigned long telemTerakhir = 0;
bool telemHeader = false;

// --- STATE IMU ---
unsigned long imuTerakhir = 0;
float imuRoll = 0, imuPitch = 0, imuAccelMag = 0;

// --- STATE DETEKSI MACET ---
unsigned long irBekuSejak = 0;
int irRefKiri = 0, irRefKanan = 0;
float accelRef = -1; // <0 = belum diinisialisasi

// --- STATE MOTOR & STATUS ---
int prevKiri = 0;
int prevKanan = 0;

int kecepatanKiri = 0;
int kecepatanKanan = 0;

// Variabel internal PID
float error = 0, lastError = 0;
float P, I, D, PID_value;

bool robotJalan = false;
bool modeSetelKd = false;

void setup() {
  Initialization();
  IMU::init(); // gyro onboard untuk deteksi miring + macet
  if (protocolRunState == false) {
    bacaNilaiDariEEPROM();  // pakai nilai tersimpan; kalibrasi manual via 2 tombol
    cetakNilaiEEPROM();    // tampilkan semua nilai saat colok USB/hidup
    Serial.print("Baterai: ");
    Serial.print(readBatteryVoltage());
    Serial.println(" V");
    Serial.println("Setup selesai. Kalibrasi manual: tekan 2 tombol bersamaan.");
  }
}

void loop() {
  protocol();
  if (protocolRunState == false) {
    // Prioritas tertinggi: baterai lemah menghentikan SEMUA fungsi.
    if (cekBateraiLemah()) {
      return;
    }

    // Robot miring/terangkat: matikan motor selama tidak rata (tak dikunci).
    if (cekMiring()) {
      return;
    }

    cekTombol();
    cekTepukTangan();  // toggle jalan/berhenti via tepuk tangan
    cekLampuGelap();   // headlight otomatis, non-blocking (throttle 200ms)

    if (robotJalan == true) {
      // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
      if (cekHalanganDepan()) {
        return;
      }

      int bacaKiri = bacaSensorHalus(2, filtKiri);
      int bacaKanan = bacaSensorHalus(1, filtKanan);

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
      if (bacaKiri >= AMBANG_KIRI && bacaKanan >= AMBANG_KANAN) {
        setRgbledColor(255, 255, 255);
        majuManual(kecepatanDasarMin,
                   150);  // Lewati perempatan dengan kecepatan aman

        // Baca ulang setelah maju: keputusan belok harus pakai data segar
        // yang sudah difilter (bacaan mentah liar = risiko belok salah arah).
        bacaKiri = bacaSensorHalus(2, filtKiri);
        bacaKanan = bacaSensorHalus(1, filtKanan);

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
      else if (bacaKiri < AMBANG_KIRI && bacaKanan < AMBANG_KANAN) {
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

        // Nyangkut di pembatas: berhenti total, tunggu operator.
        if (cekMacet(bacaKiri, bacaKanan)) {
          return;
        }

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
        I = constrain(I, -BATAS_INTEGRAL, BATAS_INTEGRAL);
        D = error - lastError;

        PID_value = (Kp * P) + (Ki * I) + (Kd * D);
        lastError = error;

        // 3. --- LOGIKA KALIBRASI KECEPATAN DASAR BERBASIS PID ---
        // Semakin besar nilai PID_value (semakin melenceng), kecepatan dasar akan
        // semakin dikurangi. abs() digunakan agar nilai koreksi negatif/positif
        // tetap dihitung sebagai nilai mutlak pengurangan.
        int faktorPengurang =
          abs(PID_value) * FAKTOR_REM_ADAPTIF;  // agresivitas pengereman (bisa di-tuning)
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
        telemetri(bacaKiri, bacaKanan);
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
  int maksKiri = 0, minKiri = 20;
  int maksKanan = 0, minKanan = 20;

  setRgbledColor(255, 255, 0);
  delay(500);
  setServoTurn(4, 0, 50);
  setServoTurn(2, 0, 50);
  setServoTurn(1, 0, 50);
  setServoTurn(3, 0, 50);

  unsigned long waktuMulai = millis();
  while (millis() - waktuMulai < 3000) {
    int sensorKanan = readInfraredDistance(1);
    int sensorKiri = readInfraredDistance(2);
    if (sensorKanan > maksKanan && sensorKanan <= 20) {
      maksKanan = sensorKanan;
    }

    if (sensorKiri > maksKiri && sensorKiri <= 20) {
      maksKiri = sensorKiri;
    }

    if (sensorKanan < minKanan && sensorKanan > 0) {
      minKanan = sensorKanan;
    }

    if (sensorKiri < minKiri && sensorKiri > 0) {
      minKiri = sensorKiri;
    }

    delay(10);
  }

  berhenti();

  AMBANG_KANAN = constrain((maksKanan + minKanan) / 2, 5, 14);
  AMBANG_KIRI = constrain((maksKiri + minKiri) / 2, 5, 14);
  Serial.print("Ambang kiri: ");
  Serial.print(AMBANG_KIRI);
  Serial.print(" kanan: ");
  Serial.println(AMBANG_KANAN);
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

  // Kalibrasi manual: tekan 2 tombol bersamaan saat berhenti.
  // (Dulu kalibrasi otomatis tiap boot — dihapus agar restart cepat.)
  // Dicek dulu agar tak bentrok dengan aksi tombol tunggal di bawah.
  if (buttonState1 != 0 && buttonState2 != 0 && !robotJalan && now - cooldownB1 >= COOLDOWN_TOMBOL_MS && now - cooldownB2 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    cooldownB2 = now;
    Serial.println("Kalibrasi manual dimulai...");
    mulaiKalibrasiSensor();
    return;
  }

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
      if (kecepatanDasarMaks > SPEED_TOMBOL_MAKS) {
        kecepatanDasarMaks = SPEED_TOMBOL_MAKS;
      }

      Serial.print("Target Speed Maks: ");
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 2) {
      kecepatanDasarMaks -= 10;
      if (kecepatanDasarMaks < SPEED_TOMBOL_MIN) {
        kecepatanDasarMaks = SPEED_TOMBOL_MIN;
      }

      Serial.print("Target Speed Maks: ");
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 3) {
      if (!robotJalan) {
        simpanNilaiKeEEPROM();
        resetStatePID();  // start bersih: tanpa hutang integral/derivatif run lama
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

// --- TEPUK TANGAN: 1x tepuk = toggle jalan/berhenti. Non-blocking. ---
void cekTepukTangan() {
  unsigned long now = millis();
  if (now - cekSuaraTerakhir < CEK_SUARA_MS) {
    return;
  }

  cekSuaraTerakhir = now;

  int suara = readSoundValue(SOUND_SENSOR_ID);
  if (tepukSiap && suara >= TEPUK_AMBANG && now - tepukTerakhir >= TEPUK_COOLDOWN_MS) {
    tepukSiap = false;  // kunci sampai suara reda, agar 1 tepuk = 1 toggle
    tepukTerakhir = now;
    if (robotJalan) {
      robotJalan = false;
      resetStatePID();
      setEyelightLook(1, 0, 5, 254, 0, 0);
      setEyelightLook(2, 0, 5, 254, 0, 0);
      Serial.println("Tepuk: BERHENTI");
    } else {
      simpanNilaiKeEEPROM();
      resetStatePID();
      robotJalan = true;
      setEyelightLook(1, 0, 7, 0, 0, 254);
      setEyelightLook(2, 0, 7, 0, 0, 254);
      Serial.println("Tepuk: JALAN");
    }
  } else if (!tepukSiap && suara < TEPUK_LEPAS) {
    tepukSiap = true;  // suara reda, siap deteksi tepukan berikutnya
  }
}

// --- BATERAI LEMAH: return true bila semua fungsi harus berhenti. ---
// Sekali terkunci, hanya power cycle (restart) yang membuka. Non-blocking.
bool cekBateraiLemah() {
  unsigned long now = millis();
  if (!bateraiLemah && now - cekBateraiTerakhir >= CEK_BATERAI_MS) {
    cekBateraiTerakhir = now;
    float v = readBatteryVoltage();
    if (v <= TEGANGAN_MIN) {
      bateraiLemah = true;
      robotJalan = false;
      resetStatePID();
      berhenti();
      Serial.print("BATERAI LEMAH: ");
      Serial.print(v);
      Serial.println(" V - semua fungsi dihentikan. Restart robot.");
    }
  }

  if (bateraiLemah) {
    berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

// --- IMU: baca gyro+accel dengan throttle, simpan ke global ---
void bacaIMU() {
  unsigned long now = millis();
  if (now - imuTerakhir < CEK_IMU_MS) {
    return;
  }
  imuTerakhir = now;
  IMU::read();
  imuRoll = IMU::getRoll();
  imuPitch = IMU::getPitch();
  float ax = IMU::getRawAccelX();
  float ay = IMU::getRawAccelY();
  float az = IMU::getRawAccelZ();
  imuAccelMag = sqrt(ax * ax + ay * ay + az * az);
}

// --- Robot miring/terangkat: hentikan motor selama tak rata ---
// Tidak dikunci: jalan lagi sendiri saat kembali rata. Non-blocking.
bool cekMiring() {
  bacaIMU();
  if (fabs(imuRoll) > TILT_MAKS_DERAJAT || fabs(imuPitch) > TILT_MAKS_DERAJAT) {
    berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

// --- Robot macet: IR beku + accel stabil saat diperintah jalan ---
// Dinyatakan macet: berhenti total (robotJalan=false), operator restart
// via tombol/tepuk. Non-blocking.
bool cekMacet(int bacaKiri, int bacaKanan) {
  bacaIMU();
  unsigned long now = millis();
  bool irBeku = (abs(bacaKiri - irRefKiri) <= STUCK_TOLERANSI_IR) &&
                (abs(bacaKanan - irRefKanan) <= STUCK_TOLERANSI_IR);
  bool accelDiam =
      (accelRef < 0) ||
      (fabs(imuAccelMag - accelRef) / (accelRef + 0.001) < STUCK_TOLERANSI_ACCEL);
  if (irBeku && accelDiam) {
    if (irBekuSejak == 0) {
      irBekuSejak = now;
    } else if (now - irBekuSejak > STUCK_DIAM_MS) {
      robotJalan = false;
      resetStatePID();
      berhenti();
      setRgbledColor(255, 0, 0);
      Serial.println("MACET: sensor & gerak beku - berhenti total.");
      return true;
    }
  } else {
    irBekuSejak = 0;
    irRefKiri = bacaKiri;
    irRefKanan = bacaKanan;
    accelRef = imuAccelMag;
  }

  return false;
}

// --- TELEMETRI CSV: t,err,pid,dasar,kiri,kanan,senK,senN ---
// Throttle 100 ms, non-blocking. Tempel output ke spreadsheet atau
// buka di Serial Plotter untuk melihat osilasi saat tuning Kp/Kd.
void telemetri(int bacaKiri, int bacaKanan) {
  unsigned long now = millis();
  if (now - telemTerakhir < TELEM_MS) {
    return;
  }

  telemTerakhir = now;
  if (!telemHeader) {
    telemHeader = true;
    Serial.println("t,err,pid,dasar,kiri,kanan,senK,senN");
  }
  
  Serial.print(now);
  Serial.print(',');
  Serial.print(error);
  Serial.print(',');
  Serial.print(PID_value);
  Serial.print(',');
  Serial.print(kecepatanDasar);
  Serial.print(',');
  Serial.print(kecepatanKiri);
  Serial.print(',');
  Serial.print(kecepatanKanan);
  Serial.print(',');
  Serial.print(bacaKiri);
  Serial.print(',');
  Serial.println(bacaKanan);
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
  while (readInfraredDistance(2) < AMBANG_KIRI) {
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
  while (readInfraredDistance(1) < AMBANG_KANAN) {
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
  filtKiri = -1; // filter adaptasi ulang ke garis baru
  filtKanan = -1;
  pidTerakhir = 0;
  irBekuSejak = 0; // referensi macet adaptasi ulang
  accelRef = -1;
  telemHeader = false; // cetak header CSV lagi di run berikutnya
  telemTerakhir = 0;
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

void simpanNilaiKeEEPROM() {
  eepromPutHemat(ADDR_KP, Kp);
  eepromPutHemat(ADDR_KD, Kd);
  eepromPutHemat(ADDR_SPEED, kecepatanDasarMaks);
  eepromPutHemat(ADDR_AMBANG, AMBANG_KIRI);
  eepromPutHemat(ADDR_AMBANG_KANAN, AMBANG_KANAN);
  EEPROM.update(ADDR_EEPROM_CHECK, 123);
}

// --- Tampilkan seluruh nilai EEPROM + parameter jalan via Serial ---
void cetakNilaiEEPROM() {
  Serial.println("--- NILAI TERSIMPAN ---");
  Serial.print("EEPROM valid: ");
  Serial.println(EEPROM.read(ADDR_EEPROM_CHECK) == 123 ? "YA" : "TIDAK (pakai default kode)");
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.print(Ki);
  Serial.println(" (tidak disimpan)");
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Speed maks: ");
  Serial.println(kecepatanDasarMaks);
  Serial.print("Speed min: ");
  Serial.print(kecepatanDasarMin);
  Serial.println(" (tidak disimpan)");
  Serial.print("Ambang kiri: ");
  Serial.println(AMBANG_KIRI);
  Serial.print("Ambang kanan: ");
  Serial.println(AMBANG_KANAN);
  Serial.println("-----------------------");
}

void bacaNilaiDariEEPROM() {
  if (EEPROM.read(ADDR_EEPROM_CHECK) == 123) {
    EEPROM.get(ADDR_KP, Kp);
    EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SPEED, kecepatanDasarMaks);
    EEPROM.get(ADDR_AMBANG, AMBANG_KIRI);
    EEPROM.get(ADDR_AMBANG_KANAN, AMBANG_KANAN);
    // Kompatibel format lama (satu ambang): sampah EEPROM dijepit ke rentang valid.
    AMBANG_KIRI = constrain(AMBANG_KIRI, 5, 14);
    AMBANG_KANAN = constrain(AMBANG_KANAN, 5, 14);
  }
}
