#include "antarmuka.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"
#include "lap_timer.h"
#include "mapping.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include <string.h>
#include <ctype.h>

#if ENABLE_RC
#include "rc_ctrl.h"
// rc adalah global yang dideklarasikan di LineFollowerFin.ino (di dalam #if ENABLE_RC).
extern RcCtrl rc;
#endif

Antarmuka::Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am, LapTimer &lt, Mapper &mp)
    : status(st), sensor(sen), gerak(g), aman(am), lapTimer(lt), mapper(mp) {}

void Antarmuka::resetSemua() {
  gerak.reset();
  sensor.resetFilter();
  aman.resetDeteksi();
  status.recoveryMulai = 0;
}

void Antarmuka::mulai(const char *via) {
  sensor.simpan();
  resetSemua();  // start bersih: tanpa hutang integral/derivatif run lama
  // Jangan reset runStartTime di sini — recordEvent/checkPrediction auto-
  // baseline agar offset waypoint konsisten dengan elapsed (lihat mapping.cpp).
  status.jalan = true;
  // Hanya mode Learning/Racing yang membutuhkan baseline timer untuk prediksi.
  if (status.modeOperasi == MODE_LEARNING || status.modeOperasi == MODE_RACING) {
    mapper.mulaiRun();
  }
  setEyelightLook(1, 0, 7, 0, 0, 254);
  setEyelightLook(2, 0, 7, 0, 0, 254);
  Serial.print(via);
  Serial.println(": JALAN");
}

void Antarmuka::berhenti(const char *via) {
  status.jalan = false;
  resetSemua();
  setEyelightLook(1, 0, 5, 254, 0, 0);
  setEyelightLook(2, 0, 5, 254, 0, 0);
  Serial.print(via);
  Serial.println(F(": BERHENTI"));
}

void Antarmuka::cekTombol() {
  unsigned long now = millis();
  int buttonState1 = readButtonValue(1);
  int buttonState2 = readButtonValue(2);

  // --- COMBO B1+B2 DICEK DULU, sebelum handler tombol tunggal ---
  // Tanpa ini, branch tombol tunggal tetap menambah Kp / ganti mode tiap
  // 300 ms dan terus me-reset cooldownB1 sehingga alur "tekan lagi dalam
  // 2 detik untuk Auto-Tune" hampir mustahil tercapai.
  // Alur baru: tahan keduanya >= 2 detik = kalibrasi + auto-tune;
  // lepas sebelum 2 detik = kalibrasi sensor saja.
  if (!status.jalan && buttonState1 != 0 && buttonState2 != 0) {
    if (comboMulai == 0) {
      comboMulai = now;
      comboSelesai = false;
      Serial.println(F("Combo B1+B2: tahan 2 detik = Auto-Tune, lepas lebih cepat = kalibrasi saja."));
    } else if (!comboSelesai && now - comboMulai >= 2000) {
      comboSelesai = true;
      Serial.println(F(">>> Memulai Kalibrasi + Auto-Tuning PID <<<"));
      tuningSequence();
    }
    return;  // combo aktif: tombol tunggal tidak diproses
  }

  // Tombol dilepas sebelum 2 detik: kalibrasi sensor saja (sekali per combo).
  if (comboMulai != 0) {
    if (!comboSelesai && buttonState1 == 0 && buttonState2 == 0) {
      Serial.println(F(">>> Kalibrasi sensor..."));
      sensor.kalibrasi(gerak);
      setRgbledColor(255, 255, 0);  // kuning selesai kalibrasi
      Serial.println(F("Kalibrasi selesai. LED kuning."));
    }
    comboMulai = 0;
    comboSelesai = false;
    return;  // frame lepas: jangan langsung proses tombol tunggal
  }

  // --- Non-blocking tombol detector ---
  // Deteksi edge dan cooldown untuk tombol tunggal
  if (buttonState1 != 0 && now - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    if (buttonState1 == 1) {
      // Mode cycling hanya saat robot berhenti dan BUKAN mode setel Kd
      if (!status.jalan && !status.modeSetelKd) {
        // Pindah mode: Normal -> Learning -> Racing -> RC -> Normal
        if (status.modeOperasi == MODE_NORMAL) {
          status.modeOperasi = MODE_LEARNING;
          Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
          setRgbledColor(255, 255, 0); // Kuning = learning
        } else if (status.modeOperasi == MODE_LEARNING) {
          status.modeOperasi = MODE_RACING;
          Serial.println(F("Mode: RACING (Gunakan Peta)"));
          setRgbledColor(0, 255, 0); // Hijau = racing
        } else if (status.modeOperasi == MODE_RACING) {
          status.modeOperasi = MODE_RC;
          Serial.println(F("Mode: RC (Remote Control)"));
          setRgbledColor(0, 255, 255); // Cyan = RC
        } else if (status.modeOperasi == MODE_RC) {
          status.modeOperasi = MODE_NORMAL;
          Serial.println(F("Mode: NORMAL"));
          setRgbledColor(0, 0, 0); // Hitam = normal
        }
      } else if (!status.modeSetelKd) {
        Kp += 1.0;
      } else {
        Kd += 0.5;
      }
      // Label & nilai: tampilkan mode hanya saat berhenti dan BUKAN mode setel Kd.
      bool showMode = (!status.modeSetelKd && !status.jalan);
      Serial.print(showMode ? F("Mode: ") : (status.modeSetelKd ? F("Kd: ") : F("Kp: ")));
      if (showMode) {
        Serial.println(status.modeOperasi == MODE_NORMAL ? F("NORMAL")
                       : status.modeOperasi == MODE_LEARNING ? F("LEARNING")
                       : status.modeOperasi == MODE_RACING ? F("RACING")
                       : F("RC"));
      } else {
        Serial.println(status.modeSetelKd ? Kd : Kp);
      }
    } else if (buttonState1 == 2) {
      if (!status.modeSetelKd) {
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
      Serial.print(!status.modeSetelKd ? F("Kp: ") : F("Kd: "));
      Serial.println(!status.modeSetelKd ? Kp : Kd);
    } else if (buttonState1 == 3) {
      if (status.jalan) {
        berhenti("Tombol1");
      } else {
        status.modeSetelKd = !status.modeSetelKd;
        Serial.print(F("Mode setel Kd: "));
        Serial.println(status.modeSetelKd ? F("YA") : F("TIDAK"));
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
      Serial.print(F("Target Speed Maks: "));
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 2) {
      kecepatanDasarMaks -= 10;
      if (kecepatanDasarMaks < SPEED_TOMBOL_MIN) {
        kecepatanDasarMaks = SPEED_TOMBOL_MIN;
      }
      Serial.print(F("Target Speed Maks: "));
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 3) {
      if (!status.jalan) {
        mulai("Tombol2");
      }
    }
  }

  // --- Kalibrasi + Auto-Tuning via combo sudah ditangani di awal cekTombol ---
}

void Antarmuka::cekTepuk() {
  unsigned long now = millis();
  if (now - cekSuaraTerakhir < CEK_SUARA_MS) {
    return;
  }

  cekSuaraTerakhir = now;

  int suara = readSoundValue(SOUND_SENSOR_ID);
  if (tepukSiap && suara >= TEPUK_AMBANG &&
      now - tepukTerakhir >= TEPUK_COOLDOWN_MS) {
    tepukSiap = false;
    tepukTerakhir = now;
    if (status.jalan) {
      berhenti("Tepuk");
    } else {
      mulai("Tepuk");
    }
  } else if (!tepukSiap && suara < TEPUK_LEPAS) {
    tepukSiap = true;
  }
}

void Antarmuka::cekSerial() {
  if (!Serial.available()) {
    return;
  }
  static char buf[64];
  int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  if (len > 0) {
    buf[len] = '\0';
    // trim trailing whitespace
    while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' ')) {
      buf[--len] = '\0';
    }
    if (len > 0) {
      parseCommand(buf);
    }
  }
}

void Antarmuka::parseCommand(char *cmd) {
  // to lowercase
  for (char *p = cmd; *p; ++p) {
    *p = tolower(*p);
  }
  char *space = strchr(cmd, ' ');
  char *key = cmd;
  char *valStr = "";
  if (space) {
    *space = '\0';
    valStr = space + 1;
  }
  float val = atof(valStr);
  bool hasVal = (valStr[0] != '\0');

  if (strcmp(key, "kp") == 0) {
    if (hasVal) Kp = val;
    Serial.print(F("Kp = "));
    Serial.println(Kp);
  } else if (strcmp(key, "ki") == 0) {
    if (hasVal) Ki = val;
    Serial.print(F("Ki = "));
    Serial.println(Ki);
  } else if (strcmp(key, "kd") == 0) {
    if (hasVal) Kd = val;
    Serial.print(F("Kd = "));
    Serial.println(Kd);
  } else if (strcmp(key, "speed") == 0) {
    if (hasVal) kecepatanDasarMaks = constrain((int)val, SPEED_TOMBOL_MIN, SPEED_TOMBOL_MAKS);
    Serial.print(F("Speed maks = "));
    Serial.println(kecepatanDasarMaks);
  } else if (strcmp(key, "speedmin") == 0) {
    if (hasVal) kecepatanDasarMin = constrain((int)val, 10, SPEED_TOMBOL_MAKS);
    Serial.print(F("Speed min = "));
    Serial.println(kecepatanDasarMin);
  } else if (strcmp(key, "ambang") == 0) {
    if (hasVal) {
      int v = constrain((int)val, 1, 20);
      AMBANG_KIRI = v;
      AMBANG_KANAN = v;
      Serial.print(F("Ambang kiri/kanan = "));
      Serial.println(v);
    } else {
      Serial.print(F("Ambang kiri = "));
      Serial.print(AMBANG_KIRI);
      Serial.print(F(" kanan = "));
      Serial.println(AMBANG_KANAN);
    }
  } else if (strcmp(key, "ambangk") == 0) {
    if (hasVal) AMBANG_KIRI = constrain((int)val, 1, 20);
    Serial.print(F("Ambang kiri = "));
    Serial.println(AMBANG_KIRI);
  } else if (strcmp(key, "ambangn") == 0) {
    if (hasVal) AMBANG_KANAN = constrain((int)val, 1, 20);
    Serial.print(F("Ambang kanan = "));
    Serial.println(AMBANG_KANAN);
  } else if (strcmp(key, "calibrate") == 0) {
    if (!status.jalan) {
      Serial.println(F("Kalibrasi dimulai via serial..."));
      sensor.kalibrasi(gerak);
    } else {
      Serial.println(F("ERROR: Robot jalan, hentikan dulu."));
    }
  } else if (strcmp(key, "tune") == 0) {
    if (!status.jalan) {
      Serial.println(F("Memulai Kalibrasi + Auto-Tuning PID..."));
      tuningSequence();
    } else {
      Serial.println(F("ERROR: Robot jalan, hentikan dulu."));
    }
  } else if (strcmp(key, "lap") == 0) {
    if (strcmp(valStr, "reset") == 0) {
      lapTimer.reset();
      Serial.println(F("Lap timer direset."));
    } else if (strcmp(valStr, "summary") == 0) {
      lapTimer.printSummary();
    } else if (strcmp(valStr, "csv") == 0) {
      lapTimer.printCSV();
    } else {
      // Default: tampilkan summary
      lapTimer.printSummary();
    }
  } else if (strcmp(key, "lapconfig") == 0) {
    if (hasVal) {
      // Format: lapconfig <threshold> <minLapMs>
      // Untuk simplicity, parse manual
      int threshold = atoi(valStr);
      char* space = strchr(valStr, ' ');
      if (space) {
        int minLap = atoi(space + 1);
        lapTimer.setThresholds(threshold, minLap);
      } else {
        lapTimer.setThresholds(threshold, LAP_MIN_LAP_MS);
      }
    } else {
      Serial.print(F("LapConfig: threshold=")); Serial.print(LAP_SF_THRESHOLD);
      Serial.print(F(" minLap=")); Serial.println(LAP_MIN_LAP_MS);
    }
  } else if (strcmp(key, "save") == 0) {
    sensor.simpan();
    mapper.simpan();  // peta waypoint learning ikut persisten
    mapper.debugPrint();
    Serial.println(F("Disimpan ke EEPROM."));
  } else if (strcmp(key, "load") == 0) {
    sensor.muat();
    mapper.muat();
    mapper.debugPrint();
    sensor.cetak();
    Serial.print(F("Waypoint tersimpan: "));
    Serial.println(mapper.getWaypointCount());
  } else if (strcmp(key, "mode") == 0) {
    if (status.jalan) {
      Serial.println(F("ERROR: Tidak dapat mengubah mode saat robot sedang jalan. Berhentikan dulu."));
    } else if (strcmp(valStr, "learn") == 0) {
      status.modeOperasi = MODE_LEARNING;
      Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
    } else if (strcmp(valStr, "race") == 0) {
      status.modeOperasi = MODE_RACING;
      Serial.println(F("Mode: RACING (Gunakan Peta)"));
    } else if (strcmp(valStr, "rc") == 0) {
      status.modeOperasi = MODE_RC;
      Serial.println(F("Mode: RC (Remote Control)"));
    } else {
      status.modeOperasi = MODE_NORMAL;
      Serial.println(F("Mode: NORMAL"));
    }
  } else if (strcmp(key, "rc") == 0) {
    if (strcmp(valStr, "on") == 0) {
      status.modeOperasi = MODE_RC;
      Serial.println(F("RC mode: ON (MODE_RC)"));
    } else if (strcmp(valStr, "off") == 0) {
      status.modeOperasi = MODE_NORMAL;
      Serial.println(F("RC mode: OFF (kembali ke MODE_NORMAL)"));
    } else if (strcmp(valStr, "status") == 0) {
#if ENABLE_RC
      Serial.print(F("RC Throttle: ")); Serial.print(rc.getThrottle());
      Serial.print(F("% | Steering: ")); Serial.print(rc.getSteering());
      Serial.print(F("% | Signal: "));
      Serial.println(rc.isSignalLost() ? F("HILANG") : F("OK"));
#else
      Serial.println(F("RC tidak aktif (ENABLE_RC=false)."));
#endif
    } else {
      Serial.println(F("Penggunaan: rc on | rc off | rc status"));
    }
  } else if (strcmp(key, "mapclear") == 0) {
    if (status.jalan) {
      Serial.println(F("ERROR: Robot sedang jalan, hentikan dulu sebelum mapclear."));
    } else {
      mapper.reset();
      Serial.println(F("Peta waypoint direset (mapclear)."));
    }
  } else if (strcmp(key, "path") == 0) {
#if ENABLE_PATH_PLANNER
    pathPlanner.handleCommand(valStr);
#else
    Serial.println(F("Path planner dinonaktifkan (ENABLE_PATH_PLANNER=false)."));
#endif
  } else if (strcmp(key, "start") == 0) {
    if (!status.jalan) {
      mulai("Serial");
    } else {
      Serial.println(F("Sudah jalan."));
    }
  } else if (strcmp(key, "stop") == 0) {
    if (status.jalan) {
      berhenti("Serial");
    } else {
      Serial.println(F("Sudah berhenti."));
    }
  } else if (strcmp(key, "status") == 0) {
    tampilkanStatusJalannya();
    Serial.println(F("ki <val>        : set Ki (contoh: ki 0.02)"));
    Serial.println(F("kd <val>        : set Kd (contoh: kd 1.8)"));
    Serial.println(F("speed <val>     : set speed maks 60-150"));
    Serial.println(F("speedmin <val>  : set speed min"));
    Serial.println(F("ambang <val>    : set ambang kiri & kanan (1-20)"));
    Serial.println(F("ambangk <val>   : set ambang kiri saja"));
    Serial.println(F("ambangn <val>   : set ambang kanan saja"));
    Serial.println(F("mode <val>      : set mode (learn/race/rc/normal)"));
    Serial.println(F("rc on|off|status: aktifkan/nonaktifkan/cek status RC"));
    Serial.println(F("calibrate       : jalankan kalibrasi sensor (harus berhenti)"));
    Serial.println(F("tune            : kalibrasi + Auto-Tuning PID sekaligus"));
    Serial.println(F("lap [summary|csv|reset] : lap timer summary/CSV/reset"));
    Serial.println(F("lapconfig <th> [minMs] : set threshold start/finish & min lap time"));
#if ENABLE_PATH_PLANNER
    Serial.println(F("path map        : tampilkan grid map"));
    Serial.println(F("path find       : cari path ke goal"));
    Serial.println(F("path goal <x> <y> : set goal position"));
    Serial.println(F("path next       : waypoint berikutnya"));
    Serial.println(F("path obstacle <x> <y> : tambah obstacle"));
    Serial.println(F("path reset      : reset path planner"));
#endif
    Serial.println(F("save            : simpan ke EEPROM (termasuk waypoint learning)"));
    Serial.println(F("load            : muat dari EEPROM & cetak waypoint"));
    Serial.println(F("start           : mulai jalan"));
    Serial.println(F("stop            : berhenti"));
    Serial.println(F("status          : tampilkan status sedang ini"));
    Serial.println(F("mapclear        : hapus semua waypoint peta (harus berhenti)"));
    Serial.println(F("help / ?        : tampilkan ini"));
    Serial.println(F("-----------------------"));
    Serial.println();
    Serial.println(F("=== MODE via TOMBOL (saat berhenti) ==="));
    Serial.println(F("B1 pendek: Pindah mode Normal <-> Learning <-> Racing <-> RC"));
    Serial.println(F("B1 lama: Toggle mode Setel Kp/Kd"));
    Serial.println(F("B1+B2: Kalibrasi sensor (pendek) / Auto-Tuning (lama)"));
  } else {
    Serial.print(F("Perintah tidak dikenal: "));
    Serial.println(cmd);
    Serial.println(F("Ketik 'help' untuk daftar perintah."));
  }
}

// Fungsi tampilkan status jalanannya ke serial
void Antarmuka::tampilkanStatusJalannya() {
  Serial.println(F("--- STATUS SEKARANG ---"));
  Serial.print(F("Jalan: ")); Serial.println(status.jalan ? F("YA") : F("TIDAK"));
  Serial.print(F("Mode operasi: "));
  switch (status.modeOperasi) {
    case 0: Serial.println(F("NORMAL")); break;
    case 1: Serial.println(F("LEARNING")); break;
    case 2: Serial.println(F("RACING")); break;
    case 3: Serial.println(F("RC")); break;
  }
  Serial.print(F("modeSetelKd: ")); Serial.println(status.modeSetelKd ? F("YA") : F("TIDAK"));
  Serial.print(F("recoveryMulai: ")); Serial.print(status.recoveryMulai); Serial.println(F(" ms"));
  Serial.print(F("Jumlah waypoint: ")); Serial.println(mapper.getWaypointCount());
  
  // Sensor status
  Serial.print(F("Sensor kiri: ")); Serial.print(sensor.bacaKiri());
  Serial.print(F(" (ambang: ")); Serial.print(AMBANG_KIRI); Serial.print(F(")"));
  Serial.print(F(" | Sensor kanan: ")); Serial.print(sensor.bacaKanan());
  Serial.print(F(" (ambang: ")); Serial.print(AMBANG_KANAN); Serial.println(F(")"));
  
  // PID & Speed
  Serial.print(F("Kp: ")); Serial.print(Kp);
  Serial.print(F(" | Kd: ")); Serial.print(Kd);
  Serial.print(F(" | Ki: ")); Serial.print(Ki);
  Serial.print(F(" | SpeedMaks: ")); Serial.print(kecepatanDasarMaks);
  Serial.print(F(" | Dasar: ")); Serial.print(kecepatanDasar);
  
  // LED status
  Serial.println(F(" --- AKHIR STATUS ---"));
}

// --- SEQUENCE: Kalibrasi + Auto-Tuning PID ---
// Alur: Kalibrasi sensor -> Test respons PID -> Fine-tuning -> Simpan ke EEPROM
void Antarmuka::tuningSequence() {
  unsigned long sequenceStart = millis();
  const unsigned long SEQ_TIMEOUT = 30000; // 30 detik total timeout
  
  Serial.println(F("--- MEMULAI SEQUENCE KALIBRASI + AUTO-TUNING ---"));
  
  // Step 1: Kalibrasi sensor
  setRgbledColor(255, 165, 0); // oranye = tahap kalibrasi
  Serial.println(F("Step 1: Kalibrasi sensor..."));
  sensor.kalibrasi(gerak);
  Serial.println(F("Kalibrasi selesai."));
  
  // Step 2: Auto-Tuning PID sederhana (grid search)
  setRgbledColor(255, 0, 255); // magenta = tahap auto-tuning
  Serial.println(F("Step 2: Auto-Tuning PID (grid search)..."));
  
  // Parameter grid untuk dicoba
  float kpValues[] = {15.0, 20.0, 25.0, 30.0};
  float kdValues[] = {0.5, 1.0, 1.5, 2.0};
  
  float bestKp = Kp;
  float bestKd = Kd;
  float bestScore = 9999.0; // Score lebih rendah = lebih baik
  
  // Simpan parameter lama
  float originalKp = Kp;
  float originalKd = Kd;
  
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // Cek timeout
      if (millis() - sequenceStart > SEQ_TIMEOUT) {
        Serial.println(F("Timeout sequence!"));
        break;
      }
      
      float testKp = kpValues[i];
      float testKd = kdValues[j];
      
      // Terapkan parameter uji
      Kp = testKp;
      Kd = testKd;
      Ki = 0.01; // Ki tetap kecil
      
      // Jalankan robot sebentar untuk test respons
      // Mulai test dari state PID/motor bersih agar skor antar kombinasi
      // sebanding dan turbo/slew run sebelumnya tidak ikut.
      gerak.reset();

      status.jalan = true;
      unsigned long testStart = millis();
      const unsigned long TEST_DURATION = 5000; // 5 detik test
      unsigned long garisHilangMulai = 0;
      
      float totalError = 0;
      int errorSamples = 0;
      bool oscillating = false;
      float prevError = 0;
      bool punyaSample = false;
      
      while (millis() - testStart < TEST_DURATION) {
        if (!status.jalan) break;
        
        // Safety tetap berjalan selama robot digerakkan auto-tune:
        // halangan/tilt/baterai lemah menghentikan test ini seketika.
        if (aman.bateraiKritis() || aman.miring() || aman.halangan()) {
          break;
        }
        
        // Baca sensor dan SICALAKAN PID sungguhan lewat gerak.hitung()
        // (sebelumnya robot tidak pernah digerakkan -> skor tidak bermakna).
        int bacaKiri = sensor.bacaKiri();
        int bacaKanan = sensor.bacaKanan();
        float error = (bacaKiri - bacaKanan) / 2.0;
        
        // Garis hilang terlalu lama: hentikan test, kombinasi ini buruk.
        if (bacaKiri < AMBANG_KIRI && bacaKanan < AMBANG_KANAN) {
          if (garisHilangMulai == 0) garisHilangMulai = millis();
          if (millis() - garisHilangMulai > RECOVERY_TIMEOUT_MS) break;
        } else {
          garisHilangMulai = 0;
        }
        
        gerak.hitung(bacaKiri, bacaKanan, true);
        
        totalError += abs(error);
        errorSamples++;
        
        // Deteksi oscillasi: bandingkan SEBELUM update prevError
        // (sebelumnya dibandingkan dengan diri sendiri -> selalu false).
        if (punyaSample && ((error > 0 && prevError < 0) || (error < 0 && prevError > 0))) {
          oscillating = true;
        }
        prevError = error;
        punyaSample = true;
        
        delay(10); // Approximate PID interval
      }
      
      status.jalan = false;
      gerak.saatBerhenti();  // motor berhenti + memori slew dibuang
      
      // Hitung skor: error rata-rata + penalty oscillasi
      float avgError = errorSamples > 0 ? totalError / errorSamples : 9999;
      float score = avgError;
      if (oscillating) score += 5.0; // Penalty besar buat oscillasi
      
      Serial.print(F("  Test Kp=")); Serial.print(testKp);
      Serial.print(F(", Kd=")); Serial.print(testKd);
      Serial.print(F(": avgError=")); Serial.print(avgError, 1);
      Serial.print(F(", oscill=")); Serial.print(oscillating ? "YA" : "TIDAK");
      Serial.println(F(", skor=")); Serial.print(score, 1);
      
      if (score < bestScore) {
        bestScore = score;
        bestKp = testKp;
        bestKd = testKd;
      }
    }
  }
  
  // Step 3: Terapkan parameter terbaik
  Serial.println(F("Step 3: Parameter terbaik..."));
  Kp = bestKp;
  Kd = bestKd;
  Serial.print(F("Kp terbaik: ")); Serial.println(Kp);
  Serial.print(F("Kd terbaik: ")); Serial.println(Kd);
  
  // Step 4: Simpan ke EEPROM
  setRgbledColor(0, 255, 0); // hijau = selesai
  gerak.saatBerhenti();
  sensor.simpan(); // Simpan Kp, Kd, ambang, speed ke EEPROM
  mapper.simpan();  // peta waypoint ikut tersimpan
  
  Serial.println(F("--- SEQUENCE SELESAI ---"));
  Serial.print(F("Parameter disimpan: Kp=")); Serial.print(Kp);
  Serial.print(F(", Kd=")); Serial.print(Kd);
  Serial.println(F(", ambang & speed ikut kalibrasi"));
  Serial.println(F("LED hijau = siap digunakan"));
  
  // Kembali ke mode normal
  status.modeOperasi = MODE_NORMAL;
  status.jalan = false;
  setRgbledColor(0, 0, 0);
}
