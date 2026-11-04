#include "ucode.h"
#include <EEPROM.h>
#include "fwd.h"
#include "config.h"
#include "status.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"
#include "antarmuka.h"
#include "mapping.h"
#include "lap_timer.h"
#if ENABLE_BLUETOOTH
#include "bluetooth_ctrl.h"
#endif
#if ENABLE_RC
#include "rc_ctrl.h"
#endif
#if ENABLE_PATH_PLANNER
#include "path_planner.h"  // Optional: enable via config.h + move optional/path_planner/ to sketch folder
#endif
// ucode.h included HERE only (single definition). Other files use ukit_wrapper.h

// Wiring antar modul via referensi (lihat header masing-masing).
Status status;
SensorGaris sensor;
Penggerak gerak;
Pengaman aman(status, gerak, sensor);
Mapper mapper;
LapTimer lapTimer;
#if ENABLE_PATH_PLANNER
PathPlanner pathPlanner;  // Optional: uncomment + include path_planner.h when needed
#endif
Antarmuka ui(status, sensor, gerak, aman, lapTimer, mapper);
#if ENABLE_BLUETOOTH
BluetoothCtrl bluetooth(status, gerak, sensor, aman, lapTimer);
#endif
#if ENABLE_RC
RcCtrl rc(status, gerak, sensor, aman, lapTimer);
#endif

void setup() {
  // Lindungi konfigurasi EEPROM 0..20: Initialization() library uKit
  // menghapus alamat itu bila byte 3 != 77. Guard harus ditulis SEBELUM
  // Initialization() agar tuning yang sudah disimpan tidak hilang.
  EEPROM.update(3, 77);
  Initialization();
  IMU::init();
  if (protocolRunState == false) {
    sensor.muat();
    mapper.muat();  // peta waypoint LEARNING persisten antar power cycle
    bool eepromValid = (EEPROM.read(ADDR_EEPROM_CHECK) == 123);
    if (!eepromValid && AUTO_CALIBRATE_ON_BOOT) {
      Serial.println(F("EEPROM invalid + AUTO_CALIBRATE_ON_BOOT=true -> kalibrasi otomatis..."));
      sensor.kalibrasi(gerak);
    } else {
      sensor.cetak();
      Serial.print(F("Baterai: "));
      Serial.print(aman.tegangan());
      Serial.println(F(" V"));
      Serial.println(F("Setup selesai. Kalibrasi manual: tekan 2 tombol bersamaan."));
    }
  }
  
  // Inisialisasi Lap Timer
  lapTimer.begin();
  lapTimer.setThresholds(LAP_SF_THRESHOLD, LAP_MIN_LAP_MS);

  // Inisialisasi Adaptive Threshold
#if ENABLE_ADAPTIVE_THRESHOLD
  sensor.setAdaptiveThreshold(true);
  sensor.muatAdaptive();
#endif

  // Inisialisasi Heading Correction
#if ENABLE_HEADING_CORRECTION
  gerak.setHeadingCorrection(true);
  gerak.setHeadingTarget(HEADING_TARGET_DEFAULT);
  gerak.muatHeading();
#endif

  // Inisialisasi Gain Scheduling
#if ENABLE_GAIN_SCHEDULING
  gerak.setGainScheduling(true);
#endif

  // Inisialisasi Bluetooth
  #if ENABLE_BLUETOOTH
    bluetooth.begin(BT_BAUD, (BTModuleType)BT_MODULE_TYPE);
    if (bluetooth.isConnected()) {
      bluetooth.setAutoTelemetry(true, BT_TELEMETRY_MS);
      Serial.println(F("Bluetooth: Ready"));
    } else {
      Serial.println(F("Bluetooth: Not connected (check wiring)"));
    }
  #endif

  // Inisialisasi Path Planner
  #if ENABLE_PATH_PLANNER
    pathPlanner.begin(PATH_GRID_WIDTH / 2, PATH_GRID_HEIGHT / 2);
    Serial.println(F("Path Planner: Ready (mode peta manual, bukan navigasi otonom)"));
  #endif

  // Inisialisasi RC
  #if ENABLE_RC
    rc.begin(RC_BAUD, RC_PIN, RC_PWM, RC_STEER_PIN);
    Serial.println(F("RC: Ready"));
  #endif
}

void loop() {
  // Prediksi RACING terakhir. Disimpan antar-iterasi karena checkPrediction()
  // hanya "fire" sekali per waypoint; aksi harus tetap tersedia sampai
  // persimpangan benar-benar tiba.
  static ActionType aksiPrediksi = ACT_NONE;
  static unsigned long prediksiMulai = 0;

  protocol();
  if (!status.jalan || status.modeOperasi != MODE_RACING) {
    aksiPrediksi = ACT_NONE;
  }
  if (protocolRunState == false) {
    // Prioritas tertinggi: baterai lemah menghentikan SEMUA fungsi.
    if (aman.bateraiKritis()) {
      return;
    }

    // Robot miring/terangkat: matikan motor selama tidak rata (tak dikunci).
    if (aman.miring()) {
      return;
    }

    ui.cekTombol();
    ui.cekTepuk();          // toggle jalan/berhenti via tepuk tangan
#if ENABLE_SERIAL_MODE
    ui.cekSerial();         // serial command parser (Kp/Ki/Kd/speed/ambang/calibrate/save/help)
#endif
    aman.lampuOtomatis();   // headlight otomatis, non-blocking (throttle 200ms)

    // IMU heading: integrasi gyro tiap loop, termasuk saat berhenti agar
    // kalibrasi bias (perintah 'heading cal') tetap berjalan tanpa PID.
    #if ENABLE_HEADING_CORRECTION
      gerak.updateHeading();
    #endif
    
    // Bluetooth update
    #if ENABLE_BLUETOOTH
      bluetooth.update();
    #endif

    // RC update
    #if ENABLE_RC
      rc.update();
    #endif

    // Path Planner TIDAK dipanggil otomatis. Fitur ini adalah alat peta/A*
    // manual lewat perintah serial ('path ...'), bukan navigasi otonom:
    // posisi grid robot tidak bisa diketahui tanpa odometri/encoder.
    // Memanggil updateMap() tiap loop juga membaca sensor dua kali dan
    // menambah ping ultrasonik tanpa manfaat untuk pengendalian.

    if (status.jalan == true) {
      // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
      if (aman.halangan()) {
        return;
      }

      int bacaKiri = sensor.bacaKiri();
      int bacaKanan = sensor.bacaKanan();

      // Ambang efektif: gunakan ambang dinamis jika adaptive threshold aktif.
#if ENABLE_ADAPTIVE_THRESHOLD
      int ambangKiriEfektif = sensor.isAdaptiveThresholdEnabled()
                              ? sensor.ambangDinamis(2) : AMBANG_KIRI;
      int ambangKananEfektif = sensor.isAdaptiveThresholdEnabled()
                               ? sensor.ambangDinamis(1) : AMBANG_KANAN;
#else
      int ambangKiriEfektif = AMBANG_KIRI;
      int ambangKananEfektif = AMBANG_KANAN;
#endif

      // --- RACING PREDICTION ---
      bool prediksiValid = false;
      if (status.modeOperasi == MODE_RACING) {
        if (mapper.getWaypointCount() == 0) {
          // Tidak ada waypoint terrekam: lewati prediksi (tanpa peta).
          aksiPrediksi = ACT_NONE;
        } else {
          EventType predType;
          ActionType predAct;
          if (mapper.checkPrediction(millis(), predType, predAct)) {
            aksiPrediksi = predAct;
            prediksiMulai = millis();
          }
        }

        prediksiValid = (aksiPrediksi != ACT_NONE) &&
                        (millis() - prediksiMulai <= RACE_PREDICTION_HOLD_MS);
        if (!prediksiValid) {
          aksiPrediksi = ACT_NONE;
        }

        // LED menampilkan prediksi yang sedang berlaku (bukan hanya saat
        // checkPrediction baru fire), supaya operator bisa melihat konteks.
        if (!prediksiValid) {
          setRgbledColor(0, 0, 255); // Biru = tidak ada event prediction
        } else if (aksiPrediksi == ACT_STRAIGHT) {
          setRgbledColor(255, 255, 255); // Putih = lurus
        } else if (aksiPrediksi == ACT_LEFT) {
          setRgbledColor(0, 255, 0); // Hijau = belok kiri
        } else if (aksiPrediksi == ACT_RIGHT) {
          setRgbledColor(255, 0, 0); // Merah = belok kanan
        }
      }

      // --- MODE RC: Remote Control menggantikan semua logika tracer ---
#if ENABLE_RC
        if (status.modeOperasi == MODE_RC) {
          if (rc.isSignalLost()) {
            gerak.berhenti();
            setRgbledColor(255, 0, 0);  // Merah = sinyal hilang
          } else {
            int speedKiri, speedKanan;
            rc.getSpeeds(speedKiri, speedKanan);
            gerak.jalankanArah(speedKiri, speedKanan);
            setRgbledColor(0, 255, 255);  // Cyan = RC aktif
          }
          return;
        }
#endif

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
      if (bacaKiri >= ambangKiriEfektif && bacaKanan >= ambangKananEfektif) {
        setRgbledColor(255, 255, 255);
        gerak.majuManual(kecepatanDasarMin,
                         150);  // Lewati perempatan dengan kecepatan aman

        // Baca ulang setelah maju: keputusan belok harus pakai data segar
        // yang sudah difilter (bacaan mentah liar = risiko belok salah arah).
        bacaKiri = sensor.bacaKiri();
        bacaKanan = sensor.bacaKanan();

        ActionType act = ACT_STRAIGHT;
        int selisih = bacaKiri - bacaKanan;
        bool pakaiPrediksi = (status.modeOperasi == MODE_RACING) &&
                             prediksiValid &&
                             (abs(selisih) < RACE_PREDICTION_MARGIN);

        if (pakaiPrediksi) {
          // Sensor ambigu: pakai aksi hasil Learning. Ini yang membuat mode
          // RACING benar-benar berpengaruh, bukan sekadar lampu LED.
          act = aksiPrediksi;
          if (act == ACT_RIGHT) {
            gerak.belokKanan90(AMBANG_KANAN);
          } else if (act == ACT_LEFT) {
            gerak.belokKiri90(AMBANG_KIRI);
          } else {
            gerak.majuTerus();
          }
          aksiPrediksi = ACT_NONE;  // sekali pakai per persimpangan
        } else if (selisih > 0) {
          act = ACT_RIGHT;
          gerak.belokKanan90(AMBANG_KANAN);
        } else if (selisih < 0) {
          act = ACT_LEFT;
          gerak.belokKiri90(AMBANG_KIRI);
        } else {
          gerak.majuTerus();
        }

        // Belok 90° bersifat open-loop: jadikan arah setelah belok sebagai
        // baseline heading baru agar koreksi tidak melawan arah lintasan.
        #if ENABLE_HEADING_CORRECTION
          gerak.resetHeading();
        #endif

        // Rekam event jika dalam mode learning
        if (status.modeOperasi == MODE_LEARNING) {
          mapper.recordEvent(EVENT_INTERSECTION, act);
          Serial.print(F("Event tercatat: "));
          Serial.println(act);
        }

        setRgbledColor(0, 0, 0);
      }

      // --- KONDISI B: LOST LINE RECOVERY (dengan timeout) ---
      else if (bacaKiri < ambangKiriEfektif && bacaKanan < ambangKananEfektif) {
        if (status.recoveryMulai == 0) {
          status.recoveryMulai = millis();
        }

        if (millis() - status.recoveryMulai > RECOVERY_TIMEOUT_MS) {
          // Garis tak ketemu >2 detik: berhenti + LED merah, tunggu operator.
          // Jangan berputar liar sampai baterai habis.
          gerak.berhenti();
          setRgbledColor(255, 0, 0);
        } else {
          setRgbledColor(255, 0, 255);
          gerak.putarCariGaris(gerak.errorTerakhir());
        }
      }

      // --- KONDISI C: MODE TRACER PID NORMAL DENGAN ADAPTIVE SPEED ---
      else {
        setRgbledColor(0, 0, 0);

        // Nyangkut di pembatas: berhenti total, tunggu operator.
        if (aman.macet(bacaKiri, bacaKanan)) {
          return;
        }

        // Reset state integral saat kembali dari recovery: hutang koreksi
        // lama tidak relevan dengan garis yang baru ditemukan.
        // (paksa = hitung PID sekarang juga, jangan tunggu interval.)
        bool dariRecovery = (status.recoveryMulai != 0);
        status.recoveryMulai = 0;
        if (dariRecovery) {
          gerak.resetIntegralSetelahRecovery((bacaKiri - bacaKanan) / 2.0);
          // Robot berputar saat recovery: arah sekarang = baseline baru.
          #if ENABLE_HEADING_CORRECTION
            gerak.resetHeading();
          #endif
        }

        // Di antara interval PID, pertahankan speed terakhir.
        if (!gerak.hitung(bacaKiri, bacaKanan, dariRecovery)) {
          gerak.terapkanTerakhir();
          return;
        }

        // --- ADAPTIVE THRESHOLD UPDATE ---
        // Ambang dinamis di-update setiap PID interval berdasarkan kecepatan.
#if ENABLE_ADAPTIVE_THRESHOLD
        if (sensor.isAdaptiveThresholdEnabled()) {
          sensor.updateAmbangDinamis(bacaKiri, bacaKanan, kecepatanDasar);
        }
#endif

        // --- LAP TIMER UPDATE ---
        // Hanya update saat PID interval tiba (gerak.hitung return true)
        if (ENABLE_LAP_TIMER) {
          float currentError = (bacaKiri - bacaKanan) / 2.0;
          lapTimer.update(bacaKiri, bacaKanan, kecepatanDasar, currentError);
        }
      }
    } else {
      gerak.saatBerhenti();
      if (status.modeSetelKd == true) {
        // Mode tuning Kd aktif: magenta
        setRgbledColor(255, 0, 255);
      } else if (status.modeOperasi == MODE_LEARNING) {
        // Mode learning aktif: kuning
        setRgbledColor(255, 255, 0);
      } else if (status.modeOperasi == MODE_RACING) {
        // Mode racing: hijau
        setRgbledColor(0, 255, 0);
      } else if (status.modeOperasi == MODE_RC) {
        // Mode RC: cyan
        setRgbledColor(0, 255, 255);
      } else {
        // Mode normal: mati
        setRgbledColor(0, 0, 0);
      }
    }

    delay(2);
  }
}
