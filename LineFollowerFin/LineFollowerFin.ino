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
#include "bluetooth_ctrl.h"
// #include "path_planner.h"  // Optional: enable via config.h + move optional/path_planner/ to sketch folder
// ucode.h included HERE only (single definition). Other files use ukit_wrapper.h

// Wiring antar modul via referensi (lihat header masing-masing).
Status status;
SensorGaris sensor;
Penggerak gerak;
Pengaman aman(status, gerak, sensor);
Mapper mapper;
LapTimer lapTimer;
// PathPlanner pathPlanner;  // Optional: uncomment + include path_planner.h when needed
Antarmuka ui(status, sensor, gerak, aman, lapTimer, mapper);
BluetoothCtrl bluetooth(status, gerak, sensor, aman, lapTimer);

void setup() {
  Initialization();
  IMU::init();
  if (protocolRunState == false) {
    sensor.muat();
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
    Serial.println(F("Path Planner: Ready"));
  #endif
}

void loop() {
  protocol();
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
    ui.cekSerial();         // serial command parser (Kp/Ki/Kd/speed/ambang/calibrate/save/help)
    aman.lampuOtomatis();   // headlight otomatis, non-blocking (throttle 200ms)
    
    // Bluetooth update
    #if ENABLE_BLUETOOTH
      bluetooth.update();
    #endif

    // Path Planner update
    #if ENABLE_PATH_PLANNER
      pathPlanner.updateMap(sensor, pathPlanner.getRobotGridX(), pathPlanner.getRobotGridY(), 0);
    #endif

    if (status.jalan == true) {
      // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
      if (aman.halangan()) {
        return;
      }

      int bacaKiri = sensor.bacaKiri();
      int bacaKanan = sensor.bacaKanan();

      // --- RACING PREDICTION ---
      if (status.modeOperasi == MODE_RACING) {
        EventType predType;
        ActionType predAct;
        if (mapper.checkPrediction(millis(), predType, predAct)) {
          // Robot memprediksi akan ada event segera
          // Optimasi: Bisa kurangi kecepatan sedikit untuk stabilitas belokan
          // atau siapkan Turbo jika predAct == ACT_STRAIGHT
          
          // Feedback LED saat ada prediction akan datang
          if (predAct == ACT_STRAIGHT) {
            setRgbledColor(255, 255, 255); // Putih = lurus
          } else if (predAct == ACT_LEFT) {
            setRgbledColor(0, 255, 0); // Hijau = belok kiri
          } else if (predAct == ACT_RIGHT) {
            setRgbledColor(255, 0, 0); // Merah = belok kanan
          }
        } else {
          setRgbledColor(0, 0, 255); // Biru = tidak ada event prediction
        }
      }

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ATAU LOST LINE ---
      // Both sensor putih (>= ambang). Bisa 2 hal:
      //   1. Intersection: loop sebelumnya di garis (dalamGaris=true) -> putih
      //      lebar = persimpangan, putar sesuai sensor.
      //   2. Lost line: loop sebelumnya di garis lebar (dalamGaris=false) ->
      //      keluar garis = garis putus, recovery.
      if (bacaKiri >= AMBANG_KIRI && bacaKanan >= AMBANG_KANAN) {
        if (!status.dalamGaris) {
          // Lost line: garis putus setelah di garis hitam lebar.
          if (status.recoveryMulai == 0) {
            status.recoveryMulai = millis();
          }
          if (millis() - status.recoveryMulai > RECOVERY_TIMEOUT_MS) {
            gerak.berhenti();
            setRgbledColor(255, 0, 0);
          } else {
            setRgbledColor(255, 0, 255);
            gerak.putarCariGaris(gerak.errorTerakhir());
          }
          return;
        }

        setRgbledColor(255, 255, 255);
        gerak.majuManual(kecepatanDasarMin,
                         150);  // Lewati perempatan dengan kecepatan aman

        // Baca ulang setelah maju: keputusan belok harus pakai data segar
        // yang sudah difilter (bacaan mentah liar = risiko belok salah arah).
        bacaKiri = sensor.bacaKiri();
        bacaKanan = sensor.bacaKanan();

        ActionType act = ACT_STRAIGHT;
        if (bacaKiri > bacaKanan) {
          act = ACT_RIGHT;
          gerak.belokKanan90(AMBANG_KANAN);
        } else if (bacaKanan > bacaKiri) {
          act = ACT_LEFT;
          gerak.belokKiri90(AMBANG_KIRI);
        } else {
          gerak.majuTerus();
        }

        // Rekam event jika dalam mode learning
        if (status.modeOperasi == MODE_LEARNING) {
          mapper.recordEvent(EVENT_INTERSECTION, act);
          Serial.print(F("Event tercatat: "));
          Serial.println(act);
        }

        // Setelah lewat perempatan: robot berada di putih lebar (luar garis).
        status.dalamGaris = false;
        setRgbledColor(0, 0, 0);
      }

      // --- KONDISI B: DI DALAM GARIS HITAM LEBAR (both sensor hitam) ---
      // Sensor IR: hitam = bacaan rendah (< ambang), putih = tinggi (>= ambang).
      // Both hitam = robot di tengah garis lebar, BUKAN hilang garis.
      // Hilang garis seharusnya both putih (di luar garis), tapi itu sudah
      // ditangani Kondisi A (intersection). Kondisi B lama salah: recovery
      // di both hitam bikin robot putar liar di garis panjang (melenceng).
      else if (bacaKiri < AMBANG_KIRI && bacaKanan < AMBANG_KANAN) {
        setRgbledColor(0, 255, 255);  // cyan = di garis hitam lebar
        gerak.majuTerus();            // maju lurus, jangan recovery
        status.recoveryMulai = 0;     // reset state recovery
        status.dalamGaris = true;     // di tengah garis, mengikuti
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
        }

        status.dalamGaris = true;     // satu sensor putih = sedang ikut garis

        // Di antara interval PID, pertahankan speed terakhir.
        if (!gerak.hitung(bacaKiri, bacaKanan, dariRecovery)) {
          gerak.terapkanTerakhir();
          return;
        }

        // --- LAP TIMER UPDATE ---
        // Hanya update saat PID interval tiba (gerak.hitung return true)
        if (ENABLE_LAP_TIMER) {
          float currentError = (bacaKiri - bacaKanan) / 2.0;
          lapTimer.update(sensor, kecepatanDasar, currentError);
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
      } else {
        // Mode normal: mati
        setRgbledColor(0, 0, 0);
      }
    }

    delay(2);
  }
}
