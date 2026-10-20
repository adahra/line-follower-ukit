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
#include "path_planner.h"
// ucode.h included HERE only (single definition). Other files use ukit_wrapper.h

// Wiring antar modul via referensi (lihat header masing-masing).
Status status;
SensorGaris sensor;
Penggerak gerak;
Pengaman aman(status, gerak, sensor);
Antarmuka ui(status, sensor, gerak, aman, lapTimer, pathPlanner);
Mapper mapper;
LapTimer lapTimer;
BluetoothCtrl bluetooth(status, gerak, sensor, aman, lapTimer);
PathPlanner pathPlanner;

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

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
      if (bacaKiri >= AMBANG_KIRI && bacaKanan >= AMBANG_KANAN) {
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

        setRgbledColor(0, 0, 0);
      }

      // --- KONDISI B: LOST LINE RECOVERY (dengan timeout) ---
      else if (bacaKiri < AMBANG_KIRI && bacaKanan < AMBANG_KANAN) {
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
        }

        // Di antara interval PID, pertahankan speed terakhir.
        if (!gerak.hitung(bacaKiri, bacaKanan, dariRecovery)) {
          gerak.terapkanTerakhir();
          return;
        }

        // --- LAP TIMER UPDATE ---
        // Hanya update saat PID interval tiba (gerak.hitung return true)
        if (ENABLE_LAP_TIMER) {
          float currentError = (bacaKiri - bacaKanan) / 2.0;
          lapTimer.update(sensor, gerak.kecepatanDasar, currentError);
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

// --- Pemeriksaan kesehatan sensor (non-blocking) ---
bool cekKesehatanSaatIni = false;
unsigned long terakhirCekKesehatan = 0;
const unsigned long INTERVAL_CEK_KESEHATAN = 5000; // Cek setiap 5 detik

// Di dalam loop(), setelah baca sensor tetapi sebelum decision:
if (status.jalan && !aman.halangan()) {
  // Periodic health check
  unsigned long now = millis();
  if (now - terakhirCekKesehatan >= INTERVAL_CEK_KESEHATAN) {
    terakhirCekKesehatan = now;
    cekKesehatanSaatIni = sensor.cekKesehatanSensor();
    if (!cekKesehatanSaatIni) {
      Serial.println(F("PENTING: Sensor IR keluar rentang normal!"));
    }
  }
  
  // Tampilkan status kesehatan di serial secara periodik
  if (cekKesehatanSaatIni) {
    Serial.print(F("."));  // Titik berkurang = sensor baik
  } else {
    Serial.println(F("!"));  // Tanda sering = sensor masalah
  }
}
