#include "ucode.h"
#include <EEPROM.h>
#include "fwd.h"
#include "config.h"
#include "eeprom_layout.h"
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
  // Initialize EEPROM with versioned layout (handles guard byte 3 internally)
  eepromInit();
  Initialization();
  IMU::init();
  if (protocolRunState == false) {
    sensor.muat();
    mapper.muat();  // peta waypoint LEARNING persisten antar power cycle
    bool eepromValid = (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID &&
                        EEPROM.read(EEPROM_ADDR::EE_ADDR_VERSION) == EEPROM_FMT_VERSION);
    if (!eepromValid && AUTO_CALIBRATE_ON_BOOT) {
      Serial.println(F("EEPROM invalid + AUTO_CALIBRATE_ON_BOOT=true -> kalibrasi otomatis..."));
      sensor.kalibrasi(gerak);
    } else {
      sensor.cetak();
      Serial.print(F("Baterai: "));
      Serial.print(aman.tegangan());
      Serial.print(F(" V"));
#if !ENABLE_BATTERY_CHECK
      Serial.print(F(" (cek lemah NONAKTIF)"));
#endif
      Serial.println();
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

// ===== HELPER FUNCTIONS FOR loop() =====

// Handle safety checks: battery, tilt, ultrasonic obstacle
// Returns true if robot should stop (safety triggered)
bool handleSafetyChecks() {
  // Prioritas tertinggi: baterai lemah menghentikan SEMUA fungsi.
  if (aman.bateraiKritis()) {
    return true;
  }

  // Robot miring/terangkat: matikan motor selama tidak rata (tak dikunci).
  if (aman.miring()) {
    return true;
  }

  // Halangan depan menghentikan semua logika jalan.
  if (aman.halangan()) {
    return true;
  }
  return false;
}

// Handle UI updates: buttons, clap, serial, auto lights, heading, BT, RC
void handleUIUpdates() {
  ui.cekTombol();
  ui.cekTepuk();          // toggle jalan/berhenti via tepuk tangan
#if ENABLE_SERIAL_MODE
  ui.cekSerial();         // serial command parser
#endif
  aman.lampuOtomatis();   // headlight otomatis, non-blocking (throttle 200ms)

  // IMU heading: integrasi gyro tiap loop
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
}

// Handle racing prediction logic, updates aksiPrediksi/prediksiMulai
void handleRacingPrediction(int bacaKiri, int bacaKanan,
                            int ambangKiriEfektif, int ambangKananEfektif,
                            ActionType &aksiPrediksi, unsigned long &prediksiMulai,
                            bool &prediksiValid) {
  if (status.modeOperasi != MODE_RACING) {
    aksiPrediksi = ACT_NONE;
    prediksiValid = false;
    return;
  }

  if (mapper.getWaypointCount() == 0) {
    aksiPrediksi = ACT_NONE;
    prediksiValid = false;
    return;
  }

  EventType predType;
  ActionType predAct;
  if (mapper.checkPrediction(millis(), predType, predAct)) {
    aksiPrediksi = predAct;
    prediksiMulai = millis();
  }

  prediksiValid = (aksiPrediksi != ACT_NONE) &&
                  (millis() - prediksiMulai <= RACE_PREDICTION_HOLD_MS);
  if (!prediksiValid) {
    aksiPrediksi = ACT_NONE;
  }

  // LED menampilkan prediksi yang sedang berlaku
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

// Handle RC mode - returns true if RC mode active and handled
bool handleRCMode() {
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
    return true;
  }
#endif
  return false;
}

// Handle SUBSTATE_INTER_CREEP (150ms creep through intersection)
void handleIntersectionCreep(int bacaKiri, int bacaKanan,
                             int ambangKiriEfektif, int ambangKananEfektif,
                             ActionType &aksiPrediksi, bool prediksiValid) {
  gerak.maju(kecepatanDasarMin);
  status.dalamGaris = true;

  if (millis() - status.subStateMulai >= 150) {
    int selisih = bacaKiri - bacaKanan;  // Use cached values
    ActionType act = ACT_STRAIGHT;
    bool pakaiPrediksi = (status.modeOperasi == MODE_RACING) &&
                         prediksiValid &&
                         (abs(selisih) < RACE_PREDICTION_MARGIN);
    if (pakaiPrediksi) {
      act = aksiPrediksi;
      aksiPrediksi = ACT_NONE;
    } else if (selisih > 0) {
      act = ACT_RIGHT;
    } else if (selisih < 0) {
      act = ACT_LEFT;
    } else {
      act = ACT_STRAIGHT;
    }

    if (act == ACT_STRAIGHT) {
      status.subState = SUBSTATE_TRACKING;
      status.subStateAksi = ACT_NONE;
      status.subStateMulai = 0;
      setRgbledColor(0, 0, 0);
#if ENABLE_HEADING_CORRECTION
      gerak.resetHeading();
#endif
      if (status.modeOperasi == MODE_LEARNING) {
        mapper.recordEvent(EVENT_INTERSECTION, act);
      }
    } else if (act == ACT_LEFT) {
      status.subState = SUBSTATE_TURNING;
      status.subStateMulai = millis();
#if ENABLE_ADAPTIVE_THRESHOLD
      status.targetAmbangBelok = sensor.isAdaptiveThresholdEnabled()
                                 ? sensor.ambangDinamis(2) : AMBANG_KIRI;
#else
      status.targetAmbangBelok = AMBANG_KIRI;
#endif
      gerak.putarKiri(kecepatanDasarMin);
    } else if (act == ACT_RIGHT) {
      status.subState = SUBSTATE_TURNING;
      status.subStateMulai = millis();
#if ENABLE_ADAPTIVE_THRESHOLD
      status.targetAmbangBelok = sensor.isAdaptiveThresholdEnabled()
                                 ? sensor.ambangDinamis(1) : AMBANG_KANAN;
#else
      status.targetAmbangBelok = AMBANG_KANAN;
#endif
      gerak.putarKanan(kecepatanDasarMin);
    }
  }
}

// Handle SUBSTATE_TURNING (non-blocking 90° turn)
void handleTurning() {
  unsigned long turnElapsed = millis() - status.subStateMulai;
  bool lineFound = false;
  if (turnElapsed >= 250) {
    if (status.subStateAksi == ACT_LEFT) {
      lineFound = ((int)readInfraredDistance(2) >= status.targetAmbangBelok);
    } else if (status.subStateAksi == ACT_RIGHT) {
      lineFound = ((int)readInfraredDistance(1) >= status.targetAmbangBelok);
    }
  }
  bool turnTimeout = (turnElapsed > (unsigned long)BELOK_TIMEOUT_MS);
  if (lineFound || turnTimeout) {
#if ENABLE_HEADING_CORRECTION
    gerak.resetHeading();
#endif
    if (status.modeOperasi == MODE_LEARNING) {
      mapper.recordEvent(EVENT_INTERSECTION, status.subStateAksi);
#if ENABLE_SERIAL_OUTPUT
      Serial.print(F("Event tercatat: "));
      Serial.println(status.subStateAksi);
#endif
    }
    setRgbledColor(0, 0, 0);
    status.subState = SUBSTATE_TRACKING;
    status.subStateAksi = ACT_NONE;
    status.subStateMulai = 0;
    status.dalamGaris = true;
  } else {
    status.dalamGaris = false;
  }
}

// Handle intersection detection with hysteresis (2 consecutive loops)
bool handleIntersectionDetection(int bacaKiri, int bacaKanan,
                                 int ambangKiriEfektif, int ambangKananEfektif) {
  static uint8_t interCount = 0;
  if (bacaKiri >= ambangKiriEfektif && bacaKanan >= ambangKananEfektif) {
    if (interCount < 2) interCount++;
  } else {
    interCount = 0;
  }

  if (interCount >= 2) {
    setRgbledColor(255, 255, 255);
    status.subState = SUBSTATE_INTER_CREEP;
    status.subStateMulai = millis();
    gerak.maju(kecepatanDasarMin);
    interCount = 0;
    return true;
  }
  return false;
}

// Handle lost line recovery with timeout
void handleLostLineRecovery(int bacaKiri, int bacaKanan,
                            int ambangKiriEfektif, int ambangKananEfektif) {
  if (bacaKiri < ambangKiriEfektif && bacaKanan < ambangKananEfektif) {
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
    status.dalamGaris = false;
  }
}

// Handle normal PID tracking mode
void handleNormalTracking(int bacaKiri, int bacaKanan,
                          int ambangKiriEfektif, int ambangKananEfektif) {
  setRgbledColor(0, 0, 0);

  // Nyangkut di pembatas: berhenti total, tunggu operator.
  if (aman.macet(bacaKiri, bacaKanan)) {
    return;
  }

  // Reset state integral saat kembali dari recovery
  bool dariRecovery = (status.recoveryMulai != 0);
  status.recoveryMulai = 0;
  if (dariRecovery) {
    gerak.resetIntegralSetelahRecovery((bacaKiri - bacaKanan) / 2.0);
#if ENABLE_HEADING_CORRECTION
    gerak.resetHeading();
#endif
  }

  // Di antara interval PID, pertahankan speed terakhir.
  if (!gerak.hitung(bacaKiri, bacaKanan, dariRecovery)) {
    gerak.terapkanTerakhir();
    return;
  }

  // ADAPTIVE THRESHOLD UPDATE
#if ENABLE_ADAPTIVE_THRESHOLD
  if (sensor.isAdaptiveThresholdEnabled()) {
    sensor.updateAmbangDinamis(bacaKiri, bacaKanan, kecepatanDasar);
  }
#endif

  // LAP TIMER UPDATE
  if (ENABLE_LAP_TIMER) {
    float currentError = (bacaKiri - bacaKanan) / 2.0;
    lapTimer.update(bacaKiri, bacaKanan, kecepatanDasar, currentError);
  }

  status.dalamGaris = (bacaKiri >= ambangKiriEfektif) || (bacaKanan >= ambangKananEfektif);
}

// Handle stopped state LED indication
void handleStoppedState() {
  gerak.saatBerhenti();
  status.dalamGaris = false;
  if (status.modeSetelKd == true) {
    setRgbledColor(255, 0, 255);  // Magenta = tuning Kd
  } else if (status.modeOperasi == MODE_LEARNING) {
    setRgbledColor(255, 255, 0);  // Kuning = learning
  } else if (status.modeOperasi == MODE_RACING) {
    setRgbledColor(0, 255, 0);    // Hijau = racing
  } else if (status.modeOperasi == MODE_RC) {
    setRgbledColor(0, 255, 255);  // Cyan = RC
  } else {
    setRgbledColor(0, 0, 0);      // Mati = normal
  }
}

// ===== MAIN LOOP =====
void loop() {
  // PERFORMANCE OPTIMIZATIONS APPLIED
  // 1. Cached sensor readings - eliminate redundant I/O
  // Prediksi RACING terakhir. Disimpan antar-iterasi karena checkPrediction()
  // hanya "fire" sekali per waypoint; aksi harus tetap tersedia sampai
  // persimpangan benar-benar tiba.
  static ActionType aksiPrediksi = ACT_NONE;
  static unsigned long prediksiMulai = 0;
  bool prediksiValid = false;

  protocol();
  if (!status.jalan || status.modeOperasi != MODE_RACING) {
    aksiPrediksi = ACT_NONE;
  }
  if (!status.jalan) {
    status.subState = SUBSTATE_TRACKING;
  }
  if (protocolRunState == false) {
    handleUIUpdates();

    if (status.jalan == true) {
      // Safety checks (battery/tilt already done in handleUIUpdates via aman.lampuOtomatis?
      // No - battery/tilt/ultrasonic are separate. Check them now.
      if (handleSafetyChecks()) {
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

      // RACING PREDICTION
      handleRacingPrediction(bacaKiri, bacaKanan,
                             ambangKiriEfektif, ambangKananEfektif,
                             aksiPrediksi, prediksiMulai, prediksiValid);

      // MODE RC: Remote Control menggantikan semua logika tracer
      if (handleRCMode()) {
        return;
      }

      // SUBSTATE 1: CREEP MELEWATI PERSIMPANGAN (150ms)
      if (status.subState == SUBSTATE_INTER_CREEP) {
        handleIntersectionCreep(bacaKiri, bacaKanan,
                                ambangKiriEfektif, ambangKananEfektif,
                                aksiPrediksi, prediksiValid);
        return;
      }

      // SUBSTATE 2: BELOK 90 DERAJAT NON-BLOCKING
      if (status.subState == SUBSTATE_TURNING) {
        handleTurning();
        return;
      }

      // Histeresis: deteksi persimpangan (butuh 2 loop berturut-turut)
      if (handleIntersectionDetection(bacaKiri, bacaKanan,
                                      ambangKiriEfektif, ambangKananEfektif)) {
        return;
      }

      // KONDISI B: LOST LINE RECOVERY (dengan timeout)
      handleLostLineRecovery(bacaKiri, bacaKanan,
                             ambangKiriEfektif, ambangKananEfektif);

      // KONDISI C: MODE TRACER PID NORMAL DENGAN ADAPTIVE SPEED
      // Only if not in recovery (both sensors below threshold)
      if (!(bacaKiri < ambangKiriEfektif && bacaKanan < ambangKananEfektif)) {
        handleNormalTracking(bacaKiri, bacaKanan,
                             ambangKiriEfektif, ambangKananEfektif);
      }
    } else {
      handleStoppedState();
    }
  }
}
