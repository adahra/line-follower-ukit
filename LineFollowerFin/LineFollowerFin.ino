#include "ucode.h"
#include <EEPROM.h>
#include "config.h"  // seluruh nilai konfigurasi
#include "status.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"
#include "antarmuka.h"

// Wiring antar modul via referensi (lihat header masing-masing).
Status status;
SensorGaris sensor;
Penggerak gerak;
Pengaman aman(status, gerak, sensor);
Antarmuka ui(status, sensor, gerak, aman);

void setup() {
  Initialization();
  IMU::init();  // gyro onboard untuk deteksi miring + macet
  if (protocolRunState == false) {
    sensor.muat();   // pakai nilai tersimpan; kalibrasi manual via 2 tombol
    sensor.cetak();   // tampilkan semua nilai saat colok USB/hidup
    Serial.print("Baterai: ");
    Serial.print(aman.tegangan());
    Serial.println(" V");
    Serial.println("Setup selesai. Kalibrasi manual: tekan 2 tombol bersamaan.");
  }
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
    aman.lampuOtomatis();   // headlight otomatis, non-blocking (throttle 200ms)

    if (status.jalan == true) {
      // Prioritas tertinggi: halangan depan menghentikan semua logika jalan.
      if (aman.halangan()) {
        return;
      }

      int bacaKiri = sensor.bacaKiri();
      int bacaKanan = sensor.bacaKanan();

      // --- KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN ---
      if (bacaKiri >= AMBANG_KIRI && bacaKanan >= AMBANG_KANAN) {
        setRgbledColor(255, 255, 255);
        gerak.majuManual(kecepatanDasarMin,
                         150);  // Lewati perempatan dengan kecepatan aman

        // Baca ulang setelah maju: keputusan belok harus pakai data segar
        // yang sudah difilter (bacaan mentah liar = risiko belok salah arah).
        bacaKiri = sensor.bacaKiri();
        bacaKanan = sensor.bacaKanan();

        if (bacaKiri > bacaKanan) {
          gerak.belokKanan90(AMBANG_KANAN);
        } else if (bacaKanan > bacaKiri) {
          gerak.belokKiri90(AMBANG_KIRI);
        } else {
          gerak.majuTerus();
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
      }
    } else {
      gerak.saatBerhenti();
      if (status.modeSetelKd == true) {
        setRgbledColor(255, 0, 255);
      } else {
        setRgbledColor(0, 0, 0);
      }
    }

    delay(2);
  }
}
