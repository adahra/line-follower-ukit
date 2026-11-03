// config.h — seluruh nilai konfigurasi robot dalam satu tempat.
// Tune di trek cukup ubah file ini. Variabel state runtime (bukan config)
// tetap di LineFollowerFin.ino.
#ifndef CONFIG_H
#define CONFIG_H

// --- PENGATURAN PID (default awal; Kp/Kd/speed/ambang dioverride EEPROM) ---
extern float Kp;
extern float Ki;
extern float Kd;

extern int kecepatanDasarMaks;  // Batas kecepatan tertinggi saat di jalan lurus
extern int kecepatanDasarMin;   // Batas kecepatan terendah saat menikung tajam
extern int kecepatanDasar;      // Variabel dinamis yang akan dikalibrasi oleh PID

// --- KONSTANTA TUNE (dulu magic number tersebar di badan kode) ---
// 0.4 -> 0.35: pengereman adaptif ikut-ikutan PID oscillasi,
// bikin speed berombak di lurus (zigzag). Turunkan agar speed stabil.

#define FAKTOR_REM_ADAPTIF 0.35
#define BATAS_INTEGRAL 50       // anti windup: I dijepit ±nilai ini
#define SPEED_TOMBOL_MIN 60     // batas bawah kecepatan maks via tombol
#define SPEED_TOMBOL_MAKS 150   // batas atas (sama dengan derate servo)

// --- DYNAMIC SPEED PROFILE (Turbo Mode) ---
// 150 -> 120: turunkan ceiling turbo di lurus (150 bikin overshoot).
#define KECEPATAN_TURBO 120
// 1.5 -> 3.0: turbo HANYA aktif di lurus benar (error > 3 = tikungan).
// 1.5 terlalu longgar -> turbo aktif di tengah zigzag -> semakin labil.
#define LURUS_THRESHOLD 3.0
#define LURUS_TIME_REQUIRED 500  // Waktu (ms) harus lurus sebelum masuk mode Turbo
#define TURBO_RAMP_SPEED 1       // 3->1: turbo naik perlahan (cepat bikin slip/overshoot di lurus)

// --- BATAS KHUSUS SERVO 360 (open-loop, lihat docs setServoTurn) ---
// Docs resmi: speed 0-255. 150 adalah derate yang disengaja agar robot stabil.
// Servo punya deadband: speed kecil hanya berdengung, roda tidak bergerak.
#define SERVO_MIN_EFEKTIF 25     // speed 1-24 dipaksa ke 25 atau 0
#define SERVO_MAKS_DIPAKAI 150   // derate dari 255, jangan naikkan tanpa uji lintasan
// 10 -> 14: slew membatasi perubahan speed per loop -> motor lamban
// di lurus, bikin zigzag lambat. Naikkan sedikit agar PID bisa mengikuti.
#define SLEW_MAKS_PER_LOOP 14
#define BELOK_TIMEOUT_MS 1500    // anti macet di while belok 90°
#define RECOVERY_TIMEOUT_MS 2000 // batas putar cari garis, lalu berhenti

// --- ANTI GOYANG SENSOR: filter eksponensial agar 1 bacaan liar
// tidak langsung jadi koreksi PID penuh (error range kecil, ±10).
// 0.5 -> 0.4: filter lebih halus, redam bacaan liar yang jadi sumber D noise.
// 0.4 -> 0.65: filter terlalu berat (5 siklus untuk 90%), koreksi datang terlambat.
// 0.65 = 2-3 siklus untuk 90%: respons lebih menggigit tanpa kehilangan redam noise.
#define ALPHA_SENSOR 0.65

// --- PID interval tetap: D = error-lastError hanya valid bila dt konstan.
// Tanpa ini, delay tombol (300-1000ms) membuat D melonjak acak.
#define PID_INTERVAL_MS 10

// --- TOMBOL NON-BLOCKING: cooldown millis() + edge-detect.
// delay() di sini membuat robot buta garis selama ratusan ms.
#define COOLDOWN_TOMBOL_MS 300

// --- LAMPU OTOMATIS SAAT GELAP (sensor cahaya + eye lamp) ---
// Docs: readLightValue(id) -> 0-4000 lux; setEyelightAllPetals(id,r,g,b).
// Sesuaikan ID sensor cahaya dengan port yang terpasang di robot.
#define LIGHT_SENSOR_ID 1
#define EYELIGHT_KIRI 1
#define EYELIGHT_KANAN 2
#define AMBANG_GELAP_NYALA 100    // lux: di bawah ini lampu menyala
#define AMBANG_GELAP_MATI 150    // lux: histeresis, di atas ini lampu mati
#define CEK_CAHAYA_MS 200        // throttle agar tak baca tiap loop

// --- HALANGAN DEPAN: berhenti bila objek <= 5 cm (sensor ultrasonic) ---
// Docs: readUltrasonicDistance(id) -> jarak 0-400 cm.
// Sesuaikan ULTRASONIC_ID dengan port sensor yang menghadap depan.
#define ULTRASONIC_ID 1
#define JARAK_BERHENTI_CM 5
#define CEK_HALANGAN_MS 100      // ultrasonic butuh jeda antar ping

// --- TEPUK TANGAN: toggle jalan/berhenti via sensor suara ---
// Docs: readSoundValue(id) -> intensitas 0-1023.
// Tepuk = lonjakan melewati ambang; histeresis + cooldown mencegah
// satu tepukan dibaca berkali-kali. Sesuaikan ID dan ambang di bawah
// dengan hasil Serial monitor di ruangan lomba (ambang saat ini tebakan).
#define SOUND_SENSOR_ID 1
#define TEPUK_AMBANG 500
#define TEPUK_LEPAS 350          // histeresis: suara harus turun segini dulu
#define CEK_SUARA_MS 30          // polling cepat agar tepuk tak terlewat
#define TEPUK_COOLDOWN_MS 800    // jeda antar toggle

// --- BATERAI LEMAH: hentikan semua fungsi bila tegangan <= ambang ---
// Docs: readBatteryVoltage() -> float 0-8.4 V. Dicek throttle 500 ms;
// sekali lemah, robot dikunci mati sampai power cycle (restart).
#define TEGANGAN_MIN 5.5
#define CEK_BATERAI_MS 500

// --- TELEMETRI: dump CSV berkala saat jalan untuk tuning berbasis data ---
// Format: t(ms),err,pid,dasar,kiri,kanan,senK,senN. Buka via Serial Plotter.
// Header dicetak sekali tiap run.
#define TELEM_MS 50

// --- IMU ONBOARD: deteksi miring/terangkat + deteksi macet ---
// Docs: IMU::init() sekali di setup; IMU::read() tiap loop (butuh SDK terbaru).
// getRawGyroZ() = yaw °/detik; getRoll()/getPitch() = kemiringan derajat.
#define CEK_IMU_MS 50
#define TILT_MAKS_DERAJAT 45.0   // di atas ini = terangkat/terbalik

// --- Deteksi macet: diperintah jalan tapi tak bergerak ---
// Kriteria ganda (keduanya harus diam): bacaan IR beku + magnitude
// akselerasi stabil selama STUCK_DIAM_MS. Kalau trek lurus panjang
// memicu false positive, naikkan STUCK_DIAM_MS atau toleransinya.
#define STUCK_DIAM_MS 2000
#define STUCK_TOLERANSI_IR 1
#define STUCK_TOLERANSI_ACCEL 0.15  // 15% perubahan magnitude

// --- AMBANG SENSOR GARIS (default awal; dioverride kalibrasi/EEPROM) ---
// Ambang per sensor: dua sensor fisik jarang identik, ambang tunggal
// memaksa kompromi yang merugikan sisi yang kurang sensitif.
extern int AMBANG_KIRI;
extern int AMBANG_KANAN;

// --- ALAMAT ADDRESS EEPROM ---
#define ADDR_EEPROM_CHECK 0
#define ADDR_KP 4
#define ADDR_KI 8
#define ADDR_KD 12
#define ADDR_SPEED 16
#define ADDR_AMBANG 20        // ambang kiri (format lama: ambang tunggal)
#define ADDR_AMBANG_KANAN 22  // ambang kanan (baru; default bila belum ada)
#define ADDR_MAP 24           // peta waypoint (magic + count + array Waypoint)

// --- OPSI BOOT ---
#define AUTO_CALIBRATE_ON_BOOT false  // true = kalibrasi otomatis saat EEPROM invalid

// --- OPSI MEMORI ---
#define ENABLE_TELEMETRI false         // true = kirim CSV telemetri (pakai RAM + Flash)

// --- LAP TIMER ---
#define LAP_SF_THRESHOLD 12           // ambang deteksi start/finish (kedua sensor)
#define LAP_MIN_LAP_MS 3000           // minimal 3 detik per lap
#define ENABLE_LAP_TIMER false        // aktifkan lap timer

// --- BLUETOOTH / BLE ---
#define ENABLE_BLUETOOTH false        // aktifkan Bluetooth control
#define BT_BAUD 115200                // baud rate BT module
#define BT_TELEMETRY_MS 200           // interval auto telemetry
#define BT_MODULE_TYPE 1              // 1=HM-10, 2=uKit built-in

// --- SERIAL OUTPUT ---
// Matikan seluruh output Serial biar beneran silent (bisa dibuat tidak ikut compile via macro)
#ifndef ENABLE_SERIAL_OUTPUT
#define ENABLE_SERIAL_OUTPUT true
#endif
#if ENABLE_BLUETOOTH
#undef ENABLE_SERIAL_OUTPUT
#define ENABLE_SERIAL_OUTPUT true
#endif

// --- SERIAL MODE ---
#define ENABLE_SERIAL_MODE true       // bisa di-disable jika tidak perlu
#if ENABLE_BLUETOOTH
#undef ENABLE_SERIAL_MODE
#define ENABLE_SERIAL_MODE true       // serial wajib aktif jika Bluetooth aktif
#endif

// --- PATH PLANNER ---
#define ENABLE_PATH_PLANNER false     // default OFF (RAM ATMega2560 terbatas)
#define GRID_CELL_SIZE_CM 20          // ukuran cell grid (cm)
#define PATH_GRID_WIDTH 8             // lebar grid (8x8 untuk hemat RAM)
#define PATH_GRID_HEIGHT 8            // tinggi grid

// --- FUZZY PID ---
// Adaptive PID: fuzzy controller menghasilkan delta Kp/Kd/Ki berdasarkan
// error dan delta_error. Aktifkan untuk respons PID yang dinamis.
#define ENABLE_FUZZY_PID false         // aktifkan fuzzy adaptive PID
#define FUZZY_ERROR_MAX 10.0           // range error masukan [-10, 10]
#define FUZZY_DERROR_MAX 10.0          // range delta_error masukan [-10, 10]
#define FUZZY_DKP_MAX 5.0              // range delta Kp keluaran [-5, 5]
#define FUZZY_DKD_MAX 2.0              // range delta Kd keluaran [-2, 2]
#define FUZZY_DKI_MAX 0.5              // range delta Ki keluaran [-0.5, 0.5]

// --- REMOTE CONTROL (RC) ---
// RC receiver dibaca lewat digital pin (PWM/PPM) atau Serial (ESP-based).
#define ENABLE_RC false               // aktifkan RC control
#define RC_BAUD 115200                // baud rate untuk RC Serial
#define RC_PIN 2                      // digital pin untuk PWM/PPM input
#define RC_MIN_US 1000                // pulse width minimum (us)
#define RC_MAX_US 2000                // pulse width maksimum (us)
#define RC_CENTER_US 1500             // pulse width tengah (netral)
#define RC_DEADBAND_US 50             // toleransi dekat center = tidak ada steer
#define RC_STEER_GAIN 1.0             // pengali delta steering
#define RC_PULSE_TIMEOUT_MS 500       // timeout kehilangan sinyal
#define RC_SCALE_MIN 0                // throttle skala minimum (%)
#define RC_SCALE_MAX 100              // throttle skala maksimum (%)
#define RC_BUTTON_A_PIN 3            // opsi: pin untuk channel tombol A
#define RC_BUTTON_B_PIN 4            // opsi: pin untuk channel tombol B

#include "serial_silent.h"

#endif
