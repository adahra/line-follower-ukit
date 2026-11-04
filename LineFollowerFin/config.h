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
// Set ENABLE_BATTERY_CHECK=false untuk menonaktifkan latch ini (mis. saat
// pengujian dengan power supply yang tegangannya di bawah ambang).
#define ENABLE_BATTERY_CHECK true
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
// Kriteria: IR beku + magnitude akselerasi stabil selama STUCK_DIAM_MS,
// DAN robot memang sedang butuh koreksi (|error| >= STUCK_ERROR_MIN).
// Syarat terakhir mencegah false positive di lintasan lurus: di lurus
// error ~0, jadi robot yang berjalan lurus tidak dianggap macet.
#define STUCK_DIAM_MS 2000
#define STUCK_TOLERANSI_IR 1
#define STUCK_TOLERANSI_ACCEL 0.15  // 15% perubahan magnitude
#define STUCK_ERROR_MIN 2.0         // |error| minimal agar deteksi macet aktif

// --- AMBANG SENSOR GARIS (default awal; dioverride kalibrasi/EEPROM) ---
// Ambang per sensor: dua sensor fisik jarang identik, ambang tunggal
// memaksa kompromi yang merugikan sisi yang kurang sensitif.
extern int AMBANG_KIRI;
extern int AMBANG_KANAN;

// --- IMU HEADING CORRECTION ---
// Koreksi arah berbasis gyro Z: koreksi PID bila robot drift/tersesak
// dari arah lurus. Hanya aktif di mode tertentu.
#define ENABLE_HEADING_CORRECTION false   // master switch
#define HEADING_UPDATE_MS 20              // throttle gyro read (cepat)
#define HEADING_CALIB_MS 1500             // durasi kalibrasi bias gyro (ms)
#define HEADING_GAIN 0.8                  // gain koreksi steering (0-2)
#define HEADING_MAX_OFFSET 40             // max steering offset dari PID (0-150)
#define HEADING_TARGET_DEFAULT 0.0        // target heading default (lurus)

// --- EEPROM Layout ---
// | Alamat | Isi | Keterangan |
// | --- | --- | --- |
// | 3 | `77` | Guard uKit: `Initialization()` menghapus 0..20 bila byte ini != 77 |
// | 0 | `123` | Magic/check validitas EEPROM |
// | 4 | `Kp` (float) | PID proportional |
// | 8 | `Ki` (float) | PID integral |
// | 12 | `Kd` (float) | PID derivative |
// | 16 | `kecepatanDasarMaks` (int) | Batas kecepatan maksimal |
// | 20 | `AMBANG_KIRI` (int) | Ambang sensor kiri |
// | 22 | `AMBANG_KANAN` (int) | Ambang sensor kanan |
// | 24 | peta waypoint (magic + count + array) | 27..266 (30 x 8 byte) |
// | 280 | `HEADING_GAIN` (float) | Heading correction gain |
// | 284 | `HEADING_TARGET` (float) | Heading target tersimpan |
// | 288 | `AMBANG_KIRI_BASE` (int) | Ambang kiri dasar (untuk adaptive) |
// | 290 | `AMBANG_KANAN_BASE` (int) | Ambang kanan dasar (untuk adaptive) |
// | 292 | `HEADING_BIAS` (float) | Bias gyro yang sudah dikalibrasi |
// | 296 | `88` | Magic data heading (validasi) |
// | 297 | `66` | Magic data adaptive threshold (validasi) |
//
// Catatan: blok 280+ sengaja berada SETELAH peta waypoint (berakhir di 266)
// agar parameter heading/adaptive tidak menimpa waypoint. Jangan menaruh
// parameter baru di 24..266. Guard byte 3 ditulis di setup() SEBELUM
// Initialization() agar library uKit tidak menghapus konfigurasi 0..20.
#define ADDR_EEPROM_CHECK 0
#define ADDR_KP 4
#define ADDR_KI 8
#define ADDR_KD 12
#define ADDR_SPEED 16
#define ADDR_AMBANG 20        // ambang kiri (format lama: ambang tunggal)
#define ADDR_AMBANG_KANAN 22  // ambang kanan (baru; default bila belum ada)
#define ADDR_MAP 24           // peta waypoint (magic + count + array Waypoint)
#define ADDR_HEADING_GAIN 280
#define ADDR_HEADING_TARGET 284
#define ADDR_AMBANG_BASE 288
#define ADDR_AMBANG_KANAN_BASE 290
#define ADDR_HEADING_BIAS 292
#define ADDR_HEADING_MAGIC 296
#define ADDR_ADAPT_MAGIC 297

// --- OPSI BOOT ---
#define AUTO_CALIBRATE_ON_BOOT false  // true = kalibrasi otomatis saat EEPROM invalid

// --- OPSI MEMORI ---
#define ENABLE_TELEMETRI false         // true = kirim CSV telemetri (pakai RAM + Flash)

// --- LAP TIMER ---
#define LAP_SF_THRESHOLD 12           // ambang deteksi start/finish (kedua sensor)
#define LAP_MIN_LAP_MS 3000           // minimal 3 detik per lap
#define ENABLE_LAP_TIMER false        // aktifkan lap timer

// --- RACING PREDICTION ---
// Prediksi Mapper dipakai untuk memilih aksi di persimpangan yang
// pembacaan sensornya ambigu (selisih kiri-kanan kecil).
#define RACE_PREDICTION_MARGIN 2      // |kiri-kanan| < ini = ambigu, pakai prediksi
#define RACE_PREDICTION_HOLD_MS 1200  // prediksi berlaku selama ini (ms)

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
// ALAT PETA/A* MANUAL lewat perintah serial, bukan navigasi otonom.
// Posisi grid robot tidak dapat diketahui tanpa odometri/encoder, jadi
// planner ini tidak mengemudikan motor. Mengaktifkannya memakai ~94% RAM
// ATMega2560 (sisa stack sangat tipis); biarkan OFF kecuali untuk debugging.
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

// --- GAIN SCHEDULING ---
// Gain berbeda per kecepatan: rendah (tikungan) vs tinggi (lurus).
// Low speed: Kp naik (responsif), Kd turun (tidak perlu damping keras).
// High speed: Kp turun (anti overshoot), Kd naik (stabil).
#define ENABLE_GAIN_SCHEDULING false
#define GS_SPEED_LOW_MAX 60
#define GS_SPEED_MID_MAX 100
#define GS_KP_LOW 1.2
#define GS_KP_MID 1.0
#define GS_KP_HIGH 0.85
#define GS_KD_LOW 0.8
#define GS_KD_MID 1.0
#define GS_KD_HIGH 1.3
#define GS_KI_LOW 1.2
#define GS_KI_MID 1.0
#define GS_KI_HIGH 0.8

// --- AUTO-TUNE PID (Relay Feedback / Ziegler-Nichols) ---
// Ganti tuningSequence() grid search dengan relay feedback yang lebih
// principled: amati osilasi, hitung gain/periode kritis, terapkan ZN.
#define ENABLE_AUTO_TUNE_RELAY false
#define AT_RELAY_AMPLITUDE 30.0     // amplitude output relay
#define AT_RELAY_HYSTERESIS 2.0     // band histeresis
#define AT_MAX_CYCLES 4             // max siklus osilasi diamati
#define AT_MIN_PERIOD_MS 300        // periode osilasi minimal (anti noise)
#define AT_SETPOINT 0.0             // setpoint error (0 = lurus)
#define AT_TIMEOUT_MS 8000          // timeout total auto-tune

// --- ADAPTIVE THRESHOLD ---
// Ambang sensor dinamis: naik seiring kecepatan (noise meningkat di
// kecepatan tinggi, sensor IR baca liar lebih sering).
#define ENABLE_ADAPTIVE_THRESHOLD true
#define ADAPT_SPEED_FACTOR 0.25     // pengaruh kecepatan terhadap ambang
#define ADAPT_THRESH_MIN 3
#define ADAPT_THRESH_MAX 18

// --- REMOTE CONTROL (RC) ---
// RC receiver dibaca lewat digital pin (PWM/PPM) atau Serial (ESP-based).
// Mode PWM butuh DUA channel terpisah: throttle dan steering. Membaca dua
// channel dari satu pin (implementasi lama) membuat steering selalu sama
// dengan throttle.
#define ENABLE_RC false               // aktifkan RC control
#define RC_BAUD 115200                // baud rate untuk RC Serial
#define RC_PIN 2                      // channel throttle (PWM/PPM input)
#define RC_STEER_PIN 3                // channel steering (PWM/PPM input)
#define RC_MIN_US 1000                // pulse width minimum (us)
#define RC_MAX_US 2000                // pulse width maksimum (us)
#define RC_CENTER_US 1500             // pulse width tengah (netral)
#define RC_DEADBAND_US 50             // toleransi dekat center = netral
#define RC_STEER_GAIN 1.0             // pengali delta steering
#define RC_PULSE_TIMEOUT_MS 500       // timeout kehilangan sinyal
#define RC_SCALE_MIN 0                // throttle skala minimum (%)
#define RC_SCALE_MAX 100              // throttle skala maksimum (%)
#define RC_BUTTON_A_PIN 5             // opsi: pin channel tombol A (belum dipakai)
#define RC_BUTTON_B_PIN 6             // opsi: pin channel tombol B (belum dipakai)

#include "serial_silent.h"

#endif
