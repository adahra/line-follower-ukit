// config.h — seluruh nilai konfigurasi robot dalam satu tempat.
// Tune di trek cukup ubah file ini. Variabel state runtime (bukan config)
// tetap di LineFollowerFin.ino.
#ifndef CONFIG_H
#define CONFIG_H

// --- PENGATURAN PID (default awal; Kp/Kd/speed/ambang dioverride EEPROM) ---
float Kp = 21.7;
float Ki = 0.01;
float Kd = 1.5;

int kecepatanDasarMaks = 110;  // Batas kecepatan tertinggi saat di jalan lurus
int kecepatanDasarMin = 50;    // Batas kecepatan terendah saat menikung tajam
int kecepatanDasar = 90;       // Variabel dinamis yang akan dikalibrasi oleh PID

// --- KONSTANTA TUNE (dulu magic number tersebar di badan kode) ---
const float FAKTOR_REM_ADAPTIF = 0.6;  // agresivitas pengereman saat melenceng
const float BATAS_INTEGRAL = 50;       // anti windup: I dijepit ±nilai ini
const int SPEED_TOMBOL_MIN = 60;       // batas bawah kecepatan maks via tombol
const int SPEED_TOMBOL_MAKS = 150;     // batas atas (sama dengan derate servo)

// --- BATAS KHUSUS SERVO 360 (open-loop, lihat docs setServoTurn) ---
// Docs resmi: speed 0-255. 150 adalah derate yang disengaja agar robot stabil.
// Servo punya deadband: speed kecil hanya berdengung, roda tidak bergerak.
const int SERVO_MIN_EFEKTIF = 25;  // speed 1-24 dipaksa ke 25 atau 0
const int SERVO_MAKS_DIPAKAI =
  150;                                        // derate dari 255, jangan naikkan tanpa uji lintasan
const int SLEW_MAKS_PER_LOOP = 10;            // batas perubahan speed per loop anti slip
const unsigned long BELOK_TIMEOUT_MS = 1500;  // anti macet di while belok 90°
const unsigned long RECOVERY_TIMEOUT_MS =
  2000;  // batas putar cari garis, lalu berhenti

// --- ANTI GOYANG SENSOR: filter eksponensial agar 1 bacaan liar
// tidak langsung jadi koreksi PID penuh (error range kecil, ±10).
const float ALPHA_SENSOR =
  0.5;  // bobot bacaan baru 0-1, makin besar makin responsif

// --- PID interval tetap: D = error-lastError hanya valid bila dt konstan.
// Tanpa ini, delay tombol (300-1000ms) membuat D melonjak acak.
const unsigned long PID_INTERVAL_MS = 10;

// --- TOMBOL NON-BLOCKING: cooldown millis() + edge-detect.
// delay() di sini membuat robot buta garis selama ratusan ms.
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

// --- HALANGAN DEPAN: berhenti bila objek <= 5 cm (sensor ultrasonic) ---
// Docs: readUltrasonicDistance(id) -> jarak 0-400 cm.
// Sesuaikan ULTRASONIC_ID dengan port sensor yang menghadap depan.
const int ULTRASONIC_ID = 1;
const int JARAK_BERHENTI_CM = 5;
const unsigned long CEK_HALANGAN_MS = 100;  // ultrasonic butuh jeda antar ping

// --- TEPUK TANGAN: toggle jalan/berhenti via sensor suara ---
// Docs: readSoundValue(id) -> intensitas 0-1023.
// Tepuk = lonjakan melewati ambang; histeresis + cooldown mencegah
// satu tepukan dibaca berkali-kali. Sesuaikan ID dan ambang di bawah
// dengan hasil Serial monitor di ruangan lomba (ambang saat ini tebakan).
const int SOUND_SENSOR_ID = 1;
const int TEPUK_AMBANG = 500;
const int TEPUK_LEPAS = 350;                  // histeresis: suara harus turun segini dulu
const unsigned long CEK_SUARA_MS = 30;        // polling cepat agar tepuk tak terlewat
const unsigned long TEPUK_COOLDOWN_MS = 800;  // jeda antar toggle

// --- BATERAI LEMAH: hentikan semua fungsi bila tegangan <= ambang ---
// Docs: readBatteryVoltage() -> float 0-8.4 V. Dicek throttle 500 ms;
// sekali lemah, robot dikunci mati sampai power cycle (restart).
const float TEGANGAN_MIN = 5.5;
const unsigned long CEK_BATERAI_MS = 500;

// --- TELEMETRI: dump CSV berkala saat jalan untuk tuning berbasis data ---
// Format: t(ms),err,pid,dasar,kiri,kanan,senK,senN. Buka via Serial Plotter.
// Header dicetak sekali tiap run.
const unsigned long TELEM_MS = 100;

// --- IMU ONBOARD: deteksi miring/terangkat + deteksi macet ---
// Docs: IMU::init() sekali di setup; IMU::read() tiap loop (butuh SDK terbaru).
// getRawGyroZ() = yaw °/detik; getRoll()/getPitch() = kemiringan derajat.
const unsigned long CEK_IMU_MS = 50;
const float TILT_MAKS_DERAJAT = 45.0;  // di atas ini = terangkat/terbalik

// --- Deteksi macet: diperintah jalan tapi tak bergerak ---
// Kriteria ganda (keduanya harus diam): bacaan IR beku + magnitude
// akselerasi stabil selama STUCK_DIAM_MS. Kalau trek lurus panjang
// memicu false positive, naikkan STUCK_DIAM_MS atau toleransinya.
const unsigned long STUCK_DIAM_MS = 2000;
const int STUCK_TOLERANSI_IR = 1;
const float STUCK_TOLERANSI_ACCEL = 0.15;  // 15% perubahan magnitude

// --- AMBANG SENSOR GARIS (default awal; dioverride kalibrasi/EEPROM) ---
// Ambang per sensor: dua sensor fisik jarang identik, ambang tunggal
// memaksa kompromi yang merugikan sisi yang kurang sensitif.
int AMBANG_KIRI = 8;
int AMBANG_KANAN = 8;

// --- ALAMAT ADDRESS EEPROM ---
const int ADDR_EEPROM_CHECK = 0;
const int ADDR_KP = 4;
const int ADDR_KD = 8;
const int ADDR_SPEED = 12;
const int ADDR_AMBANG = 16;        // ambang kiri (format lama: ambang tunggal)
const int ADDR_AMBANG_KANAN = 18;  // ambang kanan (baru; default bila belum ada)

#endif
