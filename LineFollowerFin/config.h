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
const float FAKTOR_REM_ADAPTIF = 0.6;  // agresivitas pengereman saat melenceng
const float BATAS_INTEGRAL = 50;       // anti windup: I dijepit ±nilai ini
const int SPEED_TOMBOL_MIN = 60;       // batas bawah kecepatan maks via tombol
const int SPEED_TOMBOL_MAKS = 150;     // batas atas (sama dengan derate servo)

// --- DYNAMIC SPEED PROFILE (Turbo Mode) ---
const int KECEPATAN_TURBO = 150;       // Kecepatan maksimal saat jalan lurus panjang
const float LURUS_THRESHOLD = 2.0;    // Jika abs(error) < nilai ini, dianggap jalan lurus
const unsigned long LURUS_TIME_REQUIRED = 500; // Waktu (ms) harus lurus sebelum masuk mode Turbo
const int TURBO_RAMP_SPEED = 2;        // Kenaikan kecepatan per update (agar tidak slip)

// --- BATAS KHUSUS SERVO 360 (open-loop, lihat docs setServoTurn) ---
// Docs resmi: speed 0-255. 150 adalah derate yang disengaja agar robot stabil.
// Servo punya deadband: speed kecil hanya berdengung, roda tidak bergerak.
const int SERVO_MIN_EFEKTIF = 25;  // speed 1-24 dipaksa ke 25 atau 0
const int SERVO_MAKS_DIPAKAI = 150;  // derate dari 255, jangan naikkan tanpa uji lintasan
const int SLEW_MAKS_PER_LOOP = 10;   // batas perubahan speed per loop anti slip
const unsigned long BELOK_TIMEOUT_MS = 1500;  // anti macet di while belok 90°

// --- PIVOT RODA DALAM (Counter-Steer saat tikungan tajam) ---
const bool ENABLE_PIVOT_REVERSE = true; // izinkan roda dalam berputar balik saat PID tinggi
const int PIVOT_REVERSE_MAKS = 60;      // batas kecepatan mundur roda dalam agar tidak terpelanting

// --- RECOVERY GARIS BERTINGKAT ---
const unsigned long RECOVERY_T1_MS = 400;      // 0-400ms: putar ringan searah lastError
const unsigned long RECOVERY_T2_MS = 1200;     // 400-1200ms: putar sedang searah lastError
const unsigned long RECOVERY_T3_MS = 1800;     // 1200-1800ms: sapuan balik lawan lastError
const unsigned long RECOVERY_TIMEOUT_MS = 2000; // >1800-2000ms: timeout & berhenti total
const int RECOVERY_SPEED_1 = 35;               // speed putar ringan
const int RECOVERY_SPEED_2 = 60;               // speed putar sedang
const int RECOVERY_SPEED_3 = 75;               // speed sapuan balik agresif

// --- ANTI GOYANG SENSOR: filter eksponensial agar 1 bacaan liar
// tidak langsung jadi koreksi PID penuh (error range kecil, ±10).
const float ALPHA_SENSOR = 0.5;  // bobot bacaan baru 0-1, makin besar makin responsif

// --- PID interval tetap: D = error-lastError hanya valid bila dt konstan.
// Tanpa ini, delay tombol (300-1000ms) membuat D melonjak acak.
const unsigned long PID_INTERVAL_MS = 10;

// --- TOMBOL NON-BLOCKING: cooldown millis() + edge-detect.
// delay() di sini membuat robot buta garis selama ratusan ms.
const unsigned long COOLDOWN_TOMBOL_MS = 300;

// --- LAMPU OTOMATIS SAAT GELAP (sensor cahaya + eye lamp) ---
// Docs: readLightValue(id) -> 0-4000 lux; setEyelightAllPetals(id,r,g,b).
// Wiring terverifikasi: sensor cahaya memakai ID 1.
const int LIGHT_SENSOR_ID = 1;
const int EYELIGHT_KIRI = 1;
const int EYELIGHT_KANAN = 2;
const int AMBANG_GELAP_NYALA = 100;       // lux: di bawah ini lampu menyala
const int AMBANG_GELAP_MATI = 150;        // lux: histeresis, di atas ini lampu mati
const unsigned long CEK_CAHAYA_MS = 200;  // throttle agar tak baca tiap loop

// --- HALANGAN DEPAN: berhenti bila objek <= 5 cm (sensor ultrasonic) ---
// Docs: readUltrasonicDistance(id) -> jarak 0-400 cm.
// Wiring terverifikasi: sensor ultrasonic depan memakai ID 1.
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
extern int AMBANG_KIRI;
extern int AMBANG_KANAN;

// --- ALAMAT ADDRESS EEPROM ---
const int ADDR_EEPROM_CHECK = 0;
const int ADDR_KP = 4;
const int ADDR_KI = 8;
const int ADDR_KD = 12;
const int ADDR_SPEED = 16;
const int ADDR_AMBANG = 20;        // ambang kiri (format lama: ambang tunggal)
const int ADDR_AMBANG_KANAN = 22;  // ambang kanan (baru; default bila belum ada)

// --- OPSI BOOT ---
const bool AUTO_CALIBRATE_ON_BOOT = false;  // true = kalibrasi otomatis saat EEPROM invalid

// --- OPSI MEMORI ---
const bool ENABLE_TELEMETRI = false;        // true = kirim CSV telemetri (pakai RAM + Flash)

// --- LAP TIMER ---
const int LAP_SF_THRESHOLD = 12;       // ambang deteksi start/finish (kedua sensor)
const unsigned long LAP_MIN_LAP_MS = 3000; // minimal 3 detik per lap
const bool ENABLE_LAP_TIMER = true;    // aktifkan lap timer

// --- BLUETOOTH / BLE ---
const bool ENABLE_BLUETOOTH = true;           // aktifkan Bluetooth control
const unsigned long BT_BAUD = 115200;         // baud rate BT module
const unsigned long BT_TELEMETRY_MS = 200;    // interval auto telemetry
const int BT_MODULE_TYPE = 1;                 // 1=HM-10, 2=uKit built-in

// --- PATH PLANNER ---
const bool ENABLE_PATH_PLANNER = false;       // default OFF (RAM ATMega2560 terbatas)
const int GRID_CELL_SIZE_CM = 20;             // ukuran cell grid (cm)
const int PATH_GRID_WIDTH = 16;               // lebar grid
const int PATH_GRID_HEIGHT = 16;              // tinggi grid
const int JARAK_OBSTACLE_CM = 30;             // ultrasonic <= ini = cell depan jadi obstacle

#endif