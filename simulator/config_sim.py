# config_sim.py — nilai default meniru LineFollowerFin/config.h + config.cpp.
# Semua nilai tunable ada di sini. Variabel runtime (jalan, mode, posisi) ada
# di robot.py. Nilai yang di-simulasi diambil langsung dari konfig robot asli.

# --- PID (default awal, mirip config.cpp) ---
Kp = 7.0
Ki = 0.05
Kd = 2.2

kecepatanDasarMaks = 85    # batas kecepatan tertinggi (servo derate 0-150)
kecepatanDasarMin = 40     # batas kecepatan terendah (di atas deadband 25)
kecepatanDasar = 85

# --- Dynamic Speed Profile (Turbo Mode) ---
# PENTING: KECEPATAN_TURBO adalah BONUS, bukan target absolut.
# base = kecepatanDasarMaks + bonus turbo - pengereman PID.
# 20 -> dengan maks 85 target lurus sekitar 105 (bukan 150).
KECEPATAN_TURBO = 20          # sinkron dengan dokumentasi & sim
LURUS_THRESHOLD = 2.0
LURUS_TIME_REQUIRED = 500    # Waktu (ms) harus lurus sebelum masuk mode Turbo
TURBO_RAMP_SPEED = 1         # turbo naik perlahan agar tidak slip
SLEW_MAKS_PER_LOOP = 12    # maksimum perubahan speed per loop
PID_INTERVAL_MS = 10       # interval PID konstan (ms)
PID_MS = PID_INTERVAL_MS / 1000.0   # dt untuk normalisasi derivative
LOOP_MS = 10               # periode simulasi per tick (ms)
# --- Ambang sensor garis (default; dioverride kalibrasi) ---
AMBANG_KIRI = 6
AMBANG_KANAN = 6

# --- Recovery garis hilang ---
RECOVERY_TIMEOUT_MS = 2000   # putar cari garis maks 2 detik, lalu berhenti
BELOK_DURASI_MS = 400        # durasi putar 90° aproksimasi di persimpangan

# --- Simulator tambahan (tidak ada di robot asli) ---
SENSOR_JARAK_DEPAN = 22      # px, jarak sensor dari pusat robot
SENSOR_LEBAR = 26            # px, jarak antara sensor kiri-kanan
TRACK_WIDTH_PX = 40          # px, jarak roda kiri-kanan (diferensial)
SIM_SKALA = 0.7              # px per unit speed per detik
SENSOR_SKALA = 20.0          # nilai maksimum sensor (mirip rentang kalibrasi 0-20)
ALPHA_SENSOR = 0.65          # filter eksponensial bacaan sensor
