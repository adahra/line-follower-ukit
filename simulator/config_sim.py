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

FAKTOR_REM_ADAPTIF = 0.5   # dasar = maks - abs(PID) * 0.5
BATAS_INTEGRAL = 50        # I dipenjit ±50 (anti windup)

# --- Batas servo 360 (open-loop) ---
SERVO_MIN_EFEKTIF = 25     # deadband: speed 1-24 dipaksa ke 25
SERVO_MAKS_DIPAKAI = 150   # derate dari 255

# --- Slew + interval ---
SLEW_MAKS_PER_LOOP = 12    # maksimum perubahan speed per loop
PID_INTERVAL_MS = 10       # interval PID konstan (ms)
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
