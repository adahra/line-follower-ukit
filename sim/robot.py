# robot.py — port logika kontrol dari LineFollowerFin (penggerak.cpp, config.h).
# Semua konstanta disalin dari config.h supaya perilakunya setia pada kode asli.
import math
import random

# --- geometri sensor/fisika (dari track.py, di-copy agar tidak circular) ---
SENSOR_OFFSET_LATERAL = 0.013
SENSOR_AHEAD = 0.14
SPEED_TO_V = 0.004  # servo unit -> m/s (150 = 0.6 m/s)
WHEELBASE = 0.16
LINE_NOISE_STD = 0.3

# --- config.h ---
FAKTOR_REM_ADAPTIF = 0.35
BATAS_INTEGRAL = 50
KECEPATAN_TURBO = 120
LURUS_THRESHOLD = 3.0
LURUS_TIME_REQUIRED = 500
TURBO_RAMP_SPEED = 1
SERVO_MIN_EFEKTIF = 25
SERVO_MAKS_DIPAKAI = 150
SLEW_MAKS_PER_LOOP = 14
BELOK_TIMEOUT_MS = 1500
RECOVERY_TIMEOUT_MS = 2000
ALPHA_SENSOR = 0.4
PID_INTERVAL_MS = 10
RECOVERY_TIMEOUT_MS = 2000
# --- config.cpp (default awal) ---
KP_DEFAULT, KI_DEFAULT, KD_DEFAULT = 5.0, 0.1, 1.5
SPEED_MAKS_DEFAULT, SPEED_MIN_DEFAULT = 110, 55
AMBANG_DEFAULT = 8

PID_MS = PID_INTERVAL_MS / 1000.0


def constrain(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class SensorGaris:
    """Port SensorGaris: filter eksponensial + ambang per sisi."""

    def __init__(self):
        self.filt_kiri = -1
        self.filt_kanan = -1
        self.ambang_kiri = AMBANG_DEFAULT
        self.ambang_kanan = AMBANG_DEFAULT

    def baca_halus(self, mentah, sisi_kiri):
        filt = self.filt_kiri if sisi_kiri else self.filt_kanan
        if filt < 0:
            filt = mentah
        else:
            filt += ALPHA_SENSOR * (mentah - filt)
        if sisi_kiri:
            self.filt_kiri = filt
        else:
            self.filt_kanan = filt
        return int(filt + 0.5)

    def baca(self, raw_kiri, raw_kanan):
        return (self.baca_halus(raw_kiri, True),
                self.baca_halus(raw_kanan, False))


class Penggerak:
    """Port Penggerak::hitung — PID adaptif + turbo + deadband + slew.

    Keluaran: (speed_kiri, speed_kanan) 0-150, dikonsumsi fisika.
    """

    def __init__(self):
        self.reset()
        self.speed_maks = SPEED_MAKS_DEFAULT
        self.speed_min = SPEED_MIN_DEFAULT

    def reset(self):
        self.error = self.last_error = 0.0
        self.I = self.P = self.D = self.pid = 0.0
        self.prev_k = self.prev_n = 0
        self.vk = self.vn = 0
        self.pid_terakhir = -1.0
        self.waktu_lurus = 0.0
        self.turbo = 0
        self.dasar = SPEED_MAKS_DEFAULT

    def reset_integral_setelah_recovery(self, error_baru):
        self.I = 0.0
        self.last_error = error_baru

    def hitung(self, baca_kiri, baca_kanan, now, paksa=False):
        if not paksa and (now - self.pid_terakhir) < PID_MS:
            return False
        self.pid_terakhir = now

        self.error = (baca_kiri - baca_kanan) / 2.0
        self.P = self.error
        self.I = constrain(self.I + self.error, -BATAS_INTEGRAL, BATAS_INTEGRAL)
        self.D = self.error - self.last_error
        self.pid = KP * self.P + KI * self.I + KD * self.D
        self.last_error = self.error

        # Turbo: lurus lama -> naikkan speed gradual, tikungan -> reset.
        if abs(self.error) < LURUS_THRESHOLD:
            if self.waktu_lurus == 0:
                self.waktu_lurus = now
            elif (now - self.waktu_lurus) > LURUS_TIME_REQUIRED / 1000.0:
                self.turbo = min(self.turbo + TURBO_RAMP_SPEED, KECEPATAN_TURBO)
        else:
            self.waktu_lurus = 0
            self.turbo = 0

        faktor = int(abs(self.pid) * FAKTOR_REM_ADAPTIF)
        target = self.speed_maks + self.turbo - faktor
        self.dasar = constrain(target, self.speed_min, SERVO_MAKS_DIPAKAI)

        vk = self.dasar - self.pid
        vn = self.dasar + self.pid
        vk = constrain(vk, 0, SERVO_MAKS_DIPAKAI)
        vn = constrain(vn, 0, SERVO_MAKS_DIPAKAI)
        vk = self._deadband(vk)
        vn = self._deadband(vn)
        vk = self._slew(vk, self.prev_k)
        vn = self._slew(vn, self.prev_n)
        self.prev_k, self.prev_n = vk, vn
        self.vk, self.vn = vk, vn
        return True

    def terapkan_terakhir(self):
        """Di antara interval PID, pertahankan speed terakhir."""
        return self.prev_k, self.prev_n

    @staticmethod
    def _deadband(s):
        if s == 0:
            return 0
        return max(s, SERVO_MIN_EFEKTIF)

    @staticmethod
    def _slew(target, prev):
        d = target - prev
        if d > SLEW_MAKS_PER_LOOP:
            return prev + SLEW_MAKS_PER_LOOP
        if d < -SLEW_MAKS_PER_LOOP:
            return prev - SLEW_MAKS_PER_LOOP
        return target


# Gain dipakai lewat variabel modul supaya eksperimen bisa menggantinya.
KP, KI, KD = KP_DEFAULT, KI_DEFAULT, KD_DEFAULT


def set_gain(kp=None, ki=None, kd=None):
    global KP, KI, KD
    KP = KP_DEFAULT if kp is None else kp
    KI = KI_DEFAULT if ki is None else ki
    KD = KD_DEFAULT if kd is None else kd


def sensor_pos(pose, sisi_kiri):
    """Posisi dunia sensor kiri/kanan dari pose (x, y, theta)."""
    x, y, th = pose
    lat = SENSOR_OFFSET_LATERAL if sisi_kiri else -SENSOR_OFFSET_LATERAL
    # kiri = +90 derajat dari arah hadap (CCW positif)
    sx = x + SENSOR_AHEAD * math.cos(th) - lat * math.sin(th)
    sy = y + SENSOR_AHEAD * math.sin(th) + lat * math.cos(th)
    return sx, sy


def fisika_step(pose, speed_kiri, speed_kanan, dt):
    """Unicycle: dari kecepatan roda (servo unit) ke pose baru.

    error > 0 (sensor kiri lebih kuat) harus belok kiri (CCW):
    vk = dasar - pid < vn -> omega = (vn - vk)/wheelbase > 0 = CCW.
    """
    x, y, th = pose
    vl = speed_kiri * SPEED_TO_V
    vr = speed_kanan * SPEED_TO_V
    v = (vl + vr) / 2.0
    omega = (vr - vl) / WHEELBASE
    th += omega * dt
    x += v * math.cos(th) * dt
    y += v * math.sin(th) * dt
    return (x, y, th)


def noise():
    return random.gauss(0.0, LINE_NOISE_STD)
