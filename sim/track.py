# track.py — model lintasan + sensor IR untuk simulator line follower.
# Geometri: loop tertutup (oval bergelombang) + satu segmen garis melintang
# (persimpangan) di sisi berlawanan dari start. Satuan meter.
import math
import numpy as np

# --- Konstanta sensor/geometri (perkiraan fisik, bukan dari config.h) ---
SENSOR_OFFSET_LATERAL = 0.013  # sensor kiri/kanan +/- 1.3 cm dari sumbu
SENSOR_AHEAD = 0.12            # sensor di depan poros roda
TAPE_HALF = 0.0095             # setengah lebar pita 1.9 cm
SENSOR_RAMP = 0.015            # lebar transisi baca (m)
SENSOR_FLOOR = 1.0             # bacaan di luar jangkauan
LINE_NOISE_STD = 0.3           # derau bacaan IR (skala 0-20)
WHEELBASE = 0.16               # jarak antar roda (m)
SPEED_TO_V = 0.004             # servo unit -> m/s (speed 150 = 0.6 m/s)


def _main_loop():
    """Polyline loop utama, tertutup, parameter theta 0..2pi."""
    th = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    r = 1.4 + 0.42 * np.sin(3 * th)
    return np.stack([r * np.cos(th), r * np.sin(th)], axis=1)


def _crossing_segment():
    """Garis melintang ~90 derajat di theta=pi (titik (-2, 0))."""
    perp = np.array([0.74, -0.67])
    perp /= np.linalg.norm(perp)
    c = np.array([-2.0, 0.0])
    return np.stack([c - 0.35 * perp, c + 0.35 * perp], axis=0)


class Track:
    def __init__(self):
        self.loop = _main_loop()
        self.cross = _crossing_segment()
        self.seg_all = [self.loop, self.cross]
        self.n_loop = len(self.loop)

    def dist_to(self, p, poly):
        """Jarak titik p ke polyline terdekat (numpy, tanpa loop python)."""
        a = poly[:-1]
        b = poly[1:]
        ap = p - a
        ab = b - a
        t = np.clip((ap * ab).sum(1) / (ab * ab).sum(1), 0, 1)
        proj = a + t[:, None] * ab
        return float(np.min(np.linalg.norm(p - proj, axis=1)))

    def baca_mentah(self, p):
        """Bacaan IR 0-20 ala readInfraredDistance: makin di garis makin besar.

        Model plateau ala sensor reflektif: penuh 20 selama di atas pita,
        turun tajam (sigmoid) ke lantai ~1.5 di luar. Seperti sensor asli,
        error tetap MAKSIMAL selama garis di luar jangkauan -> recovery (KONDISI B)
        punya sinyal kuat untuk menemukan garis lagi.
        """
        d_loop = self.dist_to(p, self.loop)
        d_cross = self.dist_to(p, self.cross)
        d = min(d_loop, d_cross)
        # 20 saat sensor di atas pita (d <= setengah pita), turun ke lantai
        # dalam SENSOR_RAMP. Sensor sejajar +/- 1.3 cm: mengikuti garis = SATU
        # sensor di atas pita (mode PID), kedua-duanya = hanya di persimpangan,
        # keduanya lepas = lost line. Setia pada asumsi loop() di .ino.
        frac = (0.0175 - d) / SENSOR_RAMP
        frac = max(0.0, min(1.0, frac))
        return max(0.0, min(20.0, SENSOR_FLOOR + 19.0 * frac))

    def indeks_loop(self, p):
        """Indeks titik loop terdekat (untuk lap timer)."""
        d2 = ((self.loop - p) ** 2).sum(1)
        return int(np.argmin(d2))

    def panjang_loop(self):
        seg = np.diff(self.loop, axis=0)
        return float(np.linalg.norm(seg, axis=1).sum())
