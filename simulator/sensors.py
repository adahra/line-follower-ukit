import math

import config_sim as cfg


class SensorGaris:
    """Simulasi sensor IR garis.

    - baca_kiri()  -> sensor ID 2 (kiri)
    - baca_kanan() -> sensor ID 1 (kanan)

    Nilai 0 (putih) .. SENSOR_SKALA (hitam), difilter eksponensial dengan
    ALPHA_SENSOR agar satu bacaan lira tidak langsung menjadi koreksi PID penuh,
    mirip bacaHalus() di sensor_garis.cpp.
    """

    def __init__(self, track):
        self.track = track
        self.filt_kiri = -1.0   # ID 2
        self.filt_kanan = -1.0  # ID 1
        self.pos_kiri = (0.0, 0.0)
        self.pos_kanan = (0.0, 0.0)

    def reset_filter(self):
        self.filt_kiri = -1.0
        self.filt_kanan = -1.0

    def update_posisi(self, x, y, heading):
        """Hitung posisi sensor kiri/kanan berdasarkan posisi dan heading robot.

        Heading: derajat, 0 = timur (+x canvas), positif searah jarum jam.
        """
        rad = math.radians(heading)
        fx = math.cos(rad)  # vektor depan x
        fy = math.sin(rad)  # vektor depan y (canvas y ke bawah)
        # normal ke kiri (90 derajat CCW matematis = ke atas canvas saat menghadap timur)
        lx = fy
        ly = -fx
        setengah = cfg.SENSOR_LEBAR / 2.0
        depan = cfg.SENSOR_JARAK_DEPAN
        self.pos_kiri = (
            x + fx * depan + lx * setengah,
            y + fy * depan + ly * setengah,
        )
        self.pos_kanan = (
            x + fx * depan - lx * setengah,
            y + fy * depan - ly * setengah,
        )

    def baca_kiri(self):
        return self._halus(2, *self.pos_kiri)

    def baca_kanan(self):
        return self._halus(1, *self.pos_kanan)

    def nilai_mentah(self, x, y):
        return self.track.nilai_piksel(x, y)

    def _halus(self, id_sensor, x, y):
        mentah = self.nilai_mentah(x, y) * (cfg.SENSOR_SKALA / 255.0)
        filt = self.filt_kiri if id_sensor == 2 else self.filt_kanan
        if filt < 0:
            filt = mentah
        else:
            filt += cfg.ALPHA_SENSOR * (mentah - filt)
        if id_sensor == 2:
            self.filt_kiri = filt
        else:
            self.filt_kanan = filt
        return int(filt + 0.5)
