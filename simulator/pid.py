import config_sim as cfg


class PID:
    """Implementasi PID line tracer mirip Penggerak::hitung() di penggerak.cpp.

    error = (kiri - kanan) / 2
    P = error
    I = I + error, dijepit ±BATAS_INTEGRAL
    D = error - lastError
    PID_value = Kp*P + Ki*I + Kd*D
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.error = 0.0
        self.last_error = 0.0
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.pid_value = 0.0

    def reset_integral_setelah_recovery(self, error_baru):
        """Dipanggil saat kembali dari lost-line: buang hutang integral dan
        samakan lastError agar tak ada spike D di loop pertama."""
        self.I = 0.0
        self.last_error = error_baru

    def hitung(self, baca_kiri, baca_kanan):
        self.error = (baca_kiri - baca_kanan) / 2.0
        self.P = self.error
        self.I += self.error
        self.I = max(-cfg.BATAS_INTEGRAL, min(cfg.BATAS_INTEGRAL, self.I))
        self.D = self.error - self.last_error
        self.pid_value = cfg.Kp * self.P + cfg.Ki * self.I + cfg.Kd * self.D
        self.last_error = self.error
        return True

    def kecepatan_dasar(self):
        """kecepatanDasar = maks - abs(PID)*0.5, dipenjit [min, maks]."""
        return max(
            cfg.kecepatanDasarMin,
            min(cfg.kecepatanDasarMaks,
                cfg.kecepatanDasarMaks - abs(self.pid_value) * cfg.FAKTOR_REM_ADAPTIF),
        )

    def motor_target(self):
        """Kecepatan target motor sebelum deadband/slew (0..SERVO_MAKS_DIPAKAI)."""
        dasar = self.kecepatan_dasar()
        kiri = dasar - self.pid_value
        kanan = dasar + self.pid_value
        kiri = max(0, min(cfg.SERVO_MAKS_DIPAKAI, int(kiri + 0.5)))
        kanan = max(0, min(cfg.SERVO_MAKS_DIPAKAI, int(kanan + 0.5)))
        kiri = self._deadband(kiri)
        kanan = self._deadband(kanan)
        return kiri, kanan

    @staticmethod
    def _deadband(speed):
        if speed == 0:
            return 0
        if speed < cfg.SERVO_MIN_EFEKTIF:
            return cfg.SERVO_MIN_EFEKTIF
        return speed

    def slew(self, target, prev):
        selisih = target - prev
        if selisih > cfg.SLEW_MAKS_PER_LOOP:
            return prev + cfg.SLEW_MAKS_PER_LOOP
        if selisih < -cfg.SLEW_MAKS_PER_LOOP:
            return prev - cfg.SLEW_MAKS_PER_LOOP
        return target
