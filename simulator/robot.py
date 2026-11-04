import math

import config_sim as cfg
from pid import PID


class ModeOperasi:
    NORMAL = "NORMAL"
    LEARNING = "LEARNING"
    RACING = "RACING"


class Robot:
    """Model robot + state machine mirip loop() di LineFollowerFin.ino.

    Konvensi motor (mirip penggerak.cpp::jalankan):
      - kiri  = servo 4 + 2, dir 1
      - kanan = servo 1 + 3, dir 0
    Dalam simulasi dikonversi ke model diferensial dengan signed speed:
      positif = maju, negatif = mundur.
    """

    def __init__(self, track, sensors, pid):
        self.track = track
        self.sensors = sensors
        self.pid = pid
        self.reset()

    def reset(self):
        self.x = 100.0
        self.y = 100.0
        self.heading = 0.0
        self.speed_kiri = 0
        self.speed_kanan = 0
        self.jalan = False
        self.mode = ModeOperasi.NORMAL
        self.kalibrasi_aktif = False
        self.kalibrasi_t0 = 0
        self.sub_state = "NORMAL"   # NORMAL / PERSIMPANGAN / LOST
        self.t_timer = 0.0
        self.t_sub = 0.0
        self.dt_terakhir = cfg.LOOP_MS
        self.recovery_t_masuk = None
        self.intersection_act = None
        self.last_error = 0.0
        self.baca_kiri = 0
        self.baca_kanan = 0
        self.error = 0.0
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.pid_value = 0.0
        self.kecepatan_dasar = 0.0
        self.recovery_elapsed = 0.0
        self.state_label = "DIHENTIKAN"
        self.pid.reset()
        self.sensors.reset_filter()

    # ---------- update utama (dipanggil setiap tick simulator) ----------

    def update(self, dt_ms):
        self.dt_terakhir = dt_ms
        self.t_timer += dt_ms

        if self.kalibrasi_aktif:
            self._update_kalibrasi(dt_ms)
            return

        if not self.jalan:
            self.speed_kiri = 0
            self.speed_kanan = 0
            self.state_label = "DIHENTIKAN"
            return

        self.sensors.update_posisi(self.x, self.y, self.heading)
        self.baca_kiri = self.sensors.baca_kiri()      # ID 2
        self.baca_kanan = self.sensors.baca_kanan()    # ID 1
        ak = cfg.AMBANG_KIRI
        aan = cfg.AMBANG_KANAN
        error_now = (self.baca_kiri - self.baca_kanan) / 2.0

        # --- KONDISI A: persimpangan (kedua sensor >= ambang) ---
        if self.baca_kiri >= ak and self.baca_kanan >= aan:
            if self.sub_state != "PERSIMPANGAN":
                self.sub_state = "PERSIMPANGAN"
                self.t_sub = 0
                self.intersection_act = None
                self.recovery_t_masuk = None
            self.t_sub += dt_ms
            self._persimpangan()
            return

        # --- KONDISI B: garis hilang (kedua sensor < ambang, recovery 2000 ms) ---
        if self.baca_kiri < ak and self.baca_kanan < aan:
            if self.sub_state != "LOST":
                self.sub_state = "LOST"
                self.t_sub = 0
                self.recovery_t_masuk = None
            self.t_sub += dt_ms
            self._lost(error_now)
            return

        # --- KONDISI C: PID normal dengan kecepatan adaptif ---
        dari_recovery = self.sub_state == "LOST"
        self.sub_state = "NORMAL"
        self.t_sub = 0
        self.recovery_t_masuk = None
        if dari_recovery:
            self.pid.reset_integral_setelah_recovery(error_now)
        self._pid(self.baca_kiri, self.baca_kanan)

    # ---------- sub-state: persimpangan ----------

    def _persimpangan(self):
        self.state_label = "PERSIMPANGAN"
        # Maju pendek dulu (mirip majuManual 150 ms), baru baca ulang dan keputusan belok.
        if self.t_sub < 150:
            self.speed_kiri = cfg.kecepatanDasarMin
            self.speed_kanan = cfg.kecepatanDasarMin
        else:
            if self.intersection_act is None:
                # Keputusan belok dari data segar (mirip belokKiri90/belokKanan90).
                if self.baca_kiri > self.baca_kanan:
                    self.intersection_act = "KANAN"   # ACT_RIGHT: belok kanan
                elif self.baca_kanan > self.baca_kiri:
                    self.intersection_act = "KIRI"    # ACT_LEFT: belok kiri
                else:
                    self.intersection_act = "LURUS"
            act = self.intersection_act
            if act == "LURUS":
                self.speed_kiri = cfg.kecepatanDasarMin
                self.speed_kanan = cfg.kecepatanDasarMin
                self.sub_state = "NORMAL"
            elif self.t_sub < 150 + cfg.BELOK_DURASI_MS:
                kec = cfg.kecepatanDasarMin
                if act == "KANAN":
                    self.speed_kiri = -kec
                    self.speed_kanan = +kec
                else:
                    self.speed_kiri = +kec
                    self.speed_kanan = -kec
            else:
                self.sub_state = "NORMAL"
        self._set_motor(self.speed_kiri, self.speed_kanan)

    # ---------- sub-state: lost line ----------

    def _lost(self, error_now):
        self.last_error = error_now
        self.state_label = "GARIS_HILANG"
        if self.recovery_t_masuk is None:
            self.recovery_t_masuk = self.t_timer
        self.recovery_elapsed = self.t_timer - self.recovery_t_masuk

        # Timeout 2000 ms: berhenti total, tunggu operator (mirip penggerak.cpp loop B).
        if self.recovery_elapsed > cfg.RECOVERY_TIMEOUT_MS:
            self.speed_kiri = 0
            self.speed_kanan = 0
            self.jalan = False
            self.state_label = "TIMEOUT"
            self.recovery_t_masuk = None
            self.sub_state = "NORMAL"
            return

        # Putar cari garis ke arah lastError (mirip putarCariGaris).
        kec = cfg.kecepatanDasarMin
        if error_now > 0:
            self.speed_kiri = -kec
            self.speed_kanan = +kec
        else:
            self.speed_kiri = +kec
            self.speed_kanan = -kec
        self._set_motor(self.speed_kiri, self.speed_kanan)

    # ---------- sub-state: PID normal ----------

    def _pid(self, baca_kiri, baca_kanan):
        self.pid.hitung(baca_kiri, baca_kanan)
        dasar = self.pid.kecepatan_dasar()
        tk, tn = self.pid.motor_target()
        self.speed_kiri = self.pid.slew(tk, self.speed_kiri)
        self.speed_kanan = self.pid.slew(tn, self.speed_kanan)
        self.kecepatan_dasar = dasar
        self.error = self.pid.error
        self.P = self.pid.P
        self.I = self.pid.I
        self.D = self.pid.D
        self.pid_value = self.pid.pid_value
        self.state_label = "PID"
        self._gerak()

    # ---------- motor / kinematika ----------

    def _set_motor(self, target_kiri, target_kanan):
        self.speed_kiri = self.pid.slew(self._deadband(target_kiri), self.speed_kiri)
        self.speed_kanan = self.pid.slew(self._deadband(target_kanan), self.speed_kanan)
        self._gerak()

    @staticmethod
    def _deadband(speed):
        if speed == 0:
            return 0
        mag = abs(speed)
        if mag < cfg.SERVO_MIN_EFEKTIF:
            return cfg.SERVO_MIN_EFEKTIF if speed > 0 else -cfg.SERVO_MIN_EFEKTIF
        return speed

    def _gerak(self):
        sk = max(-cfg.SERVO_MAKS_DIPAKAI, min(cfg.SERVO_MAKS_DIPAKAI, self.speed_kiri))
        sn = max(-cfg.SERVO_MAKS_DIPAKAI, min(cfg.SERVO_MAKS_DIPAKAI, self.speed_kanan))
        self.speed_kiri = sk
        self.speed_kanan = sn

        dt = self.dt_terakhir / 1000.0
        skala = cfg.SIM_SKALA
        v = (sk + sn) / 2.0 * skala * dt           # linear (px/detik)
        omega = (sn - sk) * skala / cfg.TRACK_WIDTH_PX * dt  # rad/detik

        self.heading = (self.heading + math.degrees(omega)) % 360.0
        rad = math.radians(self.heading)
        self.x += v * math.cos(rad)
        self.y += v * math.sin(rad)

        # Clamp ke batas kanvas.
        self.x = max(0.0, min(float(self.track.width), self.x))
        self.y = max(0.0, min(float(self.track.height), self.y))

    # ---------- kalibrasi simulasi ----------

    def mulai_kalibrasi(self):
        if self.jalan or self.kalibrasi_aktif:
            return
        self.kalibrasi_aktif = True
        self.kalibrasi_t0 = self.t_timer
        self.maks_kiri = 0
        self.min_kiri = 10**9
        self.maks_kanan = 0
        self.min_kanan = 10**9
        self.state_label = "KALIBRASI"

    def _update_kalibrasi(self, dt_ms):
        elapsed = self.t_timer - self.kalibrasi_t0
        if elapsed > 3000:
            self.kalibrasi_aktif = False
            self.jalan = False
            self.speed_kiri = 0
            self.speed_kanan = 0
            self.sub_state = "NORMAL"
            # Ambang = (maks+min)/2 per sensor, dikunci ke rentang valid 3..18
            # (mirip sensor_garis.cpp::kalibrasi).
            cfg.AMBANG_KIRI = int(max(3, min(18, (self.maks_kiri + self.min_kiri) / 2.0)))
            cfg.AMBANG_KANAN = int(max(3, min(18, (self.maks_kanan + self.min_kanan) / 2.0)))
            self.state_label = "DIHENTIKAN"
            return

        # Putar di tempat selama 3 detik (mirip putarKalibrasi) sambil mencatat min/max.
        self.speed_kiri = -50
        self.speed_kanan = +50
        self._set_motor(self.speed_kiri, self.speed_kanan)
        vk = self.sensors.baca_kiri()
        vn = self.sensors.baca_kanan()
        self.maks_kiri = max(self.maks_kiri, vk)
        self.min_kiri = min(self.min_kiri, vk)
        self.maks_kanan = max(self.maks_kanan, vn)
        self.min_kanan = min(self.min_kanan, vn)
