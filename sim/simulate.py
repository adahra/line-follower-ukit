# simulate.py — loop utama simulator, setia pada state machine LineFollowerFin.ino.
#
# Pakai:
#   python3 simulate.py                       # run headless + lap summary
#   python3 simulate.py --render out.mp4      # run + rekam video
#   python3 simulate.py --kp 7 --kd 2.0       # eksperimen gain
#   python3 simulate.py --durasi 60           # detik simulasi
import argparse
import math
import os

import numpy as np

from robot import (Penggerak, SensorGaris, fisika_step, sensor_pos, noise,
                   set_gain, SPEED_MAKS_DEFAULT, SPEED_MIN_DEFAULT,
                   SERVO_MAKS_DIPAKAI, BELOK_TIMEOUT_MS, RECOVERY_TIMEOUT_MS)
from track import Track, SPEED_TO_V, WHEELBASE

DT = 0.01  # detik, setia pada PID_INTERVAL_MS = 10 ms


class Simulasi:
    def __init__(self, durasi=60.0):
        self.track = Track()
        self.durasi = durasi
        self.gerak = Penggerak()
        self.sensor = SensorGaris()
        self.jalan = True
        self.recovery_mulai = None
        self.telemetry = []  # (t, err, pid, dasar, vk, vn, bk, bn)
        # Lap ala LapTimer: min 3 detik per lap (LAP_MIN_LAP_MS).
        self.lap_min_ms = 3000
        self.laps = []
        self.lap_mulai = 0.0
        self.arc = 0.0
        self.idx_terakhir = None

        # Start tepat di garis (loop[0]), hadap searah tangent loop.
        # (Di lapangan operator meletakkan robot di atas garis; recovery
        # pivot hanya bisa berputar di tempat, tidak bisa translasi.)
        d = self.track.loop[5] - self.track.loop[0]
        th0 = math.atan2(d[1], d[0])
        # offset 3 mm ke kiri garis: robot mulai dalam mode PID (satu sensor di pita)
        nx, ny = -math.sin(th0), math.cos(th0)
        self.pose = (self.track.loop[0][0] + 0.003 * nx,
                     self.track.loop[0][1] + 0.003 * ny, th0)
        self.led = "normal"

    # --- baca sensor dengan derau ---
    def baca_sensor(self):
        pk = sensor_pos(self.pose, True)
        pn = sensor_pos(self.pose, False)
        rk = self.track.baca_mentah(pk) + noise()
        rn = self.track.baca_mentah(pn) + noise()
        return self.sensor.baca(rk, rn)

    # --- manuver blocking, setia pada penggerak.cpp ---
    def maju_manual(self, speed, durasi):
        t_akhir = self.t + durasi / 1000.0
        while self.t < t_akhir:
            self.pose = fisika_step(self.pose, speed, speed, DT)
            self.t += DT

    def _pivot(self, ccw, cek, t0):
        """Belok 90: 300 ms kick lalu pivot sampai sensor target melihat garis.

        Setia pada belokKiri90/belokKanan90: timeout BELOK_TIMEOUT_MS.
        """
        t_akhir = self.t + BELOK_TIMEOUT_MS / 1000.0
        kick = self.t + 0.3
        while self.t < t_akhir:
            speed = int(SPEED_MIN_DEFAULT * 0.8)
            # pivot: roda berlawanan arah, v linier ~0
            om = (1 if ccw else -1) * speed * SPEED_TO_V * 2.0 / WHEELBASE
            x, y, th = self.pose
            self.pose = (x, y, th + om * DT)
            self.t += DT
            if self.t > kick:
                bk, bn = self.baca_sensor()
                if cek(bk, bn):
                    return True
        return False  # timeout: garis tak ketemu

    def belok_kiri90(self):
        return self._pivot(True, lambda bk, bn: bk >= self.sensor.ambang_kiri, self.t)

    def belok_kanan90(self):
        return self._pivot(False, lambda bk, bn: bn >= self.sensor.ambang_kanan, self.t)

    def putar_cari_garis(self, last_err, t_mulai):
        """Lost-line recovery: putar ke arah lastErr, timeout -> berhenti."""
        t_akhir = t_mulai + RECOVERY_TIMEOUT_MS / 1000.0
        while self.t < t_akhir:
            speed = int(SPEED_MIN_DEFAULT * 0.8)
            ccw = last_err > 0  # konsisten putarCariGaris: lastErr>0 = arah "kiri"
            om = (1 if ccw else -1) * speed * SPEED_TO_V * 2.0 / WHEELBASE
            x, y, th = self.pose
            self.pose = (x, y, th + om * DT)
            self.t += DT
            bk, bn = self.baca_sensor()
            if bk >= self.sensor.ambang_kiri or bn >= self.sensor.ambang_kanan:
                return True
        self.jalan = False
        self.led = "merah"
        return False

    # --- lap timer via akumulasi arc-length (tahan terhadap jump indeks) ---
    def cek_lap(self):
        idx = self.track.indeks_loop(self.pose[:2])
        if self.idx_terakhir is not None:
            d = idx - self.idx_terakhir
            n = self.track.n_loop
            # hanya terima pergeseran kecil; lompatan besar (pivot) diabaikan
            if 0 < abs(d) < n * 0.02:
                self.arc += d / n * self.track.panjang_loop()
                if self.arc >= self.track.panjang_loop():
                    self.arc -= self.track.panjang_loop()
                    ms = (self.t - self.lap_mulai) * 1000
                    if ms >= self.lap_min_ms:
                        self.laps.append(ms)
                    self.lap_mulai = self.t
        self.idx_terakhir = idx

    # --- loop utama: setia pada loop() di .ino ---
    def run(self, on_frame=None):
        self.t = 0.0
        prev_pid_ok = True
        while self.t < self.durasi and self.jalan:
            bk, bn = self.baca_sensor()
            ak, an = self.sensor.ambang_kiri, self.sensor.ambang_kanan

            if bk >= ak and bn >= an:
                # --- KONDISI A: persimpangan ---
                self.led = "putih"
                self.maju_manual(SPEED_MIN_DEFAULT, 150)
                bk, bn = self.baca_sensor()
                if bk > bn:
                    self.belok_kanan90()
                elif bn > bk:
                    self.belok_kiri90()
                else:
                    self.maju_manual(SPEED_MIN_DEFAULT, 100)
                self.led = "normal"
            elif bk < ak and bn < an:
                # --- KONDISI B: lost line recovery (NON-BLOCKING, setia .ino):
                # set servo pivot sekali per loop, loop tetap jalan; berhenti
                # total hanya setelah 2 dtk tanpa menemukan garis.
                if self.recovery_mulai is None:
                    self.recovery_mulai = self.t
                self.led = "magenta"
                if self.t - self.recovery_mulai > RECOVERY_TIMEOUT_MS / 1000.0:
                    self.led = "merah"
                    self.jalan = False
                    break
                ccw = self.gerak.last_error > 0
                om = (1 if ccw else -1) * SPEED_MIN_DEFAULT * SPEED_TO_V * 2.0 / WHEELBASE
                x, y, th = self.pose
                self.pose = (x, y, th + om * DT)
            else:
                # --- KONDISI C: PID tracer ---
                self.led = "normal"
                dari_recovery = self.recovery_mulai is not None
                self.recovery_mulai = None
                if dari_recovery:
                    self.gerak.reset_integral_setelah_recovery((bk - bn) / 2.0)
                if self.gerak.hitung(bk, bn, self.t, paksa=dari_recovery):
                    self.pose = fisika_step(self.pose, self.gerak.vk,
                                            self.gerak.vn, DT)
                else:
                    self.pose = fisika_step(self.pose, *self.gerak.terapkan_terakhir(), DT)
                self.cek_lap()
                prev_pid_ok = True

            self.telemetry.append((self.t, self.gerak.error, self.gerak.pid,
                                   self.gerak.dasar, self.gerak.vk,
                                   self.gerak.vn, bk, bn))
            if on_frame is not None:
                on_frame(self)
            self.t += DT
        return self


def ringkas(sim):
    laps = ", ".join(f"{ms/1000:.2f}s" for ms in sim.laps) or "-"
    out = sim.gerak
    if sim.laps:
        best = min(sim.laps)
        return f"Lap: [{laps}]  best={best/1000:.2f}s  akhir t={sim.t:.1f}s"
    status = "SELESAI (durasi habis)" if sim.jalan else "BERHENTI (garis hilang)"
    return f"Tidak ada lap penuh ({status}) t={sim.t:.1f}s"


# --- render video ---
def render(sim, path, fps=25, tangkap_per=2):
    """Rekam video dari run(): butuh sim.pose_history (lihat main)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter

    poses = sim.pose_history
    loop = sim.track.loop
    fig, ax = plt.subplots(figsize=(6, 6), dpi=110)
    ax.plot(loop[:, 0], loop[:, 1], color="0.15", lw=3)
    ax.set_aspect("equal")
    ax.set_title("Simulasi line follower uKit (port config.h)")
    m = 1.4 + 0.42 + 0.4
    ax.set_xlim(-m, m)
    ax.set_ylim(-m, m)

    arah, = ax.plot([], [], "r-", lw=2)
    sensor_k, = ax.plot([], [], "o", ms=5, color="tab:blue")
    sensor_n, = ax.plot([], [], "o", ms=5, color="tab:orange")
    jejak, = ax.plot([], [], "-", lw=0.6, color="0.6")
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                   family="monospace", fontsize=8)
    writer = FFMpegWriter(fps=fps, bitrate=2400)

    n_frame = len(poses) // tangkap_per
    with writer.saving(fig, path, dpi=110):
        for i in range(n_frame):
            idx = i * tangkap_per
            x, y, th = poses[idx]
            t, err, pid, dasar, vk, vn, bk, bn = sim.telemetry[min(idx, len(sim.telemetry) - 1)]
            arah.set_data([x, x + 0.12 * math.cos(th)],
                          [y, y + 0.12 * math.sin(th)])
            jejak.set_data([p[0] for p in poses[max(0, idx - 300):idx + 1]],
                           [p[1] for p in poses[max(0, idx - 300):idx + 1]])
            pk = sensor_pos((x, y, th), True)
            pn = sensor_pos((x, y, th), False)
            sensor_k.set_data([pk[0]], [pk[1]])
            sensor_n.set_data([pn[0]], [pn[1]])
            info.set_text(f"t={t:5.1f}s err={err:+5.2f} pid={pid:+6.2f}\n"
                          f"dasar={dasar:3.0f} kiri={vk:3.0f} kanan={vn:3.0f}\n"
                          f"senK={bk:2d} senN={bn:2d} laps={len(sim.laps)}")
            writer.grab_frame()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--durasi", type=float, default=90.0)
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--render", type=str, default=None, help="path output .mp4")
    args = ap.parse_args()

    set_gain(args.kp, args.ki, args.kd)
    sim = Simulasi(durasi=args.durasi)
    if args.render:
        sim.pose_history = []

        def catat(s):
            sim.pose_history.append(s.pose)
        sim.run(on_frame=catat)
        render(sim, args.render)
    else:
        sim.run()

    print(ringkas(sim))
    # CSV ala telemetri robot: t,err,pid,dasar,kiri,kanan,senK,senN
    csv_path = os.environ.get("SIM_CSV")
    if csv_path:
        with open(csv_path, "w") as f:
            f.write("t,err,pid,dasar,kiri,kanan,senK,senN\n")
            for row in sim.telemetry:
                f.write(",".join(f"{v:.3f}" if isinstance(v, float) else str(v)
                                 for v in row) + "\n")
        print(f"CSV: {csv_path}")


if __name__ == "__main__":
    main()
