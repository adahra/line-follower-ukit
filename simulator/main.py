#!/usr/bin/env python3
"""Simulator logika LineFollowerFin berbasis Tkinter (bukan emulasi hex).

Jalankan: python3 simulator/main.py

Mirip perilaku LineFollowerFin.ino:
- State machine: persimpangan -> garis hilang (recovery 2000 ms) -> PID normal.
- PID: error=(kiri-kanan)/2, I clamp ±50, dasar=maks-|PID|*0.5, deadband 25, slew ±12/loop.
- Konvensi: sensor kiri ID 2, kanan ID 1; motor kiri servo 4+2 (dir 1), kanan servo 1+3 (dir 0).
"""
import math
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import config_sim as cfg
from track import Track
from sensors import SensorGaris
from pid import PID
from robot import Robot, ModeOperasi


class SimulatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LineFollowerFin Simulator")
        self.geometry("1150x720")

        self.canvas = tk.Canvas(
            self, width=800, height=600, bg="#cccccc",
            highlightthickness=1, highlightbackground="#888888",
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.panel = ttk.Frame(self, width=300)
        self.panel.pack(side=tk.RIGHT, fill=tk.Y, padx=8, pady=8)
        self.panel.pack_propagate(False)

        self.track = Track(self.canvas)
        self.sensors = SensorGaris(self.track)
        self.pid = PID()
        self.robot = Robot(self.track, self.sensors, self.pid)

        self.t_prev = time.monotonic()
        self.loop_ms = cfg.LOOP_MS
        self._after_id = None
        self._drag_start = None
        self._tmp_id = None

        self._buat_panel()
        self._bind_canvas()

        self.tick()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- panel ----------

    def _buat_panel(self):
        p = self.panel

        ttk.Label(p, text="Mode operasi:").pack(anchor="w")
        self.var_mode = tk.StringVar(value=ModeOperasi.NORMAL)
        cb = ttk.Combobox(
            p,
            textvariable=self.var_mode,
            values=[ModeOperasi.NORMAL, ModeOperasi.LEARNING, ModeOperasi.RACING],
            state="readonly",
        )
        cb.pack(anchor="w", pady=(0, 6), fill=tk.X)
        cb.bind("<<ComboboxSelected>>", lambda e: self.set_mode())

        fr = ttk.Frame(p)
        fr.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(fr, text="Start", command=self.start).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(fr, text="Stop", command=self.stop).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        fr = ttk.Frame(p)
        fr.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(fr, text="Reset", command=self.reset).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(fr, text="Kalibrasi", command=self.kalibrasi).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )

        fr = ttk.LabelFrame(p, text="Track")
        fr.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(fr, text="Muat gambar (PNG/JPG)", command=self.muat_gambar).pack(
            fill=tk.X, padx=4, pady=2
        )
        ttk.Button(fr, text="Bersihkan", command=self.bersihkan).pack(
            fill=tk.X, padx=4, pady=2
        )
        ttk.Button(fr, text="Simpan gambar", command=self.simpan_track).pack(
            fill=tk.X, padx=4, pady=2
        )
        ttk.Button(fr, text="Garis default", command=self.garis_default).pack(
            fill=tk.X, padx=4, pady=2
        )

        self.fr_debug = ttk.LabelFrame(p, text="Debug")
        self.fr_debug.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        self.lbl_state = ttk.Label(self.fr_debug, text="State: DIHENTIKAN")
        self.lbl_state.pack(anchor="w", padx=4, pady=1)
        self.lbl_mode = ttk.Label(self.fr_debug, text="Mode: NORMAL")
        self.lbl_mode.pack(anchor="w", padx=4, pady=1)
        self.lbl_sens = ttk.Label(
            self.fr_debug, text="Sensor kiri(ID2): 0   kanan(ID1): 0"
        )
        self.lbl_sens.pack(anchor="w", padx=4, pady=1)
        self.lbl_err = ttk.Label(self.fr_debug, text="Error: 0.00")
        self.lbl_err.pack(anchor="w", padx=4, pady=1)
        self.lbl_pid = ttk.Label(
            self.fr_debug, text="P: 0.00   I: 0.00   D: 0.00   PID: 0.00"
        )
        self.lbl_pid.pack(anchor="w", padx=4, pady=1)
        self.lbl_dasar = ttk.Label(
            self.fr_debug, text="Dasar: 0   Kiri: 0   Kanan: 0"
        )
        self.lbl_dasar.pack(anchor="w", padx=4, pady=1)
        self.lbl_rec = ttk.Label(self.fr_debug, text="Recovery: - ms")
        self.lbl_rec.pack(anchor="w", padx=4, pady=1)

        fr = ttk.LabelFrame(p, text="Tuning live")
        fr.pack(fill=tk.X)
        self.add_slider(fr, "Kp", 0.0, 10.0, cfg.Kp, "{:.1f}",
                        lambda v: setattr(cfg, "Kp", float(v)))
        self.add_slider(fr, "Ki", 0.0, 2.0, cfg.Ki, "{:.2f}",
                        lambda v: setattr(cfg, "Ki", float(v)))
        self.add_slider(fr, "Kd", 0.0, 5.0, cfg.Kd, "{:.1f}",
                        lambda v: setattr(cfg, "Kd", float(v)))
        self.add_slider(fr, "Speed Maks", 60, 150, cfg.kecepatanDasarMaks, "{:.0f}",
                        lambda v: setattr(cfg, "kecepatanDasarMaks", int(v)))
        self.add_slider(fr, "Speed Min", 25, 110, cfg.kecepatanDasarMin, "{:.0f}",
                        lambda v: setattr(cfg, "kecepatanDasarMin", int(v)))
        self.add_slider(fr, "Ambang Kiri", 3, 18, cfg.AMBANG_KIRI, "{:.0f}",
                        lambda v: setattr(cfg, "AMBANG_KIRI", int(v)))
        self.add_slider(fr, "Ambang Kanan", 3, 18, cfg.AMBANG_KANAN, "{:.0f}",
                        lambda v: setattr(cfg, "AMBANG_KANAN", int(v)))

    def add_slider(self, parent, label, frm, to, val, fmt, cmd):
        lbl = ttk.Label(parent, text=fmt.format(val))
        lbl.pack(anchor="w", padx=4, pady=(2, 0))
        var = tk.DoubleVar(value=val)

        def cb(e, v=var, c=cmd, l=lbl, f=fmt):
            c(v.get())
            l.config(text=f.format(v.get()))

        s = ttk.Scale(parent, from_=frm, to=to, variable=var, command=cb)
        s.pack(fill=tk.X, padx=4, pady=(0, 3))

    # ---------- kontrol ----------

    def start(self):
        if self.robot.kalibrasi_aktif:
            return
        self.robot.jalan = True
        self.robot.sub_state = "NORMAL"
        self.robot.recovery_t_masuk = None

    def stop(self):
        self.robot.jalan = False

    def reset(self):
        self.stop()
        self.robot.reset()

    def kalibrasi(self):
        self.robot.mulai_kalibrasi()

    def set_mode(self):
        self.robot.mode = self.var_mode.get()

    # ---------- track ----------

    def muat_gambar(self):
        path = filedialog.askopenfilename(
            filetypes=[("Gambar", "*.png *.jpg *.jpeg *.bmp *.gif"), ("Semua", "*.*")]
        )
        if not path:
            return
        try:
            self.track.load(path)
        except Exception as exc:
            messagebox.showerror("Gagal memuat", str(exc))

    def bersihkan(self):
        self.track.clear(True)

    def simpan_track(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png", filetypes=[("PNG", "*.png")]
        )
        if path:
            self.track.save(path)

    def garis_default(self):
        self.track.garis_default()

    # ---------- gambar manual track di canvas ----------

    def _bind_canvas(self):
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

    def on_press(self, ev):
        self._drag_start = (ev.x, ev.y)
        self._tmp_id = None

    def on_drag(self, ev):
        if self._drag_start is None:
            return
        if self._tmp_id is not None:
            self.canvas.delete(self._tmp_id)
        self._tmp_id = self.canvas.create_line(
            *self._drag_start, ev.x, ev.y, width=8, fill="black", capstyle=tk.ROUND
        )

    def on_release(self, ev):
        if self._drag_start is None:
            return
        if self._tmp_id is not None:
            self.canvas.delete(self._tmp_id)
            self._tmp_id = None
        self.track.gambar_lini(
            self._drag_start[0], self._drag_start[1], ev.x, ev.y, lebar=8
        )
        self._drag_start = None

    # ---------- main loop ----------

    def tick(self):
        now = time.monotonic()
        dt = (now - self.t_prev) * 1000.0
        self.t_prev = now
        if dt <= 0:
            dt = self.loop_ms
        if dt > 60:
            dt = 60
        self.robot.update(dt)
        self.gambar_robot()
        self.update_debug()
        self._after_id = self.after(self.loop_ms, self.tick)

    def gambar_robot(self):
        self.canvas.delete("robot")
        r = self.robot
        L = 36
        W = 22
        pts = [(0, -L / 2), (W / 2, L / 4), (W / 2, L / 2), (-W / 2, L / 2), (-W / 2, L / 4)]
        rad = math.radians(r.heading)
        c = math.cos(rad)
        s = math.sin(rad)
        poly = []
        for px, py in pts:
            poly.append(r.x + px * c - py * s)
            poly.append(r.y + px * s + py * c)
        self.canvas.create_polygon(
            poly, fill="#2b2b2b", outline="black", tags="robot"
        )
        fx = r.x + c * (L / 2 + 5)
        fy = r.y + s * (L / 2 + 5)
        self.canvas.create_line(r.x, r.y, fx, fy, fill="#ffd700", width=2, tags="robot")

        # Sensor dots: kiri ID 2 (hijau), kanan ID 1 (merah); menyala jika >= ambang.
        titik = [
            (r.sensors.pos_kiri, 2, cfg.AMBANG_KIRI, "#00aa00"),
            (r.sensors.pos_kanan, 1, cfg.AMBANG_KANAN, "#aa0000"),
        ]
        for (sx, sy), id_sensor, ambang, warna in titik:
            nilai = r.baca_kiri if id_sensor == 2 else r.baca_kanan
            aktif = r.jalan and nilai >= ambang
            fill = warna if aktif else "#555555"
            self.canvas.create_oval(
                sx - 4, sy - 4, sx + 4, sy + 4, fill=fill, outline="black", tags="robot"
            )
        self.canvas.tag_raise("robot")

    def update_debug(self):
        r = self.robot
        self.lbl_state.config(text="State: {}".format(r.state_label))
        self.lbl_mode.config(text="Mode: {}".format(r.mode))
        self.lbl_sens.config(
            text="Sensor kiri(ID2): {}   kanan(ID1): {}".format(r.baca_kiri, r.baca_kanan)
        )
        self.lbl_err.config(text="Error: {:.2f}".format(r.error))
        self.lbl_pid.config(
            text="P: {:.2f}   I: {:.2f}   D: {:.2f}   PID: {:.2f}".format(
                r.P, r.I, r.D, r.pid_value
            )
        )
        self.lbl_dasar.config(
            text="Dasar: {:.0f}   Kiri: {}   Kanan: {}".format(
                r.kecepatan_dasar, r.speed_kiri, r.speed_kanan
            )
        )
        if r.state_label == "GARIS_HILANG":
            self.lbl_rec.config(
                text="Recovery: {:.0f} ms / 2000".format(r.recovery_elapsed)
            )
        else:
            self.lbl_rec.config(text="Recovery: - ms")

    def on_close(self):
        if self._after_id is not None:
            self.after_cancel(self._after_id)
        self.destroy()


if __name__ == "__main__":
    app = SimulatorApp()
    app.mainloop()
