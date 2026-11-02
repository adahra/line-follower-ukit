import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import numpy as np


class Track:
    """Lintasan: gambar PIL (putih dasar, garis hitam = linya) ditampilkan di canvas.

    Bisa load gambar (PNG/JPG), gambar manual via klik-drag, atau gunakan garis
    default. nilai_piksel(x, y) mengembalikan kekelamatan piksel (0 putih .. 255 hitam)
    yang digunakan sensor IR simulasi.
    """

    def __init__(self, canvas, width=800, height=600):
        self.canvas = canvas
        self.width = width
        self.height = height
        self.image = Image.new("RGB", (width, height), (255, 255, 255))
        self.tk_image = None
        self.bg_id = None
        self.garis_default()
        self.refresh()

    def refresh(self):
        self.tk_image = ImageTk.PhotoImage(self.image)
        if self.bg_id is None:
            self.bg_id = self.canvas.create_image(
                0, 0, image=self.tk_image, anchor="nw"
            )
        else:
            self.canvas.itemconfig(self.bg_id, image=self.tk_image)

    def clear(self, putih=True):
        warna = (255, 255, 255) if putih else (0, 0, 0)
        self.image = Image.new("RGB", (self.width, self.height), warna)
        self.refresh()

    def garis_default(self):
        self.clear(True)
        d = ImageDraw.Draw(self.image)
        cx, cy = self.width // 2, self.height // 2
        d.ellipse([cx - 220, cy - 140, cx + 220, cy + 140], outline=(0, 0, 0), width=12)
        d.line([cx - 220, cy, cx + 220, cy], fill=(0, 0, 0), width=12)
        self.refresh()

    def load(self, path):
        img = Image.open(path).convert("RGB")
        img = img.resize((self.width, self.height))
        self.image = img
        self.refresh()

    def save(self, path):
        self.image.save(path)

    def nilai_piksel(self, x, y):
        """Kekelamatan piksel 0..255 (hitam tinggi). Posisi dilingkari (wrap)
        agar simulasi berkelanjutan di tepi kanvas."""
        ix = int(round(x)) % self.width
        iy = int(round(y)) % self.height
        a = np.array(self.image)
        x0 = max(0, ix - 1)
        x1 = min(self.width, ix + 2)
        y0 = max(0, iy - 1)
        y1 = min(self.height, iy + 2)
        region = a[y0:y1, x0:x1].mean(axis=(0, 1))
        brightness = float(region.mean())
        return 255.0 - brightness

    def gambar_lini(self, x1, y1, x2, y2, lebar=8):
        d = ImageDraw.Draw(self.image)
        d.line([x1, y1, x2, y2], fill=(0, 0, 0), width=lebar)
        self.refresh()
