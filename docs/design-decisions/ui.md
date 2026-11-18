# Keputusan Desain - UI & Input

Dokumentasi keputusan desain dan alasan untuk sistem antarmuka (UI), kontrol manusia, dan parsing command di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk sistem antarmuka (UI) di LineFollowerFin, yang bertanggung jawab untuk kontrol manusia: tombol, tepuk tangan, dan serial command.

**File:** `docs/design-decisions/ui.md`

## 2. Tujuan Sistem UI

### 2.1 Kontrol Manusia
**Alasan:** Operator perlu mengendalikan robot line-following untuk berbagai task: start/stop, tuning parameter, kalibrasi sensor, dan monitoring status.

**Implementasi:**
- Tombol: toggle jalan/berhenti, tuning Kp/Kd, kalibrasi otomatis
- Tepuk tangan: toggle jalan/berhenti via bystander
- Serial command: parsing command untuk kontrol robot

### 2.2 Feedback Pengguna
**Alasan:** Operator perlu feedback visual tentang status robot. Lampu LED, serial output, dan Bluetooth telemetry memberikan feedback.

**Implementasi:**
- Lampu LED: kontrol manual dan otomatis
- Serial output: debugging dan monitoring
- Bluetooth telemetry: monitoring jarak jauh

### 2.3 Parsing Command
**Alasan:** Parsing command serial memfasilitasi kontrol robot yang fleksibel dan remote. Ini memungkinkan operator mengakses semua fitur robot melalui command yang sederhana.

**Implementasi:**
- Command tunggal: start, stop, tune, help
- Command dengan argumen: kp 1.2, ki 1.0, kd 0.8
- Command mode: mode learning, mode racing, mode rc

## 3. Desain Decision Key

### 3.1 UI Non-blocking
**Alasan:** Delay blocking akan mencegah robot melacak garis saat UI merespon. Non-blocking UI memungkinkan robot tetap melacak garis.

**Implementasi:**
- Gunakan cooldown millis() bukan delay()
- Tombol dan tepuk tangan dengan debounce dan cooldown
- Parsing command serial dengan timeout

### 3.2 Cooldown Tombol & Tepuk
**Alasan:** Mencegah false positive dari noise mekanis atau akustik. Cooldown yang berbeda untuk tombol vs tepuk tangan berdasarkan characteristic fisik.

**Implementasi:**
- COOLDOWN_TOMBOL_MS = 300 (cooldown tombol)
- TEPUK_COOLDOWN_MS = 800 (cooldown tepuk tangan)
- TEPUK_DETEKSI_MS = 500 (durasi deteksi tepuk tangan)

### 3.3 Parsing Command Serial
**Alasan:** Parsing command serial fleksibel dengan syntax sederhana. Mendukung perintah tunggal dan perintah dengan argumen.

**Implementasi:**
```cpp
// Split command dan argumen
int spasiIndex = input.indexOf(' ');
String cmd = (spasiIndex > 0) ? input.substring(0, spasiIndex) : input;
String arg = (spasiIndex > 0) ? input.substring(spasiIndex + 1) : "";
```

### 3.4 Combo Tombol
**Alasan:** Kalibrasi otomatis memerlukan koordinasi antar tombol. Menggunakan tombol bersamaan menghindari konflik dengan fungsi lain.

**Implementasi:**
- B1 + B2 dalam 100ms → kalibrasi otomatis
- Magenta LED = kalibrasi aktif
- Reset setelah kalibrasi selesai

### 3.5 Feedback LED
**Alasan:** Lampu LED memberikan feedback visual yang cepat dan mudah. Warna yang berbeda menunjukkan status yang berbeda.

**Implementasi:**
- Merah = berhenti (danger)
- Hijau = jalan (normal)
- Biru = kalibrasi (learning)
- Kuning = tuning (modeSetelKd)
- Cyan = RC (mode rc)

## 4. Desain Trade-offs

### 4.1 Non-blocking vs Blocking UI
**Trade-off:** Non-blocking UI memungkinkan pelacakan garis yang terus berlanjut tapi menambah kompleksitas coding. Blocking UI lebih sederhana tapi mencegah robot melacak garis. Memilih non-blocking untuk performance yang lebih baik.

### 4.2 Cooldown vs Tanpa Cooldown
**Trade-off:** Cooldown mencegah false positive tapi menambah latency. Tanpa cooldown lebih responsif tapi berisiko false positive. Memilih cooldown untuk reliability.

### 4.3 Command Single vs Multi-line
**Trade-off:** Command single line sederhana dan mudah di-parse. Multi-line lebih fleksibel tapi lebih kompleks. Memilih single line untuk simplicity.

## 5. Validasi Desain

### 5.1 Cooldown UI
**Validasi:** Cooldown UI diuji dengan operator yang menekan tombol/tepuk tangan dengan cepat.

**Implementasi:**
- COOLDOWN_TOMBOL_MS = 300 (cukup cepat untuk operator)
- TEPUK_COOLDOWN_MS = 800 (cukup cepat untuk tepuk tangan)

### 5.2 Parsing Command
**Validasi:** Parsing command diuji dengan command yang valid dan tidak valid.

**Implementasi:**
- Command valid: start, stop, kp 1.2, mode learning
- Command tidak valid: kp abc, mode unknown
- Handle command tidak valid dengan graceful

### 5.3 Feedback LED
**Validasi:** Feedback LED diuji dengan operator.

**Implementasi:**
- Merah = danger
- Hijau = normal
- Biru = learning
- Kuning = tuning
- Cyan = RC

## 6. Risiko & Mitigasi

### 6.1 False Positive
**Risiko:** UI memberikan false positive (mis. tombol pendek dianggap sebagai tombol panjang). **Mitigasi:** Cooldown, debounce, validasi.

### 6.2 False Negative
**Risiko:** UI tidak merespon saat operator menekan tombol. **Mitigasi:** Reduce cooldown, tambahkan hysteresis.

### 6.3 Command Injection
**Risiko:** Operator bisa mengirim command yang berbahaya. **Mitigasi:** Validasi command, batasi command yang diizinkan.

## 7. Referensi

1. Arduino UI Library - https://www.arduino.cc/en/Tutorial/UI
2. Button Debounce - https://www.arduino.cc/en/Tutorial/ButtonDebounce
3. Sound Detection - https://www.arduino.cc/en/Tutorial/SoundDetection
4. Serial Communication - https://www.arduino.cc/en/Tutorial/Serial
5. LED Control - https://www.arduino.cc/en/Tutorial/LEDControl

## 8. Catatan Perkembangan

### 8.1 Iterasi UI
- v1.0: UI dasar (tombol start/stop)
- v1.1: Tambah tepuk tangan, cooldown
- v1.2: Parsing command serial, feedback LED
- v1.3: Combo tombol untuk kalibrasi
- v1.4: Command tuning, command mode

### 8.2 Validasi UI
- Uji dengan operator yang menekan tombol
- Uji dengan operator yang tepuk tangan
- Uji dengan command serial
- Uji dengan command yang tidak valid

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/antarmuka.md` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/safety.md` untuk safety