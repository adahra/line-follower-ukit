# Non-Blocking 90° Turns and Intersection State Machine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Menghilangkan blocking `delay()` dan `while()` pada saat robot melewati persimpangan dan belok 90° dengan mengimplementasikan sub-state machine non-blocking di `loop()`.

**Architecture:** Status robot diperluas dengan `SubState` (`SUBSTATE_TRACKING`, `SUBSTATE_INTER_CREEP`, `SUBSTATE_TURNING`). Manuver maju perlahan dan belok 90° dieksekusi secara asinkron berbasis timer `millis()` dalam siklus `loop()`, sehingga safety checks (`aman.halangan()`, `aman.miring()`) dan UI tetap aktif setiap siklus.

**Tech Stack:** Arduino AVR C++ (Target ATMega2560 / uKit Explore 2).

**Spec:** `docs/superpowers/specs/2026-11-14-non-blocking-turns-design.md`

## Global Constraints
- Target platform: ATMega2560 (`arduino:avr:mega`).
- Sisa RAM dinamis harus dijaga di atas 1500 bytes (hindari alokasi dinamis).
- Safety checks (`aman.halangan()`, `aman.miring()`, `aman.bateraiKritis()`) harus tetap dievaluasi di setiap putaran loop.
- Fungsi dan variabel harus konsisten dengan konvensi penamaan yang ada (bahasa Indonesia pada domain penggerak/status).

## Review Focus
1. **Safety during creep:** Obstacle terdeteksi saat robot sedang di `SUBSTATE_INTER_CREEP`. Robot harus segera berhenti dan tidak melanjutkan belokan.
2. **Safety during turn:** Robot diangkat/miring saat di `SUBSTATE_TURNING`. Motor harus mati segera.
3. **Turn timeout:** Garis baru tidak ditemukan saat belok 90° karena keluar lintasan. Timer harus timeout pada `BELOK_TIMEOUT_MS` (1500ms) dan kembali ke `SUBSTATE_TRACKING` / recovery tanpa hang.
4. **Blind time debounce:** Sensor langsung membaca garis lama dalam <250ms saat mulai berputar. Harus diabaikan sampai blind time lewat.
5. **Mode RACING/LEARNING consistency:** Prediksi aksi persimpangan tetap tereksekusi dan tercatat ke `Mapper` tanpa duplikasi per persimpangan.

---

### Task 1: Status SubState Definition

**Files:**
- Modify: `LineFollowerFin/status.h`
- Modify: `LineFollowerFin/fwd.h` (jika dibutuhkan forward declaration `ActionType` atau `SubState`)

**Interfaces:**
- Produces:
  - `enum SubState { SUBSTATE_TRACKING = 0, SUBSTATE_INTER_CREEP = 1, SUBSTATE_TURNING = 2 };`
  - `Status::subState`
  - `Status::subStateMulai`
  - `Status::subStateAksi`
  - `Status::targetAmbangBelok`

- [ ] **Step 1: Definisikan enum `SubState` dan tambahkan field pada `struct Status` di `LineFollowerFin/status.h`**
  Pastikan `ActionType` dikenal (include `mapping.h` atau forward decl).
- [ ] **Step 2: Verifikasi compile**
  Jalankan `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin`
- [ ] **Step 3: Commit perubahan**
  `git commit -am "feat: add SubState and non-blocking fields to Status"`

---

### Task 2: Non-blocking Motor Movement Methods

**Files:**
- Modify: `LineFollowerFin/penggerak.h`
- Modify: `LineFollowerFin/penggerak.cpp`

**Interfaces:**
- Produces:
  - `void Penggerak::maju(int speed);`
  - `void Penggerak::putarKiri(int speed);`
  - `void Penggerak::putarKanan(int speed);`

- [ ] **Step 1: Deklarasikan metode di `LineFollowerFin/penggerak.h`**
- [ ] **Step 2: Implementasikan metode di `LineFollowerFin/penggerak.cpp` tanpa `delay()`**
  `maju`: servo 4,2 dir 1, servo 1,3 dir 0.
  `putarKiri`: servo 4,2 dir 0, servo 1,3 dir 0.
  `putarKanan`: servo 4,2 dir 1, servo 1,3 dir 1.
- [ ] **Step 3: Verifikasi compile**
  Jalankan `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin`
- [ ] **Step 4: Commit perubahan**
  `git commit -am "feat: add non-blocking motor primitives to Penggerak"`

---

### Task 3: State Machine Persimpangan & Belok di `loop()`

**Files:**
- Modify: `LineFollowerFin/LineFollowerFin.ino`

**Interfaces:**
- Consumes:
  - `status.subState`, `status.subStateMulai`, `status.subStateAksi`, `status.targetAmbangBelok`
  - `gerak.maju()`, `gerak.putarKiri()`, `gerak.putarKanan()`, `gerak.resetHeading()`
  - `sensor.bacaKiri()`, `sensor.bacaKanan()`

- [ ] **Step 1: Reset substate saat robot berhenti di `LineFollowerFin.ino`**
  Ketika `!status.jalan`, pastikan `status.subState = SUBSTATE_TRACKING`.
- [ ] **Step 2: Ubah cabang `KONDISI A: DETEKSI PERSEMPATAN / PERTIGAAN` dan handling `subState`**
  - Jika `status.subState == SUBSTATE_INTER_CREEP`:
    Periksa apakah `millis() - status.subStateMulai >= 150`. Jika ya, evaluasi arah belokan (selisih sensor atau prediksi RACING). Jika belok, ubah ke `SUBSTATE_TURNING`. Jika lurus, kembali ke `SUBSTATE_TRACKING`.
  - Jika `status.subState == SUBSTATE_TURNING`:
    Cek blind time 250 ms. Jika sudah lewat 250 ms, periksa sensor target terhadap `targetAmbangBelok`. Jika ketemu atau timeout `BELOK_TIMEOUT_MS`, panggil `resetHeading()`, simpan event learning, dan kembali ke `SUBSTATE_TRACKING`.
  - Deteksi awal persimpangan (`bacaKiri >= ambangKiriEfektif && bacaKanan >= ambangKananEfektif`) hanya memicu transisi saat `status.subState == SUBSTATE_TRACKING`.
- [ ] **Step 3: Verifikasi compile main sketch**
  Jalankan `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin`
- [ ] **Step 4: Commit perubahan**
  `git commit -am "refactor: implement non-blocking intersection and turn state machine"`

---

### Task 4: Verifikasi Menyeluruh & Regresi Test Sketches

**Files:**
- Test: `LineFollowerFin/tests/TestLapTimer/`
- Test: `LineFollowerFin/tests/TestBluetooth/`
- Test: `LineFollowerFin/tests/TestPathPlanner/`

- [ ] **Step 1: Jalankan kompilasi untuk seluruh test sketch**
  `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin/tests/TestLapTimer`
  `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin/tests/TestBluetooth`
  `arduino-cli compile --fqbn arduino:avr:mega --libraries "$HOME/Arduino/libraries" LineFollowerFin/tests/TestPathPlanner`
- [ ] **Step 2: Pastikan ukuran RAM dan flash memenuhi batas aman**
  Memory dynamic usage harus < 85% untuk kestabilan stack ATMega2560.
