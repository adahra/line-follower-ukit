# Keputusan Desain - Fitur Opsional

Dokumentasi keputusan desain dan alasan untuk fitur opsional di LineFollowerFin: Fuzzy PID, Heading Correction, dan Path Planner.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk fitur opsional di LineFollowerFin: Fuzzy PID, Heading Correction, dan Path Planner. Fitur ini opsional dan bisa diaktifkan/deaktifkan melalui config.h.

**File:** `docs/design-decisions/optional_features.md`

## 2. Tujuan Fitur Opsional

### 2.1 Fuzzy PID
**Alasan:** Fuzzy PID menyesuaikan gains PID secara dinamis berdasarkan error & delta_error. Ini memberikan kontrol yang lebih robust dibandingkan PID biasa.

**Implementasi:**
- Dynamic gain adjustment berdasarkan error & delta_error
- Input: error (±10), delta_error (±10)
- Output: delta Kp (±5), delta Kd (±2), delta Ki (±0.5)
- Mamdani inference + centroid defuzzification

### 2.2 Heading Correction
**Alasan:** Heading correction berbasis gyro Z memberikan koreksi heading yang akurat. Ini komplementer dengan PID berbasis error sensor.

**Implementasi:**
- Integrasi gyro Z → heading absolut
- Kalibrasi bias gyro untuk kompensasi drift
- Digunakan sebagai lapisan koreksi di atas PID

### 2.3 Path Planner
**Alasan:** Path planning manual grid-based digunakan untuk debugging dan testing. Bukan navigasi otonom.

**Implementasi:**
- Manual grid-based path planning (A*)
- Bukan navigasi otonom (tidak ada odometri)
- Digunakan untuk debugging gerakan robot

## 3. Desain Decision Key

### 3.1 Fuzzy PID vs PID Biasa
**Trade-off:** Fuzzy PID menyesuaikan gains secara dinamis tapi menambah kompleksitas komputasi. PID biasa lebih sederhana tapi kurang robust. Memilih fuzzy PID untuk robustness.

**Implementasi:**
```cpp
// Fuzzy PID
float dKp, dKd, dKi;
fuzzyPid.compute(error, deltaError, dKp, dKd, dKi);
Kp += dKp; Ki += dKi; Kd += dKd;

// PID biasa
float P = error * Kp;
float I = I + error * Ki;
float D = (error - lastError) * Kd / dt;
```

### 3.2 Heading Correction sebagai Lapisan Atas
**Alasan:** Heading correction berbasis gyro Z komplementer dengan PID berbasis error sensor. Ini adalah koreksi steer tambahan, bukan replacement.

**Implementasi:**
```cpp
float headingCorrection = getHeadingError() * HEADING_GAIN;
headingCorrection = constrain(headingCorrection, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
PID_value += headingCorrection;
```

### 3.3 Path Planner Manual
**Alasan:** Path planning manual grid-based digunakan untuk debugging dan testing. Bukan navigasi otonom.

**Implementasi:**
- Grid-based map untuk visualisasi dan debugging
- A* algorithm untuk path finding
- Tidak ada integrasi odometri

## 4. Desain Trade-offs

### 4.1 Fuzzy PID vs PID Biasa
**Trade-off:** Fuzzy PID memberikan penyesuaian yang dinamis tapi menambah kompleksitas komputasi. PID biasa lebih sederhana tapi kurang robust. Memilih fuzzy PID untuk robustness.

### 4.2 Heading Correction vs Tanpa Correction
**Trade-off:** Heading correction berbasis gyro Z memberikan koreksi yang akurat. Tanpa correction lebih sederhana tapi rentan terhadap drift. Memilih heading correction untuk accuracy.

### 4.3 Path Planner Manual vs Otonom
**Trade-off:** Path planning manual memungkinkan debugging yang mudah. Path planning otonom memerlukan odometri dan navigation algorithm. Memilih manual karena bukan kebutuhan kompetisi utama.

## 5. Validasi Desain

### 5.1 Fuzzy PID
**Validasi:** Fuzzy PID diuji dengan robot pada lintasan yang berbeda.

**Implementasi:**
- Error input: -10 hingga 10
- Delta error input: -10 hingga 10
- Output gain adjustment: ±5 untuk Kp, ±2 untuk Kd, ±0.5 untuk Ki

### 5.2 Heading Correction
**Validasi:** Heading correction diuji dengan robot pada heading yang berbeda.

**Implementasi:**
- Gyro Z input: ±2000°/s
- Heading gain: 0.5 (default)
- Maximum offset: 15°

### 5.3 Path Planner
**Validasi:** Path planning diuji dengan robot pada grid yang berbeda.

**Implementasi:**
- Grid width: 20, height: 20
- Cell types: CELL_EMPTY, CELL_LINE, CELL_OBSTACLE
- A* algorithm: heuristic yang efisien

## 6. Risiko & Mitigasi

### 6.1 Fuzzy PID
**Risiko:** Fuzzy PID menambah kompleksitas komputasi. **Mitigasi:** Batasi kompleksitas rule, gunakan interval PID yang tetap.

### 6.2 Heading Correction
**Risiko:** Gyro drift bisa mempengaruhi heading correction. **Mitigasi:** Kalibrasi bias gyro, gunakan gain yang rendah.

### 6.3 Path Planner
**Risiko:** Path planning manual tidak praktis untuk navigasi otonom. **Mitigasi:** Gunakan hanya untuk debugging dan testing.

## 7. Referensi

1. Fuzzy Logic Control - https://www.arduino.cc/en/Tutorial/FuzzyControl
2. Gyro - https://www.arduino.cc/en/Tutorial/Gyro
3. A* Algorithm - https://www.arduino.cc/en/Tutorial/A*
4. Path Planning - https://www.arduino.cc/en/Tutorial/PathPlanning
5. Grid-Based Map - https://www.arduino.cc/en/Tutorial/GridMap

## 8. Catatan Perkembangan

### 8.1 Iterasi Fitur Opsional
- v1.0: Fuzzy PID dasar, Heading Correction
- v1.1: Path Planner manual, Grid-based map
- v1.2: A* algorithm, Cell types
- v1.3: Integration dengan modul lain
- v1.4: Conditional compilation, config.h

### 8.2 Validasi Fitur Opsional
- Uji Fuzzy PID dengan error & delta_error yang berbeda
- Uji Heading Correction dengan gyro drift
- Uji Path Planner dengan grid yang berbeda
- Uji integration dengan modul lain

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/api/fuzzy_pid.md` untuk dokumentasi API
- Lihat `docs/api/heading_tracker.md` untuk dokumentasi API
- Lihat `docs/api/path_planner.md` untuk dokumentasi API
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya