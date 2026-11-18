# Keputusan Desain - Kinerja & Optimasi

Dokumentasi keputusan desain dan alasan untuk optimasi kinerja di LineFollowerFin.

## 1. Ikhtisar

Dokumen ini mencakup keputusan desain untuk optimasi kinerja di LineFollowerFin, yang bertanggung jawab untuk memastikan robot berfungsi dengan efisien dan efektif.

**File:** `docs/design-decisions/performance.md`

## 2. Tujuan Optimasi Kinerja

### 2.1 Penggunaan RAM
**Alasan:** Robot line-following memiliki keterbatasan RAM. Optimasi penggunaan RAM memfasilitasi lebih banyak fitur dan debugging.

**Implementasi:**
- Array statis untuk waypoint (30 x 8 bytes)
- Buffer statis untuk telemetry (64 bytes)
- State statis untuk setiap modul

### 2.2 Konsumsi Waktu
**Alasan:** Robot line-following harus beroperasi dengan timing yang konsisten. Optimasi konsumsi waktu memastikan robot berfungsi dengan timing yang tepat.

**Implementasi:**
- Interval PID tetap (10ms)
- Throttle semua cek safety
- Filter data untuk mengurangi beban komputasi

### 2.3 EEPROM Write Cycle
**Alasan:** EEPROM write cycle terbatas. Optimasi mengurangi write cycle untuk memperpanjang umur EEPROM.

**Implementasi:**
- eepromPutHemat: hanya tulis jika berubah
- Batasi write frequency
- Gunakan checksum untuk validasi

## 3. Desain Decision Key

### 3.1 Static Allocation
**Alasan:** Static allocation memfasilitasi penggunaan RAM yang predictable dan mencegah heap fragmentation. Ini juga memungkinkan penggunaan stack yang lebih efisien.

**Implementasi:**
```cpp
// Static allocation
Waypoint waypoints[MAX_WAYPOINTS]; // 30 x 8 bytes
char telemetryBuffer[TELEMETRY_MAX_LENGTH]; // 64 bytes
// ... static allocation lainnya
```

### 3.2 Throttle Safety
**Alasan:** Throttle safety mencegah overload CPU dengan membatasi frekuensi pemeriksaan dangerous condition. Ini juga memfasilitasi penggunaan CPU yang lebih efisien.

**Implementasi:**
```cpp
// Throttle pemeriksaan
static unsigned long lastCheck = 0;
if (millis() - lastCheck < CEK_BATERAI_MS) {
  return false;
}
lastCheck = millis();
```

### 3.3 Wear Leveling
**Alasan:** Wear leveling meminimalkan write cycle EEPROM untuk memperpanjang umur. Ini juga memfasilitasi penggunaan EEPROM yang lebih efisien.

**Implementasi:**
```cpp
void eepromPutHemat(int address, byte value) {
  byte current = EEPROM.read(address);
  if (current != value) {
    EEPROM.update(address, value);
  }
}
```

### 3.4 Fixed Interval
**Alasan:** Fixed interval memfasilitasi dt yang konstan untuk PID controller. Ini juga memfasilitasi timing yang lebih predictable.

**Implementasi:**
- PID_INTERVAL_MS = 10ms (pastikan dt konstan)
- Gunakan millis() untuk interval yang akurat

## 4. Desain Trade-offs

### 4.1 Static vs Dynamic Allocation
**Trade-off:** Static allocation memfasilitasi penggunaan RAM yang predictable tapi kurang fleksibel. Dynamic allocation lebih fleksibel tapi berisiko heap fragmentation. Memilih static allocation untuk predictability.

### 4.2 Throttle vs Non-throttle
**Trade-off:** Throttle mencegah overload CPU tapi menambah latency. Non-throttle lebih responsif tapi berisiko overheating. Memilih throttle untuk reliability.

### 4.3 Wear Leveling vs Complexity
**Trade-off:** Wear leveling memanjangkan umur EEPROM tapi menambah kompleksitas. Tanpa wear leveling lebih sederhana tapi mengurangi umur EEPROM. Memilih wear leveling untuk longevity.

## 5. Validasi Desain

### 5.1 Static Allocation
**Validasi:** Static allocation diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- Gunakan array statis untuk waypoint
- Gunakan buffer statis untuk telemetry
- Gunakan state statis untuk setiap modul

### 5.2 Throttle Safety
**Validasi:** Throttle safety diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- Throttle baterai: 500ms
- Throttle halangan: 100ms
- Throttle IMU: 50ms
- Throttle cahaya: 200ms

### 5.3 Wear Leveling
**Validasi:** Wear leveling diuji dengan robot pada berbagai kondisi.

**Implementasi:**
- eepromPutHemat: hanya tulis jika berubah
- Batasi write frequency
- Gunakan checksum untuk validasi

## 6. Risiko & Mitigasi

### 6.1 Heap Fragmentation
**Risiko:** Dynamic allocation menyebabkan heap fragmentation. **Mitigasi:** Gunakan static allocation, batasi dynamic allocation.

### 6.2 CPU Overload
**Risiko:** Throttle yang terlalu tinggi menambah latency. Throttle yang terlalu rendah menyebabkan overload CPU. **Mitigasi:** Sesuaikan interval throttle berdasarkan kebutuhan.

### 6.3 EEPROM Wear
**Risiko:** EEPROM write cycle terbatas. **Mitigasi:** Wear leveling, batasi write frequency.

## 7. Referensi

1. Arduino Memory Management - https://www.arduino.cc/en/Tutorial/MemoryManagement
2. CPU Optimization - https://www.arduino.cc/en/Tutorial/CPUPowerManagement
3. EEPROM Wear Leveling - https://www.arduino.cc/en/Tutorial/WearLeveling
4. Fixed Interval - https://www.arduino.cc/en/Tutorial/FixedInterval
5. Static Allocation - https://www.arduino.cc/en/Tutorial/StaticAllocation

## 8. Catatan Perkembangan

### 8.1 Iterasi Optimasi Kinerja
- v1.0: Static allocation, throttle safety
- v1.1: Wear leveling, fixed interval
- v1.2: Optimasi RAM, optimasi CPU
- v1.3: Optimasi EEPROM, optimasi timing
- v1.4: Optimasi secara keseluruhan

### 8.2 Validasi Optimasi Kinerja
- Uji dengan robot pada berbagai kondisi
- Uji dengan heap yang penuh
- Uji dengan CPU yang terbatas
- Uji dengan EEPROM yang wear

## 9. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk dokumentasi arsitektur
- Lihat `docs/design-decisions/overall.md` untuk keputusan desain lainnya
- Lihat `docs/design-decisions/safety.md` untuk safety
- Lihat `docs/design-decisions/pid.md` untuk PID