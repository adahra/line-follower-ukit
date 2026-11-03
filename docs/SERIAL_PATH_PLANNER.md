# Serial Command — Path Planner

Dokumen ini adalah referensi praktis untuk mengoperasikan **Path Planner (A\*)** lewat perintah serial (USB Serial Monitor). Untuk penjelasan arsitektur A\*, grid map, obstacle avoidance, dan integrasi sensor, lihat [`PATH_PLANNER.md`](./PATH_PLANNER.md).

---

## 1. Prasyarat

Path Planner bersifat opsional dan tidak dikompilasi secara default karena memakan RAM ATMega2560 yang terbatas.

| Syarat | Lokasi | Default | Catatan |
|---|---|---|---|
| `ENABLE_PATH_PLANNER = true` | `config.h` | `false` | Master switch compile-time. Jika `false`, semua perintah `path` hanya mencetak pesan nonaktif. |
| `ENABLE_SERIAL_MODE = true` | `config.h` | `true` | Mengaktifkan `ui.cekSerial()` di `loop()`. |
| Serial Monitor 115200 baud | — | — | Baud rate default uKit Explore 2. |

Aktifkan di `config.h`:

```cpp
#define ENABLE_PATH_PLANNER true
```

Lalu compile & upload ulang. Tanpa ini, semua perintah `path ...` menjawab:

```
Path planner dinonaktifkan (ENABLE_PATH_PLANNER=false).
```

---

## 2. Perintah

Semua perintah path dimulai dengan kata kunci `path` diikuti subperintah. Kata kunci `path` di-route oleh `Antarmuka::parseCommand()` ke `PathPlanner::handleCommand()`. Input otomatis di-lowercase.

### Ringkasan

| Perintah | Argumen | Fungsi |
|---|---|---|
| `path map` | — | Tampilkan grid map ASCII di serial |
| `path find` | — | Cari path A\* dari posisi robot ke goal yang sudah di-set |
| `path goal <x> <y>` | `x`, `y`: integer 0–7 | Set tujuan, cari path otomatis, cetak hasilnya |
| `path next` | — | Cetak koordinat waypoint berikutnya di path aktif |
| `path obstacle <x> <y>` | `x`, `y`: integer 0–7 | Tandai cell sebagai obstacle (radius 1 cell) |
| `path reset` | — | Reset seluruh grid map & path planner |

Jika subperintah tidak dikenal, robot mencetak daftar singkat:

```
Path commands: map, find, goal <x> <y>, reset, next, obstacle <x> <y>
```

---

## 3. Detail Tiap Perintah

### `path map`

Mencetak grid 8×8 di serial dalam format ASCII. Posisi robot (`@`), goal (`G`), dan tipe tiap cell ditampilkan.

Contoh output:

```
=== PATH PLANNER MAP ===
Robot: (4,4)
Goal: (1,1)
Legend: .=UNK  =EMP  #=LINE  O=OBS  S=STA  G=GOL  @=ROB
  Y
 0 ........
 1 .G......
 2 ........
 3 ........
 4 ....@...
 5 ........
 6 ........
 7 ........
  0123456789... X
========================
```

Legenda:

| Karakter | Arti | `CellType` |
|---|---|---|
| `.` | Unknown (belum dijelajah) | `CELL_UNKNOWN` |
| ` ` (spasi) | Kosong / jalan bebas | `CELL_EMPTY` |
| `#` | Garis terdeteksi sensor IR | `CELL_LINE` |
| `O` | Obstacle (ultrasonic) | `CELL_OBSTACLE` |
| `S` | Start / posisi awal `begin()` | `CELL_START` |
| `*` | Sudah dilewati robot | `CELL_VISITED` |
| `@` | Posisi robot sekarang | — |
| `G` | Tujuan (goal) | — |

---

### `path goal <x> <y>`

Set tujuan ke koordinat grid `(x, y)`, lalu langsung menjalankan A\* dari posisi robot saat ini. Hasilnya (daftar waypoint + total cost) langsung dicetak.

```
path goal 1 1
```

Output:

```
Goal set to (1,1)
Path found: 5 waypoints, cost: 4.2
  [0] (4,4)
  [1] (3,3)
  [2] (2,2)
  [3] (1,1)
```

Jika goal berada di cell obstacle atau di luar grid, path tidak ditemukan:

```
Goal set to (2,3)
Path: NOT FOUND
```

Grid valid: `x` ∈ [0, `GRID_WIDTH`-1], `y` ∈ [0, `GRID_HEIGHT`-1]. Default `GRID_WIDTH = GRID_HEIGHT = 8` (konfigurasi di `config.h`).

---

### `path find`

Menjalankan ulang pencarian A\* dari posisi robot sekarang ke goal yang sudah di-set sebelumnya (lewat `path goal`). Berguna untuk memperbarui path setelah peta berubah (obstacle baru ditambah, dsb).

```
path find
```

Output:

```
Path found: 4 waypoints, cost: 3.0
  [0] (4,4)
  ...
```

Jika goal belum di-set:

```
No goal set. Use: path goal <x> <y>
```

---

### `path next`

Mencetak koordinat waypoint berikutnya di path aktif. Dipakai untuk tracking manual: setiap kali robot mencapai satu waypoint, kirim `path next` untuk lihat target berikutnya.

```
path next
```

Output saat masih ada waypoint:

```
Next waypoint: (3,3)
```

Output saat path sudah habis atau belum ada path:

```
No active path or goal reached
```

---

### `path obstacle <x> <y>`

Menandai cell `(x, y)` beserta cell-cell tetangganya (radius 1) sebagai `CELL_OBSTACLE`. Cell obstacle dilewati oleh A\* (cost sangat tinggi untuk cel di sekitarnya, terblokir jika tepat pada cell obstacle).

```
path obstacle 3 3
```

Output:

```
Obstacle marked at (3,3)
```

Setelah menambah obstacle, jalankan `path find` untuk memperbarui path. Obstacle juga dapat terdeteksi otomatis dari sensor ultrasonic selama robot berjalan (bila `ENABLE_PATH_PLANNER = true` dan `loop()` memanggil `pathPlanner.updateMap()`).

---

### `path reset`

Mereset seluruh grid ke `CELL_UNKNOWN`, menghapus goal, path aktif, dan posisi robot ke tengah grid.

```
path reset
```

Output:

```
Path planner reset
```

Catatan: `path reset` **berbeda** dengan `mapclear`. Perintah `mapclear` mereset **Mapper** (waypoint learning/racing), bukan PathPlanner. Keduanya adalah modul terpisah.

---

## 4. Konfigurasi Grid

Parameter grid ada di `config.h`:

```cpp
#define ENABLE_PATH_PLANNER false   // master switch
#define GRID_CELL_SIZE_CM   20      // ukuran cell (cm) — dipakai config, path_planner.h mendefinisikan ulang CELL_SIZE_CM = 40
#define PATH_GRID_WIDTH     8       // lebar grid
#define PATH_GRID_HEIGHT    8       // tinggi grid
```

Di `path_planner.h`, nilai runtime yang dipakai:

```cpp
#define GRID_WIDTH   8
#define GRID_HEIGHT  8
#define CELL_SIZE_CM 40   // 40cm per cell → area 3.2m × 3.2m
#define MAX_WAYPOINTS 32
```

Grid 8×8 dipilih untuk menghemat RAM (ATMega2560 hanya 8 KB). Satu `PathPlanner` instance memakan sekitar 64 byte untuk grid + ~500 byte untuk node pool A\*.

---

## 5. Routing Perintah: `path` di `antarmuka.cpp`

Perintah `path` diproses di `antarmuka.cpp` dan diteruskan ke `PathPlanner::handleCommand()`:

```cpp
// antarmuka.cpp
} else if (strcmp(key, "path") == 0) {
#if ENABLE_PATH_PLANNER
    pathPlanner.handleCommand(valStr);
#else
    Serial.println(F("Path planner dinonaktifkan (ENABLE_PATH_PLANNER=false)."));
#endif
}
```

`valStr` adalah sisa string setelah `path ` — jadi `path goal 1 2` meneruskan `"goal 1 2"` ke `handleCommand`, yang kemudian memparse `key = "goal"` dan `valStr = "1 2"`.

Perintah `help` / `status` di `antarmuka.cpp` mencetak baris bantuan path planner hanya jika `ENABLE_PATH_PLANNER`:

```
path map        : tampilkan grid map
path find       : cari path ke goal
path goal <x> <y> : set goal position
path next       : waypoint berikutnya
path obstacle <x> <y> : tambah obstacle
path reset      : reset path planner
```

---

## 6. Inisialisasi di `setup()`

Path planner di-init saat boot di `LineFollowerFin.ino`:

```cpp
#if ENABLE_PATH_PLANNER
  pathPlanner.begin(PATH_GRID_WIDTH / 2, PATH_GRID_HEIGHT / 2);
  Serial.println(F("Path Planner: Ready"));
#endif
```

`begin(4, 4)` menempatkan robot di tengah grid 8×8 dan menandai cell sekitar sebagai `CELL_EMPTY`, cell robot sendiri sebagai `CELL_START`.

---

## 7. Update Otomatis via `loop()`

Selama robot berjalan, `loop()` memanggil `pathPlanner.updateMap()` tiap iterasi:

```cpp
#if ENABLE_PATH_PLANNER
  pathPlanner.updateMap(sensor, pathPlanner.getRobotGridX(), pathPlanner.getRobotGridY(), 0);
#endif
```

`updateMap()` melakukan:
1. Tandai cell posisi robot sebagai `CELL_VISITED`.
2. Jika kedua sensor IR mendeteksi garis (`>= AMBANG`), tandai cell depan sebagai `CELL_LINE`.
3. Jika sensor ultrasonic mendeteksi objek ≤ `JARAK_BERHENTI_CM` (5 cm), tandai cell depan sebagai `CELL_OBSTACLE` — throttled per `CEK_HALANGAN_MS` (100 ms).

Heading yang dikirim saat ini selalu `0` (Utara); orientasi robot belum diintegrasikan dari IMU.

---

## 8. Contoh Sesi

Hubungkan USB, buka Serial Monitor 115200:

```
> path map
(grid 8x8 muncul, robot di tengah @ posisi 4,4)

> path obstacle 2 2
Obstacle marked at (2,2)

> path goal 0 0
Goal set to (0,0)
Path found: 6 waypoints, cost: 5.6
  [0] (4,4)
  [1] (3,3)
  [2] (3,2)
  [3] (2,1)
  [4] (1,0)
  [5] (0,0)

> path map
(path terlihat di map sebagai waypoints, obstacle O di sekitar 2,2)

> path next
Next waypoint: (3,3)

> path next
Next waypoint: (3,2)

> path obstacle 3 2
Obstacle marked at (3,2)

> path find
Path found: 7 waypoints, cost: 6.8
  [0] (4,4)
  ...
(path diperbarui menghindari obstacle baru)

> path reset
Path planner reset

> path map
(semua cell kembali UNKNOWN, robot di tengah)
```

---

## 9. TestPathPlanner

File `LineFollowerFin/tests/TestPathPlanner/TestPathPlanner.ino` adalah sketch mandiri untuk menguji path planner tanpa menjalankan robot penuh. Upload sketch itu lalu buka Serial Monitor 115200. Perintah di test sketch sedikit berbeda (tidak ada prefiks `path`):

| Command test sketch | Padanan di robot utama |
|---|---|
| `map` | `path map` |
| `goal x y` | `path goal x y` |
| `find` | `path find` |
| `next` | `path next` |
| `obs x y` | `path obstacle x y` |
| `reset` | `path reset` |
| `line x y` | *(tidak ada di robot utama)* |
| `auto` | *(simulasi navigasi, tidak ada di robot utama)* |
| `sensor` | *(baca IR langsung, tidak ada di robot utama)* |

---

## 10. Troubleshooting

| Gejala | Penyebab | Solusi |
|---|---|---|
| `Path planner dinonaktifkan (ENABLE_PATH_PLANNER=false).` | Tidak di-compile | Set `ENABLE_PATH_PLANNER true` di `config.h`, recompile & upload |
| `No goal set. Use: path goal <x> <y>` | `path find` dipanggil sebelum `path goal` | Jalankan `path goal x y` lebih dulu |
| `Path: NOT FOUND` | Goal di cell obstacle, atau semua rute terblokir | Cek `path map` — apakah banyak `O`? Kurangi obstacle atau pilih goal lain |
| `No active path or goal reached` | Path habis atau belum ada path | Jalankan `path goal x y` lagi |
| Koordinat di luar grid | `x` atau `y` ≥ 8 (default grid 8×8) | Gunakan koordinat 0–7 |
| Grid selalu penuh `CELL_UNKNOWN` | `updateMap()` tidak pernah dipanggil | Pastikan robot berjalan (`status.jalan = true`) agar `loop()` memanggil `updateMap()` |
| Obstacle tidak terdeteksi otomatis | Sensor ultrasonic tidak terpasang / `ULTRASONIC_ID` salah | Cek `ULTRASONIC_ID` di `config.h` dan wiring sensor |

---

## 11. Referensi Kode

| File | Baris | Isi |
|---|---|---|
| `antarmuka.cpp` | 399–403 | Routing perintah `path` ke `handleCommand` |
| `antarmuka.cpp` | 433–439 | Baris bantuan `path` di `help`/`status` |
| `path_planner.cpp` | 446–497 | `handleCommand()` — parser subperintah |
| `path_planner.cpp` | 400–432 | `printMap()` — output ASCII grid |
| `path_planner.cpp` | 434–444 | `printPath()` — cetak daftar waypoint |
| `path_planner.cpp` | 111–119 | `setGoal()` — set goal + auto findPath |
| `path_planner.cpp` | 121–220 | `findPath()` — algoritma A\* |
| `path_planner.cpp` | 222–249 | `getNextWaypoint()` — ambil waypoint berikutnya |
| `path_planner.cpp` | 252–260 | `markObstacle()` — tandai obstacle + radius |
| `path_planner.cpp` | 37–55 | `reset()` — reset semua state |
| `path_planner.cpp` | 22–34 | `begin()` — inisialisasi posisi awal robot |
| `path_planner.h` | 13–15 | `GRID_WIDTH`, `GRID_HEIGHT`, `CELL_SIZE_CM` |
| `config.h` | ~163–168 | `ENABLE_PATH_PLANNER`, `PATH_GRID_WIDTH/HEIGHT` |
| `LineFollowerFin.ino` | ~64–67 | `pathPlanner.begin()` di `setup()` |
| `LineFollowerFin.ino` | ~112–114 | `pathPlanner.updateMap()` di `loop()` |
