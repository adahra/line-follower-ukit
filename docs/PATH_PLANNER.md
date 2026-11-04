# Path Planner & Algoritma A* — Line Follower UKIT

## 1. Pengertian

Path Planner adalah subsistem navigasi berbasis grid map yang menyimpan informasi lingkungan (garis, halangan, area yang sudah dilewati) dan mencari jalur optimal dari posisi robot ke goal dengan algoritma A*. Pada proyek ini, Path Planner bersifat **opsional** dan diaktifkan melalui `ENABLE_PATH_PLANNER` di `config.h` (default `false` untuk hemat RAM ATMega2560).

Kaitannya dengan line follower: robot mengikuti garis secara PID (reaktif). Path Planner menambahkan lapisan perencanaan rute (planning) di atas kontrol reaktif, sehingga robot dapat menghindari halangan dan memilih rute terpendek menuju target.

**Status saat ini:** Path Planner masih berupa *passive map builder* — peta dibangun dari baca sensor tetapi `getNextWaypoint()` **tidak pernah dipanggil di loop utama**, sehingga robot belum benar-benar mengikuti jalur yang dihasilkan A*. Detail ada di bagian Integrasi (bagian 13).

File sumber:
- `LineFollowerFin/path_planner.h` — deklarasi class dan struct (155 baris)
- `LineFollowerFin/path_planner.cpp` — implementasi lengkap (502 baris)
- Integrasi dipanggil di `LineFollowerFin/LineFollowerFin.ino` baris 119-121

---

## 2. Arsitektur Grid

Path Planner menggunakan grid 2D statis untuk merepresentasikan lingkungan:

```cpp
// path_planner.h:12-15
#define GRID_WIDTH  8
#define GRID_HEIGHT 8
#define CELL_SIZE_CM 40  // 40cm per cell (area 3.2m x 3.2m)
#define MAX_WAYPOINTS 32
```

- **Grid:** `GRID_WIDTH x GRID_HEIGHT` = 8x8 cell.
- **Ukuran cell:** `CELL_SIZE_CM = 40 cm`, sehingga area total peta adalah 3.2m x 3.2m.
- **Tipe cell** (`CellType`, `path_planner.h:18-25`):
  - `CELL_UNKNOWN = 0` — belum diketahui
  - `CELL_EMPTY = 1` — kosong, dapat dilalui
  - `CELL_LINE = 2` — garis terdeteksi
  - `CELL_OBSTACLE = 3` — halangan (ultrasonic)
  - `CELL_START = 4` — start/finish line
  - `CELL_VISITED = 5` — sudah dilewati robot

Grid disimpan dalam array statis `CellType grid[GRID_WIDTH][GRID_HEIGHT]` (`path_planner.h:110`).

Posisi robot dalam grid dilacak oleh `robotGridX`, `robotGridY` (default tengah grid), dan `robotHeading` dalam derajat (0 = Utara) (`path_planner.h:113-115`).

Konversi koordinat world <-> grid disediakan oleh `worldToGrid()` dan `gridToWorld()` (`path_planner.h:91-92`), yang menggunakan `CELL_SIZE_CM` untuk skala dengan origin di tengah grid.

---

## 3. PathNode dan Struktur Data A*

### 3.1 Struct PathNode

Setiap cell yang terlibat dalam pencarian A* direpresentasikan oleh `PathNode`:

```cpp
// path_planner.h:40-51
struct PathNode {
  int x, y;
  float gCost;  // Cost dari start
  float hCost;  // Heuristik ke goal
  float fCost() const { return gCost + hCost; }
  PathNode *parent;
  bool closed;
  bool open;

  PathNode() : x(0), y(0), gCost(0), hCost(0), parent(nullptr), closed(false), open(false) {}
  PathNode(int _x, int _y) : x(_x), y(_y), gCost(0), hCost(0), parent(nullptr), closed(false), open(false) {}
};
```

| Field | Keterangan |
|-------|-----------|
| `x, y` | Koordinat cell dalam grid |
| `gCost` | Biaya aktual dari start node ke node ini |
| `hCost` | Estimasi biaya (heuristik) dari node ini ke goal |
| `fCost()` | `gCost + hCost`, total estimasi biaya |
| `parent` | Pointer ke node induk pada jalur terbaik yang diketahui |
| `closed` | Sudah dievaluasi/dikeluarkan dari open set |
| `open` | Masih dalam open set (kandidat untuk dievaluasi) |

### 3.2 Struct PathResult

Hasil pencarian disimpan dalam `PathResult`:

```cpp
// path_planner.h:54-59
struct PathResult {
  int waypoints[MAX_WAYPOINTS][2];  // x,y pairs
  int count = 0;
  bool found = false;
  float totalCost = 0;
};
```

`waypoints` berisi daftar koordinat grid dari start ke goal (sudah dibalik agar urutannya start → goal). `found` menunjukkan apakah jalur ditemukan. `totalCost` adalah biaya total jalur (`gCost` dari goal node).

### 3.3 Node Pool (Tanpa malloc)

Untuk menghindari alokasi memori dinamis (malloc) yang terbatas dan berisiko pada ATMega2560, Path Planner menggunakan **node pool statis**:

```cpp
// path_planner.h:137-140
static const int MAX_NODES = GRID_WIDTH * GRID_HEIGHT;
PathNode nodePool[MAX_NODES];
PathNode* nodes[GRID_WIDTH][GRID_HEIGHT];
```

- `nodePool[64]` — array statis yang menyimpan 64 node (satu untuk setiap cell 8x8).
- `nodes[8][8]` — array pointer yang memetakan setiap koordinat grid ke entri nodePool yang sesuai.

Inisialisasi dilakukan di konstruktor (`path_planner.cpp:10-20`):

```cpp
PathPlanner::PathPlanner() {
  for (int x = 0; x < GRID_WIDTH; x++) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
      nodes[x][y] = &nodePool[x * GRID_HEIGHT + y];
      nodes[x][y]->x = x;
      nodes[x][y]->y = y;
    }
  }
  reset();
}
```

Tiap cell grid selalu terhubung ke node pool-nya secara 1:1. Tidak ada `new`, `malloc`, atau `free` di seluruh implementasi Path Planner.

---

## 4. Algoritma A* — Langkah Demi Langkah

Algoritma A* diimplementasikan dalam `PathPlanner::findPath()` (`path_planner.cpp:121-220`). Berikut adalah eksekusi step-by-step:

### Langkah 1: Validasi Input

```cpp
if (!isValid(startX, startY) || !isValid(goalX_, goalY_)) {
  return result;  // Koordinat di luar grid
}

if (isBlocked(goalX_, goalY_)) {
  return result;  // Goal adalah halangan, tidak bisa dicapai
}
```

`isValid()` memeriksa batas grid (`path_planner.cpp:292-294`). `isBlocked()` hanya menganggap `CELL_OBSTACLE` sebagai blocked (`path_planner.cpp:246-250`).

### Langkah 2: Reset Node dan Inisialisasi Open Set

```cpp
clearNodes();
openSetSize = 0;

PathNode *startNode = nodes[startX][startY];
startNode->gCost = 0;
startNode->hCost = heuristic(startX, startY, goalX_, goalY_);
startNode->parent = nullptr;
startNode->open = true;
openSetPush(startNode);
```

`clearNodes()` mereset semua field node pool ke default (`path_planner.cpp:308-316`). Start node dimasukkan ke open set melalui binary min-heap.

### Langkah 3: Loop Utama A*

Sambil open set tidak kosong, ambil node dengan `fCost` terendah:

```cpp
while (openSetSize > 0) {
  PathNode *current = openSetPop();
  current->open = false;
  current->closed = true;
```

`openSetPop()` mengeluarkan root heap (node dengan fCost minimum), kemudian melakukan heapify-down untuk menjaga properti min-heap.

### Langkah 4: Pemeriksaan Goal

Jika node yang di-pop adalah goal node, rekonstruksi jalur:

```cpp
if (current == goalNode) {
  PathNode *pathNode = current;
  int pathLength = 0;
  while (pathNode && pathLength < MAX_WAYPOINTS) {
    result.waypoints[pathLength][0] = pathNode->x;
    result.waypoints[pathLength][1] = pathNode->y;
    pathLength++;
    pathNode = pathNode->parent;
  }

  // Reverse path (start to goal)
  for (int i = 0; i < pathLength / 2; i++) {
    int tmpX = result.waypoints[i][0];
    int tmpY = result.waypoints[i][1];
    result.waypoints[i][0] = result.waypoints[pathLength - 1 - i][0];
    result.waypoints[i][1] = result.waypoints[pathLength - 1 - i][1];
    result.waypoints[pathLength - 1 - i][0] = tmpX;
    result.waypoints[pathLength - 1 - i][1] = tmpY;
  }

  result.count = pathLength;
  result.found = true;
  result.totalCost = current->gCost;
  return result;
}
```

Jalur direkonstruksi dengan menuruni rantai `parent` dari goal ke start, lalu dibalik agar urutannya start → goal. `totalCost` diambil dari `gCost` goal node.

### Langkah 5: Ekspansi 8 Tetangga

Untuk setiap node current, periksa 8 arah tetangga (N, NE, E, SE, S, SW, W, NW):

```cpp
const int dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
const int dy[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
const float moveCosts[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};

for (int i = 0; i < 8; i++) {
  int nx = current->x + dx[i];
  int ny = current->y + dy[i];

  if (!isValid(nx, ny)) continue;
  if (isBlocked(nx, ny)) continue;
  if (nodes[nx][ny]->closed) continue;

  float newGCost = current->gCost + moveCosts[i];

  // Penalty untuk dekat obstacle (3x3 neighborhood)
  for (int ox = -1; ox <= 1; ox++) {
    for (int oy = -1; oy <= 1; oy++) {
      if (isValid(nx + ox, ny + oy) && grid[nx + ox][ny + oy] == CELL_OBSTACLE) {
        newGCost += 2.0; // Penalty
      }
    }
  }

  PathNode *neighbor = nodes[nx][ny];

  if (!neighbor->open || newGCost < neighbor->gCost) {
    neighbor->gCost = newGCost;
    neighbor->hCost = heuristic(nx, ny, goalX_, goalY_);
    neighbor->parent = current;

    if (!neighbor->open) {
      neighbor->open = true;
      openSetPush(neighbor);
    }
  }
}
```

### Langkah 6: Terminasi

Loop berakhir ketika:
1. Goal node di-pop dari open set → jalur ditemukan (`found = true`), atau
2. Open set kosong → tidak ada jalur yang tersedia (`found = false`).

---

## 5. Open Set — Binary Min-Heap

Open set diimplementasikan sebagai binary min-heap sederhana berbasis array, tanpa STL:

```cpp
// path_planner.h:142-144
PathNode* openSet[MAX_NODES];
int openSetSize = 0;
```

### 5.1 openSetPush — Bubble Up

```cpp
// path_planner.cpp:318-333
void PathPlanner::openSetPush(PathNode *node) {
  if (openSetSize < MAX_NODES) {
    openSet[openSetSize++] = node;
    // Simple bubble up untuk maintain min-heap property
    int i = openSetSize - 1;
    while (i > 0) {
      int parent = (i - 1) / 2;
      if (openSet[i]->fCost() < openSet[parent]->fCost()) {
        PathNode *tmp = openSet[i];
        openSet[i] = openSet[parent];
        openSet[parent] = tmp;
        i = parent;
      } else break;
    }
  }
}
```

Node baru ditambahkan di akhir array, lalu dibubbles-up sambil membandingkan `fCost` dengan parent hingga properti min-heap terpenuhi.

### 5.2 openSetPop — Heapify Down

```cpp
// path_planner.cpp:335-362
PathNode* PathPlanner::openSetPop() {
  if (openSetSize == 0) return nullptr;

  PathNode *min = openSet[0];
  openSet[0] = openSet[--openSetSize];

  // Heapify down
  int i = 0;
  while (true) {
    int smallest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (left < openSetSize && openSet[left]->fCost() < openSet[smallest]->fCost())
      smallest = left;
    if (right < openSetSize && openSet[right]->fCost() < openSet[smallest]->fCost())
      smallest = right;

    if (smallest != i) {
      PathNode *tmp = openSet[i];
      openSet[i] = openSet[smallest];
      openSet[smallest] = tmp;
      i = smallest;
    } else break;
  }

  return min;
}
```

Root (node dengan fCost minimum) diambil, elemen terakhir dipindahkan ke root, lalu heapify-down dilakukan untuk memulihkan properti heap.

### 5.3 openSetRemove dan openSetContains

`openSetRemove` (`path_planner.cpp:364-391`) mencari node dalam open set, menggantinya dengan elemen terakhir, lalu membangun ulang heap dari atas. `openSetContains` (`path_planner.cpp:393-398`) melakukan linear scan untuk memeriksa apakah koordinat (x,y) sudah ada dalam open set.

Catatan: dalam implementasi saat ini, `openSetRemove` **tidak pernah dipanggil** dari `findPath` — node yang sudah dibuka tetap dalam open set hingga di-pop, dan pengecekan `!neighbor->open || newGCost < neighbor->gCost` menangani pembaruan biaya.

---

## 6. Heuristik — Octile Distance

Heuristik digunakan untuk estimasi biaya dari node ke goal. Karena grid mendukung gerak diagonal, digunakan **octile distance** yang lebih akurat dari Manhattan untuk 8-connected grid:

```cpp
// path_planner.cpp:285-290
float PathPlanner::heuristic(int x1, int y1, int x2, int y2) const {
  // Octile distance heuristic (better for 8-connected grid)
  int dx = abs(x1 - x2);
  int dy = abs(x1 - x2);
  return 1.414 * min(dx, dy) + abs(dx - dy);
}
```

Rumus: `1.414 * min(dx, dy) + |dx - dy|`

- `1.414` ≈ √2 adalah biaya gerak diagonal.
- Heuristik ini **admissible** (tidak pernah overestimate biaya aktual) untuk grid 8-connected, sehingga A* dijamin menemukan jalur optimal jika ada.

Catatan: ada potensi typo di baris `int dy = abs(x1 - x2);` — seharusnya `abs(y1 - y2)`. Namun karena simetris (dx dan dy sama nilainya setelah abs), hasilnya tetap sama untuk perhitungan octile distance ini.

---

## 7. Move Cost dan 8-Arah Gerak

Biaya gerak didefinisikan secara eksplisit dalam loop ekspansi (`path_planner.cpp:181-183`):

```cpp
const float moveCosts[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};
```

Urutan arah sesuai enum `Direction` (`path_planner.h:28-37`):

| Index | Arah | dx | dy | Biaya |
|-------|------|----|----|-------|
| 0 | DIR_N (North/Atas) | 0 | -1 | 1.0 (orthogonal) |
| 1 | DIR_NE | 1 | -1 | 1.414 (diagonal) |
| 2 | DIR_E (East/Kanan) | 1 | 0 | 1.0 (orthogonal) |
| 3 | DIR_SE | 1 | 1 | 1.414 (diagonal) |
| 4 | DIR_S (South/Bawah) | 0 | 1 | 1.0 (orthogonal) |
| 5 | DIR_SW | -1 | 1 | 1.414 (diagonal) |
| 6 | DIR_W (West/Kiri) | -1 | 0 | 1.0 (orthogonal) |
| 7 | DIR_NW | -1 | -1 | 1.414 (diagonal) |

Fungsi `moveCost()` terpisah juga ada (`path_planner.cpp:296-301`) tetapi **tidak digunakan** dalam `findPath` — biaya secara langsung diambil dari array `moveCosts[8]`.

---

## 8. Penalty Halangan 3x3

Untuk mendorong jalur menjauh dari halangan, setiap tetangga yang memiliki obstacle dalam lingkungan 3x3 dikenai penalti tambahan:

```cpp
// path_planner.cpp:196-202
for (int ox = -1; ox <= 1; ox++) {
  for (int oy = -1; oy <= 1; oy++) {
    if (isValid(nx + ox, ny + oy) && grid[nx + ox][ny + oy] == CELL_OBSTACLE) {
      newGCost += 2.0; // Penalty
    }
  }
}
```

Setiap obstacle dalam radius 3x3 sekitar cell kandidat menambahkan +2.0 ke `gCost`. Ini membuat A* memilih jalur yang lebih jauh dari obstacle bila ada alternatif yang sebanding.

---

## 9. Rekonstruksi Path

Setelah goal tercapai, jalur direkonstruksi dengan menuruni pointer `parent` dari goal node hingga start node (yang `parent`-nya `nullptr`). Koordinat disimpan ke array `waypoints`, lalu dibalik agar urutan dari start ke goal. Batas maksimal `MAX_WAYPOINTS = 32` waypoint.

---

## 10. Update Map dari Sensor

Fungsi `updateMap()` adalah jembatan antara sensor fisik dan grid peta. Sejak P1, fungsi ini **tidak dipanggil otomatis dari `loop()`**: Path Planner dipakai sebagai alat peta/A* manual lewat perintah serial (`path ...`), bukan pengendali gerak. Memanggilnya tiap loop juga membuat sensor IR dan ultrasonik dibaca dua kali tanpa manfaat pengendalian.

```cpp
// path_planner.cpp:56-109
void PathPlanner::updateMap(SensorGaris &sensor, int robotX, int robotY, float robotHeading) {
  // Update posisi robot
  robotGridX = constrain(robotX, 0, GRID_WIDTH - 1);
  robotGridY = constrain(robotY, 0, GRID_HEIGHT - 1);
  this->robotHeading = robotHeading;

  // Mark current cell as visited/empty
  grid[robotGridX][robotGridY] = CELL_VISITED;

  // Baca sensor IR untuk deteksi garis
  int leftIR = sensor.bacaKiri();
  int rightIR = sensor.bacaKanan();

  // Jika kedua sensor mendeteksi garis, mark cell depan sebagai line
  if (leftIR >= AMBANG_KIRI && rightIR >= AMBANG_KANAN) {
    int frontX = robotGridX;
    int frontY = robotGridY;

    // Hitung cell depan berdasarkan heading
    if (robotHeading >= -45 && robotHeading < 45) frontY--;      // North
    else if (robotHeading >= 45 && robotHeading < 135) frontX++; // East
    else if (robotHeading >= 135 || robotHeading < -135) frontY++; // South
    else frontX--; // West

    if (isValid(frontX, frontY)) {
      grid[frontX][frontY] = CELL_LINE;
    }
  }

  // --- Integrate ultrasonic untuk obstacle detection (throttled) ---
  unsigned long nowMs = millis();
  if (nowMs - lastUltrasonicCheck >= CEK_HALANGAN_MS) {
    lastUltrasonicCheck = nowMs;
    int jarak = readUltrasonicDistance(ULTRASONIC_ID);
    // 0 = bacaan tak valid (di luar jangkauan) -> abaikan.
    if (jarak > 0 && jarak <= JARAK_BERHENTI_CM) {
      // Hitung cell depan berdasarkan heading (sama dengan logika garis di atas)
      int frontX = robotGridX;
      int frontY = robotGridY;
      if (robotHeading >= -45 && robotHeading < 45) frontY--;      // North
      else if (robotHeading >= 45 && robotHeading < 135) frontX++; // East
      else if (robotHeading >= 135 || robotHeading < -135) frontY++; // South
      else frontX--; // West

      if (isValid(frontX, frontY)) {
        // Cell berikutnya langsung terhalang -> tandai sebagai obstacle
        // (radius 0 = hanya cell depan, konsisten dengan jarak 5 cm).
        markObstacle(frontX, frontY, 0);
      }
    }
  }
}
```

Alur `updateMap`:
1. Update posisi robot dalam grid (diconstrain ke batas 0-7).
2. Tandai cell posisi robot saat ini sebagai `CELL_VISITED`.
3. Baca sensor IR kiri dan kanan. Jika keduanya di atas ambang, tandai cell depan (berdasarkan heading) sebagai `CELL_LINE`.
4. Baca ultrasonic dengan throttling `CEK_HALANGAN_MS = 100ms`. Jika jarak > 0 dan ≤ `JARAK_BERHENTI_CM` (5 cm), tandai cell depan sebagai `CELL_OBSTACLE` dengan radius 0 (hanya cell itu sendiri).

Heading dalam derajat dipetakan ke arah grid:
- `-45° ≤ heading < 45°` → Utara (frontY--)
- `45° ≤ heading < 135°` → Timur (frontX++)
- `heading ≥ 135° atau heading < -135°` → Selatan (frontY--)
- Sebaliknya → Barat (frontX--)

---

## 11. isBlocked, markObstacle, markLine

### 11.1 isBlocked

```cpp
// path_planner.cpp:246-250
bool PathPlanner::isBlocked(int x, int y) const {
  if (!isValid(x, y)) return true;
  CellType t = grid[x][y];
  return (t == CELL_OBSTACLE); // Only hard obstacles block
}
```

Hanya cell dengan tipe `CELL_OBSTACLE` yang dianggap blocked. Cell `CELL_LINE`, `CELL_VISITED`, `CELL_UNKNOWN`, dll. tetap dapat dilalui oleh A*. Ini berarti garis terdeteksi **tidak** menghalangi perencanaan jalur — hanya obstacle fisik dari ultrasonic.

### 11.2 markObstacle

```cpp
// path_planner.cpp:252-262
void PathPlanner::markObstacle(int x, int y, int radius) {
  for (int dx = -radius; dx <= radius; dx++) {
    for (int dy = -radius; dy <= radius; dy++) {
      int nx = x + dx;
      int ny = y + dy;
      if (isValid(nx, ny) && (dx*dx + dy*dy <= radius*radius)) {
        grid[nx][ny] = CELL_OBSTACLE;
      }
    }
  }
}
```

Mengisi lingkaran berjajar (circle fill) berdasarkan kondisi `dx² + dy² ≤ r²`. Default radius = 1. Saat ultrasonic mendeteksi obstacle ≤ 5 cm, dipanggil dengan radius 0 (hanya cell depan).

### 11.3 markLine

```cpp
// path_planner.cpp:264-268
void PathPlanner::markLine(int x, int y) {
  if (isValid(x, y) && grid[x][y] != CELL_OBSTACLE) {
    grid[x][y] = CELL_LINE;
  }
}
```

Tandai cell sebagai `CELL_LINE`, kecuali jika cell sudah menjadi obstacle.

---

## 12. Goal dan Waypoint Following

### 12.1 setGoal

```cpp
// path_planner.cpp:111-119
void PathPlanner::setGoal(int goalX_, int goalY_) {
  goalX = constrain(goalX_, 0, GRID_WIDTH - 1);
  goalY = constrain(goalY_, 0, GRID_HEIGHT - 1);
  hasGoal_ = true;
  currentWaypointIndex = 0;

  // Cari path baru
  currentPath = findPath(robotGridX, robotGridY, goalX, goalY);
}
```

Goal di-constrain ke batas grid, flag `hasGoal_` di-set, indeks waypoint direset, dan `findPath` dipanggil dari posisi robot saat ini ke goal. Hasil disimpan di `currentPath`.

### 12.2 getNextWaypoint

```cpp
// path_planner.cpp:222-244
bool PathPlanner::getNextWaypoint(int robotX, int robotY, int &nextX, int &nextY) {
  if (!currentPath.found || currentWaypointIndex >= currentPath.count) {
    return false;
  }

  // Cek apakah sudah dekat waypoint saat ini
  int wpX = currentPath.waypoints[currentWaypointIndex][0];
  int wpY = currentPath.waypoints[currentWaypointIndex][1];

  int dist = abs(robotX - wpX) + abs(robotY - wpY); // Manhattan distance
  if (dist <= 1) {
    currentWaypointIndex++;
    if (currentWaypointIndex >= currentPath.count) {
      return false; // Goal reached
    }
    wpX = currentPath.waypoints[currentWaypointIndex][0];
    wpY = currentPath.waypoints[currentWaypointIndex][1];
  }

  nextX = wpX;
  nextY = wpY;
  return true;
}
```

Jika Manhattan distance dari robot ke waypoint saat ini ≤ 1, indeks waypoint dinaikkan. Mengembalikan waypoint berikutnya sebagai target navigasi. Jika semua waypoint telah tercapai, mengembalikan `false` (goal reached).

**Catatan:** `getNextWaypoint` **tidak pernah dipanggil di `loop()` utama** — lihat bagian Integrasi.

---

## 13. Integrasi dengan loop()

Path Planner diaktifkan bila `ENABLE_PATH_PLANNER` true. Deklarasi objek dan inisialisasi di `LineFollowerFin.ino`:

```cpp
// LineFollowerFin.ino:18-20, 30-32
#if ENABLE_PATH_PLANNER
#include "path_planner.h"  // Optional: enable via config.h + move optional/path_planner/ to sketch folder
#endif
...
#if ENABLE_PATH_PLANNER
PathPlanner pathPlanner;  // Optional: uncomment + include path_planner.h when needed
#endif
```

Setup (`LineFollowerFin.ino:75-79`):

```cpp
// Inisialisasi Path Planner
#if ENABLE_PATH_PLANNER
  pathPlanner.begin(PATH_GRID_WIDTH / 2, PATH_GRID_HEIGHT / 2);
  Serial.println(F("Path Planner: Ready"));
#endif
```

`begin()` memanggil `reset()` lalu menetapkan posisi start di tengah grid dan menandai area sekitarnya sebagai `CELL_EMPTY` dengan center sebagai `CELL_START`.

Pemanggilan otomatis di loop **sudah dihapus** (lihat `LineFollowerFin.ino`, blok Path Planner di `loop()`). Pemanggilan lama yang bermasalah:

```cpp
// DIHAPUS: posisi/heading hardcoded + membaca sensor dua kali
// pathPlanner.updateMap(sensor, pathPlanner.getRobotGridX(), pathPlanner.getRobotGridY(), 0);
```

### Masalah Integrasi yang Diketahui

1. **Posisi robot hardcoded:** `pathPlanner.getRobotGridX()` dan `getRobotGridY()` mengembalikan posisi internal Path Planner (default 4,4 = tengah grid), bukan posisi robot yang diukur dari odometri/IMU. Posisi tidak pernah diperbarui dari world coordinate ke grid secara eksternal.
2. **Heading hardcoded 0:** Heading selalu dikirim sebagai `0` (Utara), tanpa integrasi IMU yaw. Deteksi cell depan berdasarkan heading selalu asumsi robot menghadap Utara.
3. **`getNextWaypoint` tidak dipanggil:** Meskipun `currentPath` tersedia setelah `setGoal`, tidak ada kode di loop yang memanggil `getNextWaypoint` untuk menggerakkan robot menuju waypoint berikutnya.
4. **Tidak ada pemanggilan `setGoal` di loop:** Goal tidak pernah ditetapkan secara otomatis; hanya dapat melalui perintah serial `path goal <x> <y>`.

Kesimpulan: Path Planner saat ini berfungsi sebagai **passive map builder** — peta dibangun dari bacaan sensor tetapi tidak digunakan untuk menggerakkan robot.

---

## 14. worldToGrid dan gridToWorld

```cpp
// path_planner.cpp:270-283
void PathPlanner::worldToGrid(float worldX, float worldY, int &gridX, int &gridY) const {
  // World origin at center of grid
  float centerX = GRID_WIDTH * CELL_SIZE_CM / 2.0;
  float centerY = GRID_HEIGHT * CELL_SIZE_CM / 2.0;
  gridX = constrain(int((worldX + centerX) / CELL_SIZE_CM), 0, GRID_WIDTH - 1);
  gridY = constrain(int((worldY + centerY) / CELL_SIZE_CM), 0, GRID_HEIGHT - 1);
}

void PathPlanner::gridToWorld(int gridX, int gridY, float &worldX, float &worldY) const {
  float centerX = GRID_WIDTH * CELL_SIZE_CM / 2.0;
  float centerY = GRID_HEIGHT * CELL_SIZE_CM / 2.0;
  worldX = gridX * CELL_SIZE_CM - centerX;
  worldY = gridY * CELL_SIZE_CM - centerY;
}
```

Origin world di tengah grid. `worldToGrid` mengkonversi koordinat world (cm) ke indeks grid. `gridToWorld` sebaliknya. Keduanya menggunakan `CELL_SIZE_CM` untuk skala.

---

## 15. Serial Commands

Ditangani oleh `handleCommand(const char*)` (`path_planner.cpp:446-502`). Perintah diawali dengan kata kunci `path`, diurai oleh parser serial di `antarmuka.cpp`:

| Perintah | Deskripsi |
|----------|-----------|
| `path map` | Cetak peta grid ke Serial |
| `path find` | Jalankan ulang pathfinding dari posisi robot ke goal |
| `path goal <x> <y>` | Set goal dan cari jalur |
| `path reset` | Reset peta dan node |
| `path next` | Tampilkan waypoint berikutnya |
| `path obstacle <x> <y>` | Tandai obstacle di koordinat grid |

Contoh implementasi `handleCommand`:

```cpp
// path_planner.cpp:446-501
void PathPlanner::handleCommand(const char *cmd) {
  char cmdCopy[64];
  strncpy(cmdCopy, cmd, 63);
  cmdCopy[63] = '\0';

  char *space = strchr(cmdCopy, ' ');
  char *key = cmdCopy;
  char *valStr = "";
  if (space) {
    *space = '\0';
    valStr = space + 1;
  }

  for (char *p = key; *p; ++p) *p = tolower(*p);

  if (strcmp(key, "map") == 0) {
    printMap();
  } else if (strcmp(key, "find") == 0) {
    if (hasGoal_) {
      currentPath = findPath(robotGridX, robotGridY, goalX, goalY);
      printPath(currentPath);
    } else {
      Serial.println(F("No goal set. Use: path goal <x> <y>"));
    }
  } else if (strcmp(key, "goal") == 0) {
    int gx, gy;
    if (sscanf(valStr, "%d %d", &gx, &gy) == 2) {
      setGoal(gx, gy);
      Serial.print(F("Goal set to (")); Serial.print(goalX); Serial.print(F(",")); Serial.print(goalY); Serial.println(F(")"));
      printPath(currentPath);
    } else {
      Serial.println(F("Usage: path goal <x> <y>"));
    }
  } else if (strcmp(key, "reset") == 0) {
    reset();
    Serial.println(F("Path planner reset"));
  } else if (strcmp(key, "next") == 0) {
    int nx, ny;
    if (getNextWaypoint(robotGridX, robotGridY, nx, ny)) {
      Serial.print(F("Next waypoint: (")); Serial.print(nx); Serial.print(F(",")); Serial.print(ny); Serial.println(F(")"));
    } else {
      Serial.println(F("No active path or goal reached"));
    }
  } else if (strcmp(key, "obstacle") == 0) {
    int ox, oy;
    if (sscanf(valStr, "%d %d", &ox, &oy) == 2) {
      markObstacle(ox, oy, 1);
      Serial.print(F("Obstacle marked at (")); Serial.print(ox); Serial.print(F(",")); Serial.print(oy); Serial.println(F(")"));
    } else {
      Serial.println(F("Usage: path obstacle <x> <y>"));
    }
  } else {
    Serial.println(F("Path commands: map, find, goal <x> <y>, reset, next, obstacle <x> <y>"));
  }
}
```

Fungsi debug tambahan: `printMap()` menampilkan grid dalam format ASCII dengan legenda, dan `printPath()` mencetak daftar waypoint dengan biaya total.

---

## 16. Konfigurasi

Semua opsi Path Planner ada di `config.h` baris 163-167:

```cpp
// --- PATH PLANNER ---
#define ENABLE_PATH_PLANNER false     // default OFF (RAM ATMega2560 terbatas)
#define GRID_CELL_SIZE_CM 20          // ukuran cell grid (cm)
#define PATH_GRID_WIDTH 8             // lebar grid (8x8 untuk hemat RAM)
#define PATH_GRID_HEIGHT 8            // tinggi grid
```

### Inkonsistensi CELL_SIZE_CM vs GRID_CELL_SIZE_CM

Ada **inkonsistensi** antara dua makro yang seharusnya merujuk pada hal yang sama:

| File | Makro | Nilai | Baris |
|------|-------|-------|-------|
| `path_planner.h` | `CELL_SIZE_CM` | 40 | 14 |
| `config.h` | `GRID_CELL_SIZE_CM` | 20 | 165 |

- `path_planner.h` menggunakan `CELL_SIZE_CM = 40` untuk konversi world<->grid (`worldToGrid`, `gridToWorld`). Dengan 8x8 cell × 40 cm = area 3.2m × 3.2m.
- `config.h` mendefinisikan `GRID_CELL_SIZE_CM = 20` tetapi makro ini **tidak digunakan di mana pun** dalam kode Path Planner (karena `path_planner.h` memiliki definisi mandiri `CELL_SIZE_CM`).

**Rekomendasi:** Diselaraskan ke satu nilai. Jika grid 8x8 diinginkan untuk area 1.6m × 1.6m, ubah `CELL_SIZE_CM` di header menjadi 20. Jika area 3.2m × 3.2m diinginkan, ubah `config.h` menjadi 40 dan gunakan konsisten di kedua tempat.

Konfigurasi lain terkait:
- `ENABLE_PATH_PLANNER` — harus `true` untuk mengaktifkan codePath planner (default `false`).
- `PATH_GRID_WIDTH` / `PATH_GRID_HEIGHT` — hanya digunakan di `begin()` untuk posisi start default, grid tetap hardcoded 8x8 di header.
- `CEK_HALANGAN_MS` (100ms) dan `JARAK_BERHENTI_CM` (5 cm) — mengontrol throttling dan ambang ultrasonic dalam `updateMap`.
- `AMBANG_KIRI` / `AMBANG_KANAN` — ambang sensor IR untuk deteksi garis dalam `updateMap`.

---

## 17. Memory Layout dan Constraints

ATMega2560 memiliki RAM sekitar 8KB. Path Planner dirancang hemat memori:

### 17.1 Alokasi Memori Statis

| Komponen | Ukuran | Detail |
|----------|--------|--------|
| `grid[8][8]` (CellType enum = byte) | 64 byte | Tipe cell per grid |
| `nodePool[64]` (PathNode) | ~64 × 48 byte ≈ 3 KB | Pool node A* |
| `nodes[8][8]` (pointer) | 64 × 2 byte = 128 byte | Pointer ke nodePool |
| `openSet[64]` (pointer) | 64 × 2 byte = 128 byte | Binary min-heap |
| `currentPath.waypoints[32][2]` (int) | 32 × 2 × 2 byte = 128 byte | Daftar waypoint |
| `PathResult currentPath` | ~130 byte | Struct hasil path |
| **Total estimasi** | **~3.5 KB** | Hampir separuh RAM tersedia |

PathNode terdiri dari: 2 int (x,y) + 3 float (g,h,fCost inline) + 1 pointer (parent) + 2 bool ≈ 24-32 byte per node. Dengan 64 node ≈ 1.5-2 KB untuk nodePool saja.

### 17.2 Alasan Tidak Ada malloc

- Arduino AVR tidak mendukung malloc secara efisien; fragmentasi heap dapat menyebabkan crash.
- Node pool statis memastikan alokasi determinin dan tidak ada overhead runtime.
- Ukuran terbatas pada 64 node = maksimal grid 8x8. Grid lebih besar (misal 16x16) akan membutuhkan 256 node dan melebihi budget RAM.

### 17.3 Default OFF

`ENABLE_PATH_PLANNER` default `false` karena:
1. RAM terbatas (node pool + open set ≈ 3.5 KB dari 8 KB).
2. Masih dalam status passive map builder — manfaat runtime masih terbatas.
3. Menambah ukuran flash code (~500 baris implementasi).

---

## 18. Contoh Eksekusi A* Lengkap

Misalkan robot di grid (4,4), goal di (7,1), tidak ada obstacle:

1. Start node (4,4): g=0, h=heuristic(4,4,7,1) = 1.414×min(3,3) + |3-3| = 1.414×3 + 0 = 4.242. f=4.242. Dimasukkan open set.
2. Pop (4,4). Cek 8 tetangga. Tetangga dengan fCost terendah biasanya arah menuju goal (SE).
3. Setiap tetangga mendapat gCost = 1.0 (orthogonal) atau 1.414 (diagonal), hCost dihitung, parent = current, dimasukkan open set.
4. Proses berulang: node dengan fCost terendah diekspansi hingga goal (7,1) terpop.
5. Jalur direkonstruksi dari chain parent, dibalik, disimpan ke `currentPath.waypoints`.

Jika ada obstacle di cell tengah, penalti +2.0 per obstacle dalam 3x3 akan membuat A* memilih jalur yang melinting di sekitarnya.

---

## 19. Function-by-Function Breakdown

Berikut adalah rincian setiap fungsi publik dan privat class `PathPlanner`:

### Fungsi Publik

| Fungsi | File:Baris | Deskripsi |
|--------|-----------|-----------|
| `PathPlanner()` | path_planner.cpp:10 | Konstruktor: inisialisasi node pool + reset |
| `begin(int, int)` | path_planner.cpp:22 | Set posisi start, tandai area start |
| `reset()` | path_planner.cpp:37 | Reset grid, node, goal, path, open set |
| `updateMap(SensorGaris&, int, int, float)` | path_planner.cpp:56 | Update peta dari sensor IR + ultrasonic |
| `setGoal(int, int)` | path_planner.cpp:111 | Set goal + cari jalur A* |
| `findPath(int, int, int, int)` | path_planner.cpp:121 | Algoritma A* utama |
| `getNextWaypoint(int, int, int&, int&)` | path_planner.cpp:222 | Dapatkan waypoint berikutnya |
| `isBlocked(int, int) const` | path_planner.cpp:246 | Cek apakah cell blocked |
| `markObstacle(int, int, int=1)` | path_planner.cpp:252 | Tandai obstacle (circle fill) |
| `markLine(int, int)` | path_planner.cpp:264 | Tandai cell sebagai garis |
| `worldToGrid(float, float, int&, int&) const` | path_planner.cpp:270 | Konversi world ke grid |
| `gridToWorld(int, int, float&, float&) const` | path_planner.cpp:278 | Konversi grid ke world |
| `printMap()` | path_planner.cpp:400 | Debug: cetak peta grid ASCII |
| `printPath(const PathResult&)` | path_planner.cpp:434 | Debug: cetak daftar waypoint |
| `handleCommand(const char*)` | path_planner.cpp:446 | Parser perintah serial |

### Fungsi Privat (Helpers)

| Fungsi | File:Baris | Deskripsi |
|--------|-----------|-----------|
| `heuristic(int, int, int, int) const` | path_planner.cpp:285 | Octile distance |
| `isValid(int, int) const` | path_planner.cpp:292 | Cek batas grid |
| `moveCost(int, int, int, int) const` | path_planner.cpp:296 | Biaya gerak (tidak digunakan di findPath) |
| `getNode(int, int)` | path_planner.cpp:303 | Dapatkan pointer node dari koordinat |
| `clearNodes()` | path_planner.cpp:308 | Reset semua node pool |
| `openSetPush(PathNode*)` | path_planner.cpp:318 | Masukkan ke min-heap |
| `openSetPop()` | path_planner.cpp:335 | Keluarkan node terendah dari min-heap |
| `openSetRemove(PathNode*)` | path_planner.cpp:364 | Hapus node dari heap (tidak digunakan) |
| `openSetContains(int, int) const` | path_planner.cpp:393 | Cek keberadaan dalam open set |
| `cekHalanganGrid()` | path_planner.h:135 | Deklarasi untuk cek ultrasonic (diupdateMap) |

---

## 20. Perbandingan dengan Mapper Subsystem

Path Planner dan Mapper adalah dua subsistem terpisah yang dapat hidup berdampingan:

| Aspek | Path Planner | Mapper (mapping.h) |
|-------|-------------|-------------------|
| **Algoritma** | A* pada grid 8x8 | Rekam/prediksi event persimpangan |
| **Tujuan** | Perencanaan jalur & obstacle avoidance | Optimasi balap berdasarkan riwayat lintasan |
| **Data** | Grid cell (64), node pool (64), waypoint (32) | Waypoint event (maks 30) |
| **Persistensi** | Tidak ada (reset tiap boot) | EEPROM (simpan/muat waypoint) |
| **Aktif saat** | `ENABLE_PATH_PLANNER` | Mode LEARNING / RACING |
| **Integrasi loop** | `updateMap` dipanggil, tapi passive | `recordEvent` di persimpangan, `checkPrediction` di RACING |
| **Prediksi** | Tidak ada (hanya cari jalur statis) | Memprediksi event 150ms mendatang |
| **Obstacle** | Ya (ultrasonic → grid) | Tidak |
| **File** | path_planner.h/.cpp | mapping.h |

**Mapper** mencatat event persimpangan dengan time offset dalam mode learning. Dalam mode racing, `checkPrediction()` memprediksi event berikutnya berdasarkan waktu relatif sejak run dimulai, dengan feedback LED (putih = lurus, hijau = kiri, merah = kanan, biru = tidak ada prediksi).

**Path Planner** adalah subsistem terpisah yang menggunakan grid + A* untuk perencanaan rute dan obstacle avoidance. Saat ini tidak terintegrasi langsung dengan Mapper — keduanya beroperasi independen. Di masa depan, Mapper dapat memberikan data lintasan yang sudah dipelajari sebagai input awal ke Path Planner, atau Path Planner dapat memberikan rute alternatif yang diikuti oleh robot dalam mode racing.

---

## 21. Gotchas dan Keterbatasan

1. **RAM ATMega2560 (8KB):** Grid 8x8 dipilih untuk hemat RAM. Node pool 64 node berkontribusi signifikan pada penggunaan memori.
2. **Default OFF:** `ENABLE_PATH_PLANNER = false` di config.h.
3. **Node pool 64 node:** Maksimal grid 8x8 karena `MAX_NODES = GRID_WIDTH * GRID_HEIGHT`.
4. **Max 32 waypoint:** `MAX_WAYPOINTS = 32` membatasi panjang jalur yang dapat disimpan.
5. **Passive builder:** `getNextWaypoint` tidak dipanggil di loop utama; robot belum mengikuti jalur A*.
6. **Posisi dan heading hardcoded:** Robot selalu di tengah grid (4,4), heading selalu 0 (Utara).
7. **Inkonsistensi `CELL_SIZE_CM`:** `path_planner.h` = 40 cm, `config.h` = 20 cm (tidak digunakan).
8. **`moveCost()` tidak digunakan:** Biaya diambil langsung dari array statis dalam `findPath`, bukan dari fungsi `moveCost()`.
9. **Potensi typo heuristik:** `int dy = abs(x1 - x2);` seharusnya `abs(y1 - y2)` (hasil tetap sama karena simetris).
10. **90° turn helpers blocking:** `belokKiri90`/`belokKanan90` di `penggerak` menggunakan `while` loop — tidak terkait langsung Path Planner tetapi perlu diperhatikan saat integrasi gerak.
11. **Komentar dalam Bahasa Indonesia:** Pertahankan konsistensi bahasa saat menambah atau mengedit kode Path Planner.

---

## 22. Rekomendasi Pengembangan Lanjutan

1. **Integrasi posisi robot:** Sertakan odometri atau IMU untuk memperbarui `robotGridX`/`robotGridY` dan `robotHeading` secara real-time sebelum memanggil `updateMap`.
2. **Panggil `getNextWaypoint` di loop:** Setelah `setGoal`, gunakan `getNextWaypoint` untuk menghasilkan target navigasi yang dikirim ke penggerak.
3. **Selesaikan inkonsistensi `CELL_SIZE_CM`:** Pilih satu nilai (20 atau 40 cm) dan terapkan konsisten di `path_planner.h` dan `config.h`.
4. **Aktifkan `openSetRemove`:** Gunakan untuk memperbarui node yang sudah dalam open set dengan biaya lebih rendah, bukan hanya menambahkan entri duplikat.
5. **Integrasi dengan Mapper:** Gunakan data waypoint Mapper sebagai goal candidate otomatis untuk Path Planner.
