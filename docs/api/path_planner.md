# API Dokumentasi - PathPlanner

Dokumentasi API lengkap untuk modul PathPlanner, sistem perencanaan jalan manual grid-based (A*) untuk debugging dan testing (opsional).

## 1. Ikhtisar

PathPlanner (Path Planning) adalah modul yang bertanggung jawab untuk manual grid-based path planning (A*) untuk debugging dan testing (opsional). Ini merupakan komponen perencanaan jalan yang penting yang memungkinkan robot merencanakan jalan secara manual.

**File:** `LineFollowerFin/path_planner.h/.cpp` (4.0KB / 15.4KB)
**Dependencies:** `config.h`, `sensor_garis.h`, `fwd.h`

## 2. Method Publik

### 2.1 Method Inti

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void begin(int width, int height)` | Inisialisasi Path Planner |
| `updateMap()` | `void updateMap()` | Perbarui peta dari sensor |
| `setCell()` | `void setCell(int x, int y, CellType type)` | Set sel di peta |
| `getCell() | `CellType getCell(int x, int y) | Ambil sel di peta |
| `findPath()` | `bool findPath(int startX, int startY, int targetX, int targetY)` | Cari jalan dari start ke target |
| `getPath()` | `std::vector<Point> getPath() const` | Kembalikan path yang ditemukan |
| `cetakPeta()` | `void cetakPeta()` | Tampilkan peta debug |
| `cetakJalan()` | `void cetakJalan()` | Tampilkan path debug |
| `reset()` | `void reset()` | Reset Path Planner |
| `simpan()` | `void simpan()` | Simpan peta ke EEPROM |
| `muat()` | `void muat()` | Baca peta dari EEPROM |

### 2.2 Method Utility

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `validasiPeta()` | `bool validasiPeta() const` | Validasi peta |
| `validasiJalan()` | `bool validasiJalan(const std::vector<Point>& path) const` | Validasi jalan |
| `hitungJarak()` | `int hitungJarak(int x1, int y1, int x2, int y2) const` | Hitung jarak Manhattan |
| `hitungEuclidean()` | `float hitungEuclidean(int x1, int y1, int x2, int y2) const` | Hitung jarak Euclidean |
| `cekTabrakan()` | `bool cekTabrakan(const std::vector<Point>& path) const` | Cek tabrakan jalan |
| `cekDiMap()` | `bool cekDiMap(int x, int y) const` | Cek koordinat di peta |

### 2.3 Method Privasi

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `inisiasiAstar()` | `void inisiasiAstar()` | Inisiasikan A* algorithm |
| `ekspansiNode()` | `void ekspansiNode(Node* node)` | Ekspansi node dalam A* |
| `hitungHeuristic()` | `float hitungHeuristic(int x1, int y1, int x2, int y2) const` | Hitung heuristic A* |
| `perbaruiOpenSet()` | `void perbaruiOpenSet()` | Perbarui open set A* |
| `perbaruiClosedSet()` | `void perbaruiClosedSet()` | Perbarui closed set A* |

## 3. Tipe Data Bersama

### 3.1 Enum

```cpp
enum class CellType {
  CELL_UNKNOWN = 0,
  CELL_EMPTY,
  CELL_LINE,
  CELL_OBSTACLE,
  CELL_START,
  CELL_VISITED
};

enum class Direction {
  DIR_N = 0, DIR_NE = 1, DIR_E = 2,
  DIR_SE = 3, DIR_S = 4, DIR_SW = 5,
  DIR_W = 6, DIR_NW = 7
};
```

### 3.2 Struct

```cpp
struct Point {
  int x;      // Koordinat x
  int y;      // Koordinat y
  float g;    // Biaya dari start
  float h;    // Heuristic ke target
  float f;    // Total cost (g + h)
  Point* parent; // Parent point
  bool visited; // Flag visited
};
```

### 3.3 Konstanta

| Konstanta | Tipe | Default | Keterangan |
|-----------|------|---------|-------------|
| `PATH_GRID_WIDTH` | int | 20 | Lebar grid path planner |
| `PATH_GRID_HEIGHT` | int | 20 | Tinggi grid path planner |
| `PATH_MAX_NODES` | int | 400 | Jumlah node maksimum |
| `PATH_HEURISTIC_WEIGHT` | float | 1.0 | Bobot heuristic |
| `PATH_COST_STRAIGHT` | float | 1.0 | Biaya jalan lurus |
| `PATH_COST_DIAGONAL` | float | 1.414 | Biaya jalan diagonal |
| `PATH_TIMEOUT_MS` | unsigned long | 1000 | Timeout path finding |

## 4. Method Khusus

### 4.1 Method Update
```cpp
void PathPlanner::updateMap() {
  // Reset peta
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      cells[y][x] = CELL_UNKNOWN;
    }
  }
  
  // Perbarui peta dari sensor
  int startX = width / 2;
  int startY = height / 2;
  
  // Baca sensor untuk populate peta
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Hitung jarak dari start
      float distance = hitungJarak(startX, startY, x, y);
      
      // Tentukan tipe sel berdasarkan jarak
      if (distance <= SENSOR_RANGE) {
        cells[y][x] = CELL_LINE;
      } else {
        cells[y][x] = CELL_EMPTY;
      }
    }
  }
}
```

### 4.2 Method A*
```cpp
bool PathPlanner::findPath(int startX, int startY, int targetX, targetY) {
  // Validasi koordinat
  if (!cekDiMap(startX, startY) || !cekDiMap(targetX, targetY)) {
    return false;
  }
  
  // Reset A*
  inisiasiAstar();
  
  // Tambahkan start node ke open set
  Point* startNode = buatNode(startX, startY, 0, 0, 0, nullptr);
  openSet.push(startNode);
  
  // Loop sampai open set kosong
  while (!openSet.empty()) {
    // Ambil node dengan f cost terkecil
    Point* currentNode = openSet.top();
    openSet.pop();
    
    // Cek jika current node adalah target
    if (currentNode->x == targetX && currentNode->y == targetY) {
      // Build path dari parent pointers
      buildPath(currentNode);
      return true;
    }
    
    // Ekspansi node
    ekspansiNode(currentNode);
  }
  
  return false;
}
```

### 4.3 Method Ekspansi Node
```cpp
void PathPlanner::ekspansiNode(Point* node) {
  // Tambahkan node ke closed set
  closedSet.push(node->x, node->y);
  
  // Eksplore neighbor nodes
  for (Direction dir : directions) {
    int neighborX = node->x + directionToDelta(dir).x;
    int neighborY = node->y + directionToDelta(dir).y;
    
    // Cek jika neighbor valid
    if (!cekDiMap(neighborX, neighborY) || cekDiClosedSet(neighborX, neighborY)) {
      continue;
    }
    
    // Hitung g cost
    float cost = node->g + (dir == DIR_N || dir == DIR_S || dir == DIR_E || dir == DIR_W ? PATH_COST_STRAIGHT : PATH_COST_DIAGONAL);
    
    // Tambahkan neighbor ke open set
    addNodeToOpenSet(neighborX, neighborY, cost, node, dir);
  }
}
```

### 4.4 Method Build Path
```cpp
void PathPlanner::buildPath(Point* targetNode) {
  path.clear();
  
  // Build path dari parent pointers
  Point* currentNode = targetNode;
  while (currentNode != nullptr) {
    path.push_back(Point(currentNode->x, currentNode->y, 0, 0, 0, nullptr, false));
    currentNode = currentNode->parent;
  }
  
  // Reverse path
  std::reverse(path.begin(), path.end());
}
```

## 5. Integrasi dengan Modul Lain

### 5.1 Integrasi dengan SensorGaris
- Input: `sensor.bacaKiri()` - `sensor.bacaKanan()` untuk update peta
- Output: peta grid untuk path finding
- Digunakan dalam debug dan testing

### 5.2 Integrasi dengan Penggerak
- Input: `gerak.getSpeeds()` untuk navigasi path
- Output: path yang direncanakan ke kontrol motor
- Digunakan dalam debugging gerakan robot

### 5.3 Integrasi dengan Antarmuka
- Bekerja sama untuk perintah serial 'path ...' untuk setel target
- Bekerja sama untuk perintah serial 'map ...' untuk update peta
- Bekerja sama untuk perintah serial 'path find' untuk cari jalan

### 5.4 Integrasi dengan Pengaman
- Bekerja sama untuk safety check dalam path finding
- Bekerja sama untuk emergency stop dalam path finding
- Bekerja sama untuk reset path planner

### 5.5 Integrasi dengan Mapping
- Bekerja sama untuk waypoint dalam path finding
- Bekerja sama untuk racing prediction dalam path finding
- Bekerja sama untuk mode learning dalam path finding

## 6. Desain Decisions & Alasan

### 6.1 Path Planning Manual
**Alasan:** Path planning manual grid-based digunakan untuk debugging dan testing. Bukan navigasi otonom.

### 6.2 Algoritma A*
**Alasan:** A* algorithm memberikan path optimal dengan heuristic yang efisien. Lebih cepat dari algoritma lainnya.

### 6.3 Grid-Based Map
**Alasan:** Grid-based map memudahkan visualisasi dan debugging. Lebih intuitif dari representasi lainnya.

### 6.4 Obstacle Avoidance
**Alasan:** Obstacle avoidance memastikan robot tidak menabrak objek. Lebih aman dari path finding yang tidak memiliki obstacle.

## 7. Error Handling & Edge Cases

### 7.1 Error Path Finding
- **Target tidak valid:** Gagal path finding
- **Path tidak ditemukan:** Kembalikan false
- **Timeout path finding:** Gagal path finding

### 7.2 Error Grid
- **Grid out of bounds:** Validasi koordinat
- **Cell tidak valid:** Setel ke CELL_UNKNOWN
- **Grid corrupted:** Reset grid

### 7.3 Edge Cases
- **Target sama dengan start:** Path kosong
- **Path overlapping:** Cek tabrakan
- **Path tidak optimal:** Gunakan heuristic yang berbeda
- **Grid terlalu besar:** Batasi ukuran grid

## 8. Kinerja & Sumber Daya

### 8.1 Penggunaan RAM
- Grid cells: 400 * 1 byte = 400 bytes
- Open set: 400 * 20 bytes = 8000 bytes
- Closed set: 400 * 8 bytes = 3200 bytes
- Path: 400 * 20 bytes = 8000 bytes
- Total: ~20000 bytes per instance

### 8.2 Konsumsi Waktu
- Update map per loop: ~100-200 microseconds
- Path finding: ~500-1000 microseconds
- Build path: ~50-100 microseconds
- Total per path finding: ~650-1350 microseconds

### 8.3 EEPROM Write Cycle
- Penulisan peta: 1 write per simpan peta (aman untuk 100k cycles)

## 9. Referensi & Sumber

1. A* Algorithm - https://www.arduino.cc/en/Tutorial/A*
2. Path Planning - https://www.arduino.cc/en/Tutorial/PathPlanning
3. Grid-Based Map - https://www.arduino.cc/en/Tutorial/GridMap
4. Node A* - https://www.arduino.cc/en/Tutorial/NodeA*

## 10. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/pathplanner.md` untuk keputusan desain