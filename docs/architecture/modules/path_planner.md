# Modul PathPlanner - Dokumentasi Teknis

## 1. Ikhtisar

PathPlanner adalah path planning dengan A* dan obstacle avoidance untuk LineFollowerFin. Ini grid-based mapping untuk navigasi autonom, tapi bukan navigasi otonom (tidak ada odometri).

**File:** `LineFollowerFin/path_planner.h/.cpp` (4.0KB / 15.4KB)
**Dependencies:** `Arduino.h`, `fwd.h`, `config.h`, `sensor_garis.h`

## 2. Desain & Arsitektur

### 2.1 Konfigurasi Grid
- **Grid size:** 8x8 cells (GRID_WIDTH = 8, GRID_HEIGHT = 8)
- **Cell size:** 20cm per cell (CELL_SIZE_CM = 20)
- **Memory:** Static array CellType grid[GRID_WIDTH][GRID_HEIGHT]
- **Obstacle:** Ultrasonic detection + manual marking

### 2.2 Tipe Cell
```cpp
enum CellType {
  CELL_UNKNOWN = 0,
  CELL_EMPTY = 1,
  CELL_LINE = 2,      // Garis terdeteksi
  CELL_OBSTACLE = 3,  // Halangan (ultrasonic)
  CELL_START = 4,     // Start/finish line
  CELL_VISITED = 5    // Sudah dilewati
};
```

### 2.3 A* Path Planning
- **Nodes:** PathNode struct dengan gCost, hCost, fCost
- **Open set:** Priority queue untuk A*
- **Closed set:** Node yang sudah diproses
- **Node pool:** Static array untuk menghindari malloc

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void` | Inisialisasi path planner |
| `reset()` | `void` | Reset grid & path |
| `updateMap()` | `void updateMap(SensorGaris &sensor, int robotX, int robotY, float robotHeading)` | Update map dari sensor |
| `setGoal()` | `void setGoal(int goalX, int goalY)` | Set goal position |
| `findPath()` | `PathResult findPath(int startX, int startY, int goalX, int goalY)` | Cari path dengan A* |
| `getNextWaypoint()` | `bool getNextWaypoint(int robotX, int robotY, int &nextX, int &nextY)` | Dapatkan waypoint berikutnya untuk robot |
| `isBlocked()` | `bool isBlocked(int x, int y) const` | Cek apakah cell blocked |
| `markObstacle()` | `void markObstacle(int x, int y, int radius = 1)` | Mark obstacle dari ultrasonic |
| `markLine()` | `void markLine(int x, int y)` | Mark line detected |
| `printMap()` | `void printMap()` | Debug print map |
| `printPath()` | `void printPath(const PathResult &path)` | Debug print path |
| `handleCommand()` | `void handleCommand(const char *cmd)` | Serial command handler |

## 4. Integrasi dengan Modul Lain

### 4.1 Integrasi dengan SensorGaris
- Bekerja sama untuk updateMap(): `sensor.bacaKiri()`, `sensor.bacaKanan()`
- Bekerja sama untuk marking: `markLine()`
- Bekerja sama untuk obstacle: `markObstacle()`

### 4.2 Integrasi dengan Pengaman
- Bekerja sama untuk obstacle detection: `cekHalanganGrid()`
- Bekerja sama untuk safety: Halangan ultrasonik mencegah path planning

### 4.3 Integrasi dengan Antarmuka
- Bekerja sama untuk command serial: `handleCommand()`
- Bekerja sama untuk manual map update

## 5. Desain Decisions & Alasan

### 5.1 Manual Planning
**Alasan:** Path planning manual (user-defined) karena tidak ada odometri untuk posisi robot yang akurat. Grid-based mapping lebih andal.

### 5.2 Grid-Based
**Alasan:** Grid-based lebih sederhana dari graph-based. 8x8 hemat RAM (ATMega2560: 8KB).

### 5.3 Static Memory
**Alasan:** Hindari malloc di embedded system. Node pool statis menghindari fragmentasi.

### 5.4 A* Algorithm
**Alasan:** A* efisien untuk grid kecil. Heuristic (Manhattan distance) admissible dan consistent.

## 6. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/path_planner.md` untuk dokumentasi API lengkap

