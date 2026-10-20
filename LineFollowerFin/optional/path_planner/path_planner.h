// path_planner.h — Path planning dengan A* dan obstacle avoidance
// Grid-based mapping untuk navigasi autonom
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <Arduino.h>
#include "fwd.h"
#include "config.h"
#include "sensor_garis.h"

// Ukuran grid map (cells) - 16x16 untuk hemat RAM (ATMega2560: 8KB)
#define GRID_WIDTH  16
#define GRID_HEIGHT 16
#define CELL_SIZE_CM 20  // 20cm per cell (area 3.2m x 3.2m)

// Tipe cell
enum CellType {
  CELL_UNKNOWN = 0,
  CELL_EMPTY = 1,
  CELL_LINE = 2,      // Garis terdeteksi
  CELL_OBSTACLE = 3,  // Halangan (ultrasonic)
  CELL_START = 4,     // Start/finish line
  CELL_VISITED = 5    // Sudah dilewati
};

// Arah gerak (8-connected)
enum Direction {
  DIR_N  = 0,  // North (atas)
  DIR_NE = 1,  // North-East
  Dir_E  = 2,  // East (kanan)
  DIR_SE = 3,  // South-East
  DIR_S  = 4,  // South (bawah)
  DIR_SW = 5,  // South-West
  DIR_W  = 6,  // West (kiri)
  DIR_NW = 7   // North-West
};

// Node untuk A*
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

// Path result
struct PathResult {
  int waypoints[64][2];  // x,y pairs
  int count = 0;
  bool found = false;
  float totalCost = 0;
};

class PathPlanner {
 public:
  PathPlanner();
  
  // Inisialisasi
  void begin(int startX, int startY);
  void reset();
  
  // Update map dari sensor
  void updateMap(SensorGaris &sensor, int robotX, int robotY, float robotHeading);
  
  // Set goal position
  void setGoal(int goalX, int goalY);
  
  // Cari path dengan A*
  PathResult findPath(int startX, int startY, int goalX, int goalY);
  
  // Get next waypoint untuk robot follow
  bool getNextWaypoint(int robotX, int robotY, int &nextX, int &nextY);
  
  // Obstacle avoidance: cek apakah cell blocked
  bool isBlocked(int x, int y) const;
  
  // Mark obstacle dari ultrasonic
  void markObstacle(int x, int y, int radius = 1);
  
  // Mark line detected
  void markLine(int x, int y);
  
  // Convert world ke grid
  void worldToGrid(float worldX, float worldY, int &gridX, int &gridY) const;
  void gridToWorld(int gridX, int gridY, float &worldX, float &worldY) const;
  
  // Debug
  void printMap();
  void printPath(const PathResult &path);
  
  // Serial commands
  void handleCommand(const char *cmd);
  
  // Getters
  int getRobotGridX() const { return robotGridX; }
  int getRobotGridY() const { return robotGridY; }
  bool hasGoal() const { return hasGoal_; }
  int getGoalX() const { return goalX; }
  int getGoalY() const { return goalY; }

 private:
  // Grid map
  CellType grid[GRID_WIDTH][GRID_HEIGHT];
  
  // Robot position dalam grid
  int robotGridX = GRID_WIDTH / 2;
  int robotGridY = GRID_HEIGHT / 2;
  float robotHeading = 0;  // derajat, 0 = Utara
  
  // Goal
  int goalX = -1;
  int goalY = -1;
  bool hasGoal_ = false;
  
  // Current path
  PathResult currentPath;
  int currentWaypointIndex = 0;
  
  // A* helpers
  float heuristic(int x1, int y1, int x2, int y2) const;
  bool isValid(int x, int y) const;
  float moveCost(int x1, int y1, int x2, int y2) const;
  PathNode* getNode(int x, int y);
  void clearNodes();
  
  // Node pool untuk A* (avoid malloc)
  static const int MAX_NODES = GRID_WIDTH * GRID_HEIGHT;
  PathNode nodePool[MAX_NODES];
  PathNode* nodes[GRID_WIDTH][GRID_HEIGHT];
  
  // Open set untuk A* (simple array-based priority queue)
  PathNode* openSet[MAX_NODES];
  int openSetSize = 0;
  
  void openSetPush(PathNode *node);
  PathNode* openSetPop();
  void openSetRemove(PathNode *node);
  bool openSetContains(int x, int y) const;
};

// Global path planner instance (defined in .ino)
extern PathPlanner pathPlanner;

#endif