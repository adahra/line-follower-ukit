// path_planner.cpp — Implementasi Path Planning dengan A*
#include "path_planner.h"
#include "ukit_macros.h"
#include <math.h>

// Global instance defined in LineFollowerFin.ino
// PathPlanner pathPlanner;

PathPlanner::PathPlanner() {
  // Inisialisasi node pool
  for (int x = 0; x < GRID_WIDTH; x++) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
      nodes[x][y] = &nodePool[x * GRID_HEIGHT + y];
      nodes[x][y]->x = x;
      nodes[x][y]->y = y;
    }
  }
  reset();
}

void PathPlanner::begin(int startX, int startY) {
  reset();
  robotGridX = constrain(startX, 0, GRID_WIDTH - 1);
  robotGridY = constrain(startY, 0, GRID_HEIGHT - 1);
  // Mark start area as empty
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      int nx = robotGridX + dx;
      int ny = robotGridY + dy;
      if (isValid(nx, ny)) grid[nx][ny] = CELL_EMPTY;
    }
  }
  grid[robotGridX][robotGridY] = CELL_START;
}

void PathPlanner::reset() {
  for (int x = 0; x < GRID_WIDTH; x++) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
      grid[x][y] = CELL_UNKNOWN;
      nodes[x][y]->gCost = 0;
      nodes[x][y]->hCost = 0;
      nodes[x][y]->parent = nullptr;
      nodes[x][y]->closed = false;
      nodes[x][y]->open = false;
    }
  }
  goalX = -1;
  goalY = -1;
  hasGoal_ = false;
  currentPath.count = 0;
  currentWaypointIndex = 0;
  openSetSize = 0;
}

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
  if (leftIR >= 8 && rightIR >= 8) {  // threshold approx
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
  
  // TODO: Integrate ultrasonic untuk obstacle detection
  // if (ultrasonic < threshold) markObstacle(...)
}

void PathPlanner::setGoal(int goalX_, int goalY_) {
  goalX = constrain(goalX_, 0, GRID_WIDTH - 1);
  goalY = constrain(goalY_, 0, GRID_HEIGHT - 1);
  hasGoal_ = true;
  currentWaypointIndex = 0;
  
  // Cari path baru
  currentPath = findPath(robotGridX, robotGridY, goalX, goalY);
}

PathResult PathPlanner::findPath(int startX, int startY, int goalX_, int goalY_) {
  PathResult result;
  result.found = false;
  result.count = 0;
  result.totalCost = 0;
  
  if (!isValid(startX, startY) || !isValid(goalX_, goalY_)) {
    return result;
  }
  
  if (isBlocked(goalX_, goalY_)) {
    return result; // Goal blocked
  }
  
  clearNodes();
  openSetSize = 0;
  
  PathNode *startNode = nodes[startX][startY];
  startNode->gCost = 0;
  startNode->hCost = heuristic(startX, startY, goalX_, goalY_);
  startNode->parent = nullptr;
  startNode->open = true;
  openSetPush(startNode);
  
  PathNode *goalNode = nodes[goalX_][goalY_];
  
  while (openSetSize > 0) {
    PathNode *current = openSetPop();
    current->open = false;
    current->closed = true;
    
    // Check if reached goal
    if (current == goalNode) {
      // Reconstruct path
      PathNode *pathNode = current;
      int pathLength = 0;
      while (pathNode && pathLength < 64) {
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
    
    // Check neighbors (8-connected)
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
      
      // Penalty untuk dekat obstacle
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
  }
  
  return result; // No path found
}

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

bool PathPlanner::isBlocked(int x, int y) const {
  if (!isValid(x, y)) return true;
  CellType t = grid[x][y];
  return (t == CELL_OBSTACLE); // Only hard obstacles block
}

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

void PathPlanner::markLine(int x, int y) {
  if (isValid(x, y) && grid[x][y] != CELL_OBSTACLE) {
    grid[x][y] = CELL_LINE;
  }
}

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

float PathPlanner::heuristic(int x1, int y1, int x2, int y2) const {
  // Octile distance heuristic (better for 8-connected grid)
  int dx = abs(x1 - x2);
  int dy = abs(y1 - y2);
  return 1.414 * min(dx, dy) + abs(dx - dy);
}

bool PathPlanner::isValid(int x, int y) const {
  return (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT);
}

float PathPlanner::moveCost(int x1, int y1, int x2, int y2) const {
  int dx = abs(x1 - x2);
  int dy = abs(y1 - y2);
  if (dx == 1 && dy == 1) return 1.414; // Diagonal
  return 1.0; // Orthogonal
}

PathNode* PathPlanner::getNode(int x, int y) {
  if (isValid(x, y)) return nodes[x][y];
  return nullptr;
}

void PathPlanner::clearNodes() {
  for (int i = 0; i < MAX_NODES; i++) {
    nodePool[i].gCost = 0;
    nodePool[i].hCost = 0;
    nodePool[i].parent = nullptr;
    nodePool[i].closed = false;
    nodePool[i].open = false;
  }
}

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

void PathPlanner::openSetRemove(PathNode *node) {
  for (int i = 0; i < openSetSize; i++) {
    if (openSet[i] == node) {
      openSet[i] = openSet[--openSetSize];
      // Re-heapify dari posisi i
      // Simplified: just rebuild heap (acceptable for small size)
      for (int j = openSetSize / 2 - 1; j >= 0; j--) {
        int k = j;
        while (true) {
          int smallest = k;
          int left = 2 * k + 1;
          int right = 2 * k + 2;
          if (left < openSetSize && openSet[left]->fCost() < openSet[smallest]->fCost())
            smallest = left;
          if (right < openSetSize && openSet[right]->fCost() < openSet[smallest]->fCost())
            smallest = right;
          if (smallest != k) {
            PathNode *tmp = openSet[k];
            openSet[k] = openSet[smallest];
            openSet[smallest] = tmp;
            k = smallest;
          } else break;
        }
      }
      break;
    }
  }
}

bool PathPlanner::openSetContains(int x, int y) const {
  for (int i = 0; i < openSetSize; i++) {
    if (openSet[i]->x == x && openSet[i]->y == y) return true;
  }
  return false;
}

void PathPlanner::printMap() {
  Serial.println(F("\n=== PATH PLANNER MAP ==="));
  Serial.print(F("Robot: (")); Serial.print(robotGridX); Serial.print(F(",")); Serial.print(robotGridY); Serial.println(F(")"));
  if (hasGoal_) {
    Serial.print(F("Goal: (")); Serial.print(goalX); Serial.print(F(",")); Serial.print(goalY); Serial.println(F(")"));
  }
  Serial.println(F("Legend: .=UNK  =EMP  #=LINE  O=OBS  S=STA  G=GOL  @=ROB"));
  Serial.println(F("  Y"));
  
  for (int y = 0; y < GRID_HEIGHT; y++) {
    if (y < 10) Serial.print(F(" "));
    Serial.print(y); Serial.print(F(" "));
    for (int x = 0; x < GRID_WIDTH; x++) {
      char c = '.';
      if (x == robotGridX && y == robotGridY) c = '@';
      else if (x == goalX && y == goalY) c = 'G';
      else {
        switch (grid[x][y]) {
          case CELL_EMPTY: c = ' '; break;
          case CELL_LINE: c = '#'; break;
          case CELL_OBSTACLE: c = 'O'; break;
          case CELL_START: c = 'S'; break;
          case CELL_VISITED: c = '*'; break;
          default: c = '.'; break;
        }
      }
      Serial.print(c);
    }
    Serial.println();
  }
  Serial.println(F("  0123456789... X"));
  Serial.println(F("========================\n"));
}

void PathPlanner::printPath(const PathResult &path) {
  if (!path.found) {
    Serial.println(F("Path: NOT FOUND"));
    return;
  }
  Serial.print(F("Path found: ")); Serial.print(path.count); Serial.print(F(" waypoints, cost: ")); Serial.println(path.totalCost, 1);
  for (int i = 0; i < path.count; i++) {
    Serial.print(F("  [")); Serial.print(i); Serial.print(F("] (")); 
    Serial.print(path.waypoints[i][0]); Serial.print(F(",")); Serial.print(path.waypoints[i][1]); Serial.println(F(")"));
  }
}

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
  
  if (strcmp(key, "path") == 0) {
    if (strcmp(valStr, "map") == 0) {
      printMap();
    } else if (strcmp(valStr, "find") == 0) {
      if (hasGoal_) {
        currentPath = findPath(robotGridX, robotGridY, goalX, goalY);
        printPath(currentPath);
      } else {
        Serial.println(F("No goal set. Use: path goal <x> <y>"));
      }
    } else if (strcmp(valStr, "goal") == 0) {
      // Parse: path goal <x> <y>
      int gx, gy;
      if (sscanf(valStr + 5, "%d %d", &gx, &gy) == 2) { // skip "goal "
        setGoal(gx, gy);
        Serial.print(F("Goal set to (")); Serial.print(goalX); Serial.print(F(",")); Serial.print(goalY); Serial.println(F(")"));
        printPath(currentPath);
      } else {
        Serial.println(F("Usage: path goal <x> <y>"));
      }
    } else if (strcmp(valStr, "reset") == 0) {
      reset();
      Serial.println(F("Path planner reset"));
    } else if (strcmp(valStr, "next") == 0) {
      int nx, ny;
      if (getNextWaypoint(robotGridX, robotGridY, nx, ny)) {
        Serial.print(F("Next waypoint: (")); Serial.print(nx); Serial.print(F(",")); Serial.print(ny); Serial.println(F(")"));
      } else {
        Serial.println(F("No active path or goal reached"));
      }
    } else if (strcmp(valStr, "obstacle") == 0) {
      int ox, oy;
      if (sscanf(valStr + 9, "%d %d", &ox, &oy) == 2) { // skip "obstacle "
        markObstacle(ox, oy, 1);
        Serial.print(F("Obstacle marked at (")); Serial.print(ox); Serial.print(F(",")); Serial.print(oy); Serial.println(F(")"));
      } else {
        Serial.println(F("Usage: path obstacle <x> <y>"));
      }
    } else {
      Serial.println(F("Path commands: map, find, goal <x> <y>, reset, next, obstacle <x> <y>"));
    }
  }
}