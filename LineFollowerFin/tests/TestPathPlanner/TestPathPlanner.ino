// TestPathPlanner.ino — Test mandiri Path Planning A*
// Upload ke uKit Explore 2, buka Serial Monitor 115200
// Test grid mapping, A* pathfinding, obstacle avoidance

#include "ukit_wrapper.h"

// ===== KONFIGURASI GRID =====
const int GRID_W = 16;
const int GRID_H = 16;
const int CELL_SIZE = 10; // cm

enum CellType { UNK=0, EMP=1, LINE=2, OBS=3, STA=4, VIS=5 };

struct Node {
  int x, y;
  float g, h;
  Node* parent;
  bool open, closed;
};

CellType grid[GRID_W][GRID_H];
Node nodes[GRID_W][GRID_H];
Node* openSet[GRID_W * GRID_H];
int openSize = 0;

int robotX = GRID_W/2, robotY = GRID_H/2;
int goalX = -1, goalY = -1;
bool hasGoal = false;

int pathX[64], pathY[64];
int pathLen = 0, pathIdx = 0;

// ===== SETUP =====
void setup() {
  Initialization();
  Serial.begin(115200);
  delay(500);
  
  resetPlanner();
  
  Serial.println(F("\n=== TEST PATH PLANNER A* ==="));
  Serial.println(F("Grid: 16x16, Cell: 10cm"));
  Serial.println(F("Robot di tengah (8,8)"));
  Serial.println(F(""));
  Serial.println(F("Perintah:"));
  Serial.println(F("  map           - tampilkan map"));
  Serial.println(F("  goal x y      - set goal (contoh: goal 12 5)"));
  Serial.println(F("  find          - cari path ke goal"));
  Serial.println(F("  next          - waypoint berikutnya"));
  Serial.println(F("  obs x y       - tambah obstacle"));
  Serial.println(F("  line x y      - tambah garis"));
  Serial.println(F("  reset         - reset planner"));
  Serial.println(F("  sensor        - baca sensor IR"));
  Serial.println(F("  auto          - auto update dari sensor"));
  Serial.println(F("==============================\n"));
  
  printMap();
}

// ===== LOOP =====
void loop() {
  protocol();
  
  // Auto update dari sensor
  static unsigned long lastAuto = 0;
  if (millis() - lastAuto > 500) {
    lastAuto = millis();
    // Bisa tambah auto sensor update di sini
  }
  
  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "map") {
      printMap();
    }
    else if (cmd == "find") {
      if (hasGoal) {
        findPath();
        printPath();
      } else {
        Serial.println(F("Set goal dulu: goal x y"));
      }
    }
    else if (cmd.startsWith("goal ")) {
      int x, y;
      if (sscanf(cmd.c_str(), "goal %d %d", &x, &y) == 2) {
        setGoal(x, y);
        findPath();
        printPath();
      }
    }
    else if (cmd == "next") {
      if (pathIdx < pathLen) {
        Serial.print(F("Next WP: (")); Serial.print(pathX[pathIdx]); 
        Serial.print(F(",")); Serial.print(pathY[pathIdx]); Serial.println(F(")"));
        pathIdx++;
      } else {
        Serial.println(F("Path selesai / tidak ada path"));
      }
    }
    else if (cmd.startsWith("obs ")) {
      int x, y;
      if (sscanf(cmd.c_str(), "obs %d %d", &x, &y) == 2) {
        markObstacle(x, y, 1);
        Serial.print(F("Obstacle di (")); Serial.print(x); Serial.print(F(",")); Serial.print(y); Serial.println(F(")"));
      }
    }
    else if (cmd.startsWith("line ")) {
      int x, y;
      if (sscanf(cmd.c_str(), "line %d %d", &x, &y) == 2) {
        if (valid(x,y)) grid[x][y] = LINE;
        Serial.print(F("Line di (")); Serial.print(x); Serial.print(F(",")); Serial.print(y); Serial.println(F(")"));
      }
    }
    else if (cmd == "reset") {
      resetPlanner();
      Serial.println(F("Planner reset"));
    }
    else if (cmd == "sensor") {
      int L = readInfraredDistance(2);
      int R = readInfraredDistance(1);
      Serial.print(F("IR Left: ")); Serial.print(L);
      Serial.print(F(" Right: ")); Serial.println(R);
    }
    else if (cmd == "auto") {
      // Demo: robot bergerak ke goal secara simulasi
      if (hasGoal && pathLen > 0) {
        Serial.println(F("=== AUTO NAV SIMULASI ==="));
        for (int i = 0; i < pathLen; i++) {
          robotX = pathX[i];
          robotY = pathY[i];
          grid[robotX][robotY] = VIS;
          Serial.print(F("Move to (")); Serial.print(robotX); Serial.print(F(",")); Serial.print(robotY); Serial.println(F(")"));
          printMap();
          delay(500);
        }
        Serial.println(F("GOAL REACHED!"));
      }
    }
    else if (cmd == "help") {
      printHelp();
    }
    else {
      Serial.print(F("Unknown: ")); Serial.println(cmd);
    }
  }
  
  delay(10);
}

// ===== PATH PLANNER FUNCTIONS =====

void resetPlanner() {
  for (int x=0; x<GRID_W; x++) {
    for (int y=0; y<GRID_H; y++) {
      grid[x][y] = UNK;
      nodes[x][y] = {x, y, 0, 0, nullptr, false, false};
    }
  }
  robotX = GRID_W/2;
  robotY = GRID_H/2;
  grid[robotX][robotY] = STA;
  goalX = -1; goalY = -1;
  hasGoal = false;
  pathLen = 0; pathIdx = 0;
  openSize = 0;
}

bool valid(int x, int y) {
  return x>=0 && x<GRID_W && y>=0 && y<GRID_H;
}

bool blocked(int x, int y) {
  if (!valid(x,y)) return true;
  return grid[x][y] == OBS;
}

float heuristic(int x1, int y1, int x2, int y2) {
  int dx = abs(x1-x2), dy = abs(y1-y2);
  return 1.414 * min(dx,dy) + abs(dx-dy); // Octile
}

void openPush(Node* n) {
  if (openSize < GRID_W*GRID_H) {
    openSet[openSize++] = n;
    // Bubble up
    int i = openSize-1;
    while (i>0) {
      int p = (i-1)/2;
      if (openSet[i]->g + openSet[i]->h < openSet[p]->g + openSet[p]->h) {
        Node* t = openSet[i]; openSet[i] = openSet[p]; openSet[p] = t;
        i = p;
      } else break;
    }
  }
}

Node* openPop() {
  if (openSize==0) return nullptr;
  Node* min = openSet[0];
  openSet[0] = openSet[--openSize];
  // Heapify down
  int i=0;
  while (true) {
    int s=i, l=2*i+1, r=2*i+2;
    if (l<openSize && openSet[l]->g+openSet[l]->h < openSet[s]->g+openSet[s]->h) s=l;
    if (r<openSize && openSet[r]->g+openSet[r]->h < openSet[s]->g+openSet[s]->h) s=r;
    if (s!=i) { Node* t=openSet[i]; openSet[i]=openSet[s]; openSet[s]=t; i=s; }
    else break;
  }
  return min;
}

void setGoal(int x, int y) {
  goalX = constrain(x, 0, GRID_W-1);
  goalY = constrain(y, 0, GRID_H-1);
  hasGoal = true;
  pathIdx = 0;
  Serial.print(F("Goal set: (")); Serial.print(goalX); Serial.print(F(",")); Serial.print(goalY); Serial.println(F(")"));
}

void findPath() {
  // Clear node state
  for (int x=0; x<GRID_W; x++)
    for (int y=0; y<GRID_H; y++) {
      nodes[x][y].g = 0; nodes[x][y].h = 0;
      nodes[x][y].parent = nullptr;
      nodes[x][y].open = false; nodes[x][y].closed = false;
    }
  openSize = 0;
  
  Node* start = &nodes[robotX][robotY];
  start->g = 0; start->h = heuristic(robotX, robotY, goalX, goalY);
  start->open = true;
  openPush(start);
  
  Node* goalNode = &nodes[goalX][goalY];
  pathLen = 0;
  
  while (openSize > 0) {
    Node* cur = openPop();
    cur->open = false; cur->closed = true;
    
    if (cur == goalNode) {
      // Reconstruct
      Node* p = cur;
      while (p && pathLen < 64) {
        pathX[pathLen] = p->x;
        pathY[pathLen] = p->y;
        pathLen++;
        p = p->parent;
      }
      // Reverse
      for (int i=0; i<pathLen/2; i++) {
        swap(pathX[i], pathX[pathLen-1-i]);
        swap(pathY[i], pathY[pathLen-1-i]);
      }
      pathIdx = 0;
      return;
    }
    
    // 8 neighbors
    int dx[8] = {0,1,1,1,0,-1,-1,-1};
    int dy[8] = {-1,-1,0,1,1,1,0,-1};
    float cost[8] = {1,1.414,1,1.414,1,1.414,1,1.414};
    
    for (int i=0; i<8; i++) {
      int nx = cur->x + dx[i];
      int ny = cur->y + dy[i];
      if (!valid(nx,ny)) continue;
      if (blocked(nx,ny)) continue;
      if (nodes[nx][ny].closed) continue;
      
      float ng = cur->g + cost[i];
      // Obstacle penalty
      for (int ox=-1; ox<=1; ox++)
        for (int oy=-1; oy<=1; oy++)
          if (valid(nx+ox, ny+oy) && grid[nx+ox][ny+oy]==OBS)
            ng += 2.0;
      
      Node* nb = &nodes[nx][ny];
      if (!nb->open || ng < nb->g) {
        nb->g = ng;
        nb->h = heuristic(nx, ny, goalX, goalY);
        nb->parent = cur;
        if (!nb->open) {
          nb->open = true;
          openPush(nb);
        }
      }
    }
  }
  Serial.println(F("PATH NOT FOUND!"));
}

void markObstacle(int x, int y, int r) {
  for (int dx=-r; dx<=r; dx++)
    for (int dy=-r; dy<=r; dy++)
      if (valid(x+dx, y+dy) && dx*dx+dy*dy <= r*r)
        grid[x+dx][y+dy] = OBS;
}

void printMap() {
  Serial.println(F("\n+------------------+"));
  Serial.print(F("| Robot:(")); Serial.print(robotX); Serial.print(F(",")); Serial.print(robotY); Serial.println(F(")     |"));
  if (hasGoal) {
    Serial.print(F("| Goal: (")); Serial.print(goalX); Serial.print(F(",")); Serial.print(goalY); Serial.println(F(")     |"));
  }
  Serial.println(F("+------------------+"));
  Serial.println(F("Leg: .=UNK  =EMP  #=LINE  O=OBS  S=STA  G=GOL  @=ROB  +=PTH"));
  
  for (int y=0; y<GRID_H; y++) {
    if (y<10) Serial.print(F(" "));
    Serial.print(y); Serial.print(F(" "));
    for (int x=0; x<GRID_W; x++) {
      char c = '.';
      if (x==robotX && y==robotY) c = '@';
      else if (x==goalX && y==goalY) c = 'G';
      else {
        bool onPath = false;
        for (int i=0; i<pathLen; i++) if (pathX[i]==x && pathY[i]==y) { onPath=true; break; }
        if (onPath) c = '+';
        else switch(grid[x][y]) {
          case EMP: c=' '; break;
          case LINE: c='#'; break;
          case OBS: c='O'; break;
          case STA: c='S'; break;
          case VIS: c='*'; break;
          default: c='.'; break;
        }
      }
      Serial.print(c);
    }
    Serial.println();
  }
  Serial.println(F("  0123456789ABCDEF"));
  Serial.println(F("+------------------+\n"));
}

void printPath() {
  if (pathLen==0) { Serial.println(F("No path")); return; }
  Serial.print(F("Path: ")); Serial.print(pathLen); Serial.println(F(" waypoints"));
  for (int i=0; i<pathLen; i++) {
    Serial.print(F("  [")); Serial.print(i); Serial.print(F("] ("));
    Serial.print(pathX[i]); Serial.print(F(",")); Serial.print(pathY[i]); Serial.println(F(")"));
  }
}

void printHelp() {
  Serial.println(F("\n=== COMMANDS ==="));
  Serial.println(F("map           - show grid"));
  Serial.println(F("goal x y      - set goal"));
  Serial.println(F("find          - find path"));
  Serial.println(F("next          - next waypoint"));
  Serial.println(F("obs x y       - add obstacle"));
  Serial.println(F("line x y      - add line"));
  Serial.println(F("reset         - reset"));
  Serial.println(F("sensor        - read IR"));
  Serial.println(F("auto          - auto nav demo"));
  Serial.println(F("================\n"));
}