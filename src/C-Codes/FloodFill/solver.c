// floodfill.c - Classic Micromouse Flood Fill Algorithm
#include "solver.h"
#include "API.h"
#include <stdio.h>
#include <string.h>

#define MAX_SIZE 16
#define INF 9999
#define MAX_QUEUE 256

// Global state
static int x = 0;
static int y = 0;
static int direction = 0;  // 0=N, 1=E, 2=S, 3=W
static int mazeWidth = 0;
static int mazeHeight = 0;

// Distance map
static int distance[MAX_SIZE][MAX_SIZE];

// Wall map: walls[y][x][dir] - 1 if wall exists
static char walls[MAX_SIZE][MAX_SIZE][4];

// Goal cells
static int goalX[4];
static int goalY[4];

// Direction vectors
static const int dx[] = {0, 1, 0, -1};
static const int dy[] = {1, 0, -1, 0};

// BFS Queue for flood fill
typedef struct {
    int x;
    int y;
} QueueNode;

static QueueNode queue[MAX_QUEUE];
static int queueHead = 0;
static int queueTail = 0;

void initMaze() {
    static int initialized = 0;
    if (initialized) return;
    
    mazeWidth = API_mazeWidth();
    mazeHeight = API_mazeHeight();
    
    // Initialize walls (optimistic - no walls initially)
    memset(walls, 0, sizeof(walls));
    
    // Add boundary walls
    for (int y = 0; y < mazeHeight; y++) {
        walls[y][0][3] = 1;  // West wall
        walls[y][mazeWidth - 1][1] = 1;  // East wall
    }
    for (int x = 0; x < mazeWidth; x++) {
        walls[0][x][2] = 1;  // South wall
        walls[mazeHeight - 1][x][0] = 1;  // North wall
    }
    
    // Calculate goal cells (2x2 center)
    int centerX = mazeWidth / 2;
    int centerY = mazeHeight / 2;
    goalX[0] = centerX - 1; goalY[0] = centerY - 1;
    goalX[1] = centerX;     goalY[1] = centerY - 1;
    goalX[2] = centerX - 1; goalY[2] = centerY;
    goalX[3] = centerX;     goalY[3] = centerY;
    
    char msg[64];
    sprintf(msg, "Maze: %dx%d, Goals: (%d,%d) (%d,%d) (%d,%d) (%d,%d)", 
            mazeWidth, mazeHeight,
            goalX[0], goalY[0], goalX[1], goalY[1],
            goalX[2], goalY[2], goalX[3], goalY[3]);
    debug_log(msg);
    
    // Mark goal cells in red
    for (int i = 0; i < 4; i++) {
        API_setColor(goalX[i], goalY[i], 'R');
    }
    
    initialized = 1;
}

int isGoal(int px, int py) {
    for (int i = 0; i < 4; i++) {
        if (px == goalX[i] && py == goalY[i]) {
            return 1;
        }
    }
    return 0;
}

void addWall(int px, int py, int dir) {
    walls[py][px][dir] = 1;
    
    // Add mirror wall
    int nx = px + dx[dir];
    int ny = py + dy[dir];
    if (nx >= 0 && nx < mazeWidth && ny >= 0 && ny < mazeHeight) {
        int oppositeDir = (dir + 2) % 4;
        walls[ny][nx][oppositeDir] = 1;
    }
}

void scanWalls() {
    if (API_wallFront()) {
        addWall(x, y, direction);
    }
    if (API_wallRight()) {
        addWall(x, y, (direction + 1) % 4);
    }
    if (API_wallLeft()) {
        addWall(x, y, (direction + 3) % 4);
    }
}

void floodFillDistances() {
    // Initialize all distances
    for (int i = 0; i < mazeHeight; i++) {
        for (int j = 0; j < mazeWidth; j++) {
            distance[i][j] = INF;
        }
    }
    
    // BFS from all goal cells
    queueHead = 0;
    queueTail = 0;
    
    for (int i = 0; i < 4; i++) {
        distance[goalY[i]][goalX[i]] = 0;
        queue[queueTail].x = goalX[i];
        queue[queueTail].y = goalY[i];
        queueTail++;
    }
    
    // BFS
    while (queueHead < queueTail) {
        int cx = queue[queueHead].x;
        int cy = queue[queueHead].y;
        queueHead++;
        
        int currentDist = distance[cy][cx];
        
        // Check all 4 neighbors
        for (int d = 0; d < 4; d++) {
            if (walls[cy][cx][d]) continue;
            
            int nx = cx + dx[d];
            int ny = cy + dy[d];
            
            if (nx < 0 || nx >= mazeWidth || ny < 0 || ny >= mazeHeight) continue;
            
            int newDist = currentDist + 1;
            if (newDist < distance[ny][nx]) {
                distance[ny][nx] = newDist;
                queue[queueTail].x = nx;
                queue[queueTail].y = ny;
                queueTail++;
            }
        }
    }
    
    // Display distances
    for (int j = 0; j < mazeHeight; j++) {
        for (int i = 0; i < mazeWidth; i++) {
            if (distance[j][i] < INF) {
                char text[8];
                sprintf(text, "%d", distance[j][i]);
                API_setText(i, j, text);
            }
        }
    }
}

int getBestDirection() {
    int minDist = INF;
    int bestDir = -1;
    
    for (int d = 0; d < 4; d++) {
        if (walls[y][x][d]) continue;
        
        int nx = x + dx[d];
        int ny = y + dy[d];
        
        if (nx < 0 || nx >= mazeWidth || ny < 0 || ny >= mazeHeight) continue;
        
        if (distance[ny][nx] < minDist) {
            minDist = distance[ny][nx];
            bestDir = d;
        }
    }
    
    return bestDir;
}

void turnTo(int targetDir) {
    while (direction != targetDir) {
        int diff = (targetDir - direction + 4) % 4;
        if (diff == 1) {
            API_turnRight();
            direction = (direction + 1) % 4;
        } else if (diff == 3) {
            API_turnLeft();
            direction = (direction + 3) % 4;
        } else {
            API_turnRight();
            API_turnRight();
            direction = (direction + 2) % 4;
        }
    }
}

Action solver() {
    return floodFill();
}

Action floodFill() {
    static int initialized = 0;
    static int steps = 0;
    static int goalReached = 0;
    
    if (!initialized) {
        initMaze();
        floodFillDistances();  // Initial optimistic flood fill
        debug_log("Starting Flood Fill Algorithm");
        initialized = 1;
    }
    
    if (goalReached) {
        return IDLE;
    }
    
    // Check if goal reached
    if (isGoal(x, y)) {
        API_setColor(x, y, 'G');
        char msg[64];
        sprintf(msg, "GOAL REACHED in %d steps!", steps);
        debug_log(msg);
        goalReached = 1;
        return IDLE;
    }
    
    // Mark path
    API_setColor(x, y, 'B');
    
    // Scan walls
    scanWalls();
    
    // Check if reflood is needed (inconsistency detection)
    int currentDist = distance[y][x];
    int minNeighborDist = INF;
    
    for (int d = 0; d < 4; d++) {
        if (walls[y][x][d]) continue;
        
        int nx = x + dx[d];
        int ny = y + dy[d];
        
        if (nx >= 0 && nx < mazeWidth && ny >= 0 && ny < mazeHeight) {
            if (distance[ny][nx] < minNeighborDist) {
                minNeighborDist = distance[ny][nx];
            }
        }
    }
    
    // If inconsistent, reflood
    if (minNeighborDist != INF && currentDist != minNeighborDist + 1) {
        debug_log("Inconsistency detected - reflooding");
        floodFillDistances();
    }
    
    // Get best direction
    int bestDir = getBestDirection();
    
    if (bestDir == -1) {
        debug_log("ERROR: No path available!");
        return IDLE;
    }
    
    // Log move
    char msg[80];
    sprintf(msg, "Step %d: (%d,%d) dist=%d -> %c", 
            steps, x, y, distance[y][x], "NESW"[bestDir]);
    debug_log(msg);
    
    // Execute move
    turnTo(bestDir);
    if (API_moveForward()) {
        steps++;
        x += dx[direction];
        y += dy[direction];
    }
    
    return IDLE;
}

Action leftWallFollower() {
    return IDLE;
}

Action rightWallFollower() {
    return IDLE;
}