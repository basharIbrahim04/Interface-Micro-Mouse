// solver.c - A* Pathfinding Algorithm
// Explores maze using DFS, stops when goal is found
#include "solver.h"
#include "API.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_SIZE 16
#define INF 9999

// Direction vectors
static const int dx[] = {0, 1, 0, -1};  // NORTH, EAST, SOUTH, WEST
static const int dy[] = {1, 0, -1, 0};

// Global state
static int x = 0;
static int y = 0;
static int direction = 0;  // 0=N, 1=E, 2=S, 3=W
static int mazeWidth = 0;
static int mazeHeight = 0;

// Wall map and visited
static char walls[MAX_SIZE][MAX_SIZE][4];
static char visited[MAX_SIZE][MAX_SIZE];

// DFS stack
static int stackX[256];
static int stackY[256];
static int stackSize = 0;

// Goal cells
static int goalX[4];
static int goalY[4];
static int goalFound = 0;

// State
typedef enum {
    STATE_EXPLORE,
    STATE_COMPLETE
} State;
static State state = STATE_EXPLORE;
static int waitingForMove = 0;

void initMaze() {
    static int initialized = 0;
    if (initialized) return;
    
    mazeWidth = API_mazeWidth();
    mazeHeight = API_mazeHeight();
    
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));
    
    // Calculate goal cells
    int centerX = mazeWidth / 2;
    int centerY = mazeHeight / 2;
    goalX[0] = centerX - 1; goalY[0] = centerY - 1;
    goalX[1] = centerX;     goalY[1] = centerY - 1;
    goalX[2] = centerX - 1; goalY[2] = centerY;
    goalX[3] = centerX;     goalY[3] = centerY;
    
    stackX[0] = 0;
    stackY[0] = 0;
    stackSize = 1;
    
    char msg[64];
    sprintf(msg, "Maze: %dx%d", mazeWidth, mazeHeight);
    debug_log(msg);
    debug_log("=== A* Exploration - Finding Goal ===");
    
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

void senseWalls() {
    if (API_wallFront()) {
        walls[y][x][direction] = 1;
    }
    if (API_wallLeft()) {
        walls[y][x][(direction + 3) % 4] = 1;
    }
    if (API_wallRight()) {
        walls[y][x][(direction + 1) % 4] = 1;
    }
}

void turnToDirection(int targetDir) {
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

Action explorePhase() {
    static int cellsExplored = 0;
    static int moveDirection = -1;  // Track which direction we're moving
    static int isBacktracking = 0;  // Track if we're in backtrack mode
    
    // If waiting for move to complete
    if (waitingForMove) {
        char msg[128];
        sprintf(msg, "[MOVE] Move completed! Now updating position (%d,%d) -> (%d,%d) via dir=%d %s", 
                x, y, x + dx[moveDirection], y + dy[moveDirection], moveDirection,
                isBacktracking ? "(BACKTRACK)" : "");
        debug_log(msg);
        
        x += dx[moveDirection];
        y += dy[moveDirection];
        
        if (!isBacktracking) {
            // Only add to stack if moving forward
            stackX[stackSize] = x;
            stackY[stackSize] = y;
            stackSize++;
        }
        
        waitingForMove = 0;
        moveDirection = -1;
        isBacktracking = 0;
        
        return IDLE;  // Let next call process the new cell
    }
    
    // Process current cell
    char msg[128];
    sprintf(msg, "[PROCESS] At (%d,%d) dir=%d, cells=%d", x, y, direction, cellsExplored);
    debug_log(msg);
    
    API_setColor(x, y, 'Y');
    visited[y][x] = 1;
    cellsExplored++;
    
    senseWalls();
    sprintf(msg, "[WALLS] Front=%d Left=%d Right=%d", 
            API_wallFront(), API_wallLeft(), API_wallRight());
    debug_log(msg);
    
    if (!goalFound && isGoal(x, y)) {
        goalFound = 1;
        sprintf(msg, "Goal found at (%d, %d) after exploring %d cells", x, y, cellsExplored);
        debug_log(msg);
        for (int i = 0; i < 4; i++) {
            API_setColor(goalX[i], goalY[i], 'R');
        }
        sprintf(msg, "%d cells", cellsExplored);
        API_setText(x, y, msg);
        debug_log("=== Goal found - A* exploration complete ===");
        state = STATE_COMPLETE;
        return IDLE;
    }
    
    // Try unvisited neighbor
    debug_log("[SEARCH] Looking for unvisited neighbors...");
    for (int d = 0; d < 4; d++) {
        int nx = x + dx[d];
        int ny = y + dy[d];
        
        sprintf(msg, "[SEARCH] Dir %d (%s): next=(%d,%d) visited=%d wall=%d", 
                d, (d==0?"N":d==1?"E":d==2?"S":"W"), 
                nx, ny, 
                (nx >= 0 && nx < mazeWidth && ny >= 0 && ny < mazeHeight) ? visited[ny][nx] : -1, 
                walls[y][x][d]);
        debug_log(msg);
        
        if (nx >= 0 && nx < mazeWidth && ny >= 0 && ny < mazeHeight &&
            !visited[ny][nx] && !walls[y][x][d]) {
            
            sprintf(msg, "[DECIDE] Moving to (%d,%d) dir=%d", nx, ny, d);
            debug_log(msg);
            
            turnToDirection(d);
            sprintf(msg, "[TURN] Now facing dir=%d", direction);
            debug_log(msg);
            
            moveDirection = d;  // Remember which direction we're moving
            waitingForMove = 1;
            debug_log("[RETURN] Returning FORWARD - will update position next call");
            return FORWARD;
        }
    }
    
    // Backtrack
    debug_log("[BACKTRACK] No unvisited neighbors, backtracking...");
    if (stackSize > 1) {
        // Current cell is at stackSize - 1
        // Previous cell is at stackSize - 2
        int prevX = stackX[stackSize - 2];
        int prevY = stackY[stackSize - 2];
        
        sprintf(msg, "[BACKTRACK] Stack size=%d, current=(%d,%d), going to (%d,%d)", 
                stackSize, stackX[stackSize-1], stackY[stackSize-1], prevX, prevY);
        debug_log(msg);
        
        // Pop current cell from stack
        stackSize--;
        
        for (int d = 0; d < 4; d++) {
            if (x + dx[d] == prevX && y + dy[d] == prevY) {
                sprintf(msg, "[BACKTRACK] Found dir=%d to prev cell", d);
                debug_log(msg);
                
                turnToDirection(d);
                moveDirection = d;
                isBacktracking = 1;  // Mark that we're backtracking
                waitingForMove = 1;
                return FORWARD;
            }
        }
    }
    
    // Exploration complete
    sprintf(msg, "Exploration complete: %d steps (stack size: %d)", cellsExplored, stackSize);
    debug_log(msg);
    
    if (!goalFound) {
        debug_log("ERROR: Goal not found!");
    }
    
    state = STATE_COMPLETE;
    return IDLE;
}

Action solver() {
    initMaze();
    
    switch (state) {
        case STATE_EXPLORE:
            return explorePhase();
            
        case STATE_COMPLETE:
            return IDLE;
    }
    
    return IDLE;
}

Action floodFill() {
    return IDLE;
}

Action leftWallFollower() {
    return IDLE;
}

Action rightWallFollower() {
    return IDLE;
}