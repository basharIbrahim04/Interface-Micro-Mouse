// lefthand.c - Left-Hand Wall Following Algorithm
#include "solver.h"
#include "API.h"
#include <stdio.h>

// Global state variables
static int x = 0;
static int y = 0;
static Heading direction = NORTH;
static int steps = 0;
static int goalReached = 0;

// Direction vectors: dx[NORTH] = 0, dx[EAST] = 1, etc.
static const int dx[] = {0, 1, 0, -1};  // NORTH, EAST, SOUTH, WEST
static const int dy[] = {1, 0, -1, 0};

int isGoal() {
    int mazeWidth = API_mazeWidth();
    int mazeHeight = API_mazeHeight();
    
    int centerX = mazeWidth / 2;
    int centerY = mazeHeight / 2;
    
    // Check if in 2x2 center area
    if ((x == centerX - 1 || x == centerX) && 
        (y == centerY - 1 || y == centerY)) {
        return 1;
    }
    return 0;
}

Action solver() {
    return leftWallFollower();
}

Action leftWallFollower() {
    // Color current cell
    API_setColor(x, y, 'B');
    
    // Check if goal reached
    if (!goalReached && isGoal()) {
        goalReached = 1;
        API_setColor(x, y, 'G');
        
        char buffer[32];
        sprintf(buffer, "Goal! (%d steps)", steps);
        API_setText(x, y, buffer);
        
        char logMsg[64];
        sprintf(logMsg, "Goal reached in %d steps", steps);
        debug_log(logMsg);
        
        return IDLE;  // Stop at goal
    }
    
    // Left-hand wall following logic
    // Priority: Left > Forward > Right > Back
    
    if (!API_wallLeft()) {
        // Left is open - turn left and move forward
        API_turnLeft();
        direction = (direction + 3) % 4;  // -1 mod 4 = 3
        
        if (API_moveForward()) {
            steps++;
            x += dx[direction];
            y += dy[direction];
        }
        return IDLE;
    }
    else if (!API_wallFront()) {
        // Front is open - move forward
        if (API_moveForward()) {
            steps++;
            x += dx[direction];
            y += dy[direction];
        }
        return IDLE;
    }
    else if (!API_wallRight()) {
        // Right is open - turn right
        API_turnRight();
        direction = (direction + 1) % 4;
        return IDLE;
    }
    else {
        // All walls - turn around (turn right twice)
        API_turnRight();
        direction = (direction + 1) % 4;
        return IDLE;
    }
}
