// solver.c - Complete Maze Solver with DFS + A* + Optimal Path
#include "solver.h"
#include "API.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SIZE 16
#define INF 9999
#define MAX_STACK 256
#define MAX_QUEUE 512
#define MAX_HEAP 512
#define MAX_WALLS 1024

// Direction vectors: 0=N, 1=E, 2=S, 3=W
static const int dx[] = {0, 1, 0, -1};
static const int dy[] = {1, 0, -1, 0};

// Wall structure
typedef struct {
    int x, y, dir;
} Wall;

// Position structure
typedef struct {
    int x, y;
} Position;

// A* node
typedef struct {
    int f_score;
    Position pos;
} AStarNode;

// Global state
static int mouse_x = 0;
static int mouse_y = 0;
static int mouse_dir = 0;  // 0=N, 1=E, 2=S, 3=W
static int maze_width = 0;
static int maze_height = 0;

// Wall tracking
static Wall walls[MAX_WALLS];
static int wall_count = 0;

// Visited cells for DFS
static int visited[MAX_SIZE][MAX_SIZE];

// Distance map
static int distances[MAX_SIZE][MAX_SIZE];

// Goal cells
static Position goal_cells[4];

// DFS stack
static Position dfs_stack[MAX_STACK];
static int stack_top = -1;

// BFS queue
static Position bfs_queue[MAX_QUEUE];
static int queue_head = 0;
static int queue_tail = 0;

// A* heap
static AStarNode heap[MAX_HEAP];
static int heap_size = 0;

// Phase control
static int phase = 0;  // 0=explore, 1=return, 2=optimal, 3=done
static int exploration_done = 0;
static Position path_to_start[MAX_STACK];
static int path_index = 0;
static int path_length = 0;
static int optimal_run_started = 0;

// Helper functions
int manhattan_distance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

int is_goal(int x, int y) {
    for (int i = 0; i < 4; i++) {
        if (goal_cells[i].x == x && goal_cells[i].y == y)
            return 1;
    }
    return 0;
}

int has_wall(int x, int y, int dir) {
    for (int i = 0; i < wall_count; i++) {
        if (walls[i].x == x && walls[i].y == y && walls[i].dir == dir)
            return 1;
    }
    return 0;
}

void add_wall(int x, int y, int dir) {
    if (!has_wall(x, y, dir) && wall_count < MAX_WALLS) {
        walls[wall_count].x = x;
        walls[wall_count].y = y;
        walls[wall_count].dir = dir;
        wall_count++;
    }
}

void sense_walls() {
    if (API_wallFront())
        add_wall(mouse_x, mouse_y, mouse_dir);
    if (API_wallLeft())
        add_wall(mouse_x, mouse_y, (mouse_dir + 3) % 4);
    if (API_wallRight())
        add_wall(mouse_x, mouse_y, (mouse_dir + 1) % 4);
    
    // Check back wall (opposite direction)
    int back_dir = (mouse_dir + 2) % 4;
    int back_x = mouse_x + dx[back_dir];
    int back_y = mouse_y + dy[back_dir];
    if (back_x < 0 || back_x >= maze_width || back_y < 0 || back_y >= maze_height)
        add_wall(mouse_x, mouse_y, back_dir);
}

void turn_to_direction(int target_dir) {
    while (mouse_dir != target_dir) {
        int diff = (target_dir - mouse_dir + 4) % 4;
        if (diff == 1) {
            API_turnRight();
            mouse_dir = (mouse_dir + 1) % 4;
        } else if (diff == 3) {
            API_turnLeft();
            mouse_dir = (mouse_dir + 3) % 4;
        } else {
            API_turnRight();
            API_turnRight();
            mouse_dir = (mouse_dir + 2) % 4;
        }
    }
}

// Stack operations
void stack_push(Position p) {
    if (stack_top < MAX_STACK - 1)
        dfs_stack[++stack_top] = p;
}

Position stack_pop() {
    return dfs_stack[stack_top--];
}

Position stack_peek() {
    return dfs_stack[stack_top];
}

int stack_size() {
    return stack_top + 1;
}

// Queue operations
void queue_push(Position p) {
    if (queue_tail < MAX_QUEUE)
        bfs_queue[queue_tail++] = p;
}

Position queue_pop() {
    return bfs_queue[queue_head++];
}

int queue_empty() {
    return queue_head >= queue_tail;
}

// Heap operations
void heap_swap(int i, int j) {
    AStarNode temp = heap[i];
    heap[i] = heap[j];
    heap[j] = temp;
}

void heap_push(AStarNode node) {
    if (heap_size >= MAX_HEAP) return;
    
    int i = heap_size++;
    heap[i] = node;
    
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap[parent].f_score <= heap[i].f_score) break;
        heap_swap(parent, i);
        i = parent;
    }
}

AStarNode heap_pop() {
    AStarNode result = heap[0];
    heap[0] = heap[--heap_size];
    
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < heap_size && heap[left].f_score < heap[smallest].f_score)
            smallest = left;
        if (right < heap_size && heap[right].f_score < heap[smallest].f_score)
            smallest = right;
        
        if (smallest == i) break;
        
        heap_swap(i, smallest);
        i = smallest;
    }
    
    return result;
}

// Get unvisited neighbors
int get_unvisited_neighbors(Position* neighbors) {
    int count = 0;
    for (int d = 0; d < 4; d++) {
        int nx = mouse_x + dx[d];
        int ny = mouse_y + dy[d];
        if (nx >= 0 && nx < maze_width && ny >= 0 && ny < maze_height &&
            !visited[nx][ny] && !has_wall(mouse_x, mouse_y, d)) {
            neighbors[count].x = nx;
            neighbors[count].y = ny;
            count++;
        }
    }
    return count;
}

// Calculate distances using BFS
void calculate_distances() {
    debug_log("Calculating distances from goal...");
    
    // Initialize distances
    for (int i = 0; i < MAX_SIZE; i++)
        for (int j = 0; j < MAX_SIZE; j++)
            distances[i][j] = INF;
    
    // Reset queue
    queue_head = 0;
    queue_tail = 0;
    
    // Add all goal cells
    for (int i = 0; i < 4; i++) {
        distances[goal_cells[i].x][goal_cells[i].y] = 0;
        queue_push(goal_cells[i]);
    }
    
    // BFS
    while (!queue_empty()) {
        Position current = queue_pop();
        int current_dist = distances[current.x][current.y];
        
        for (int d = 0; d < 4; d++) {
            if (has_wall(current.x, current.y, d))
                continue;
            
            int nx = current.x + dx[d];
            int ny = current.y + dy[d];
            
            if (nx < 0 || nx >= maze_width || ny < 0 || ny >= maze_height)
                continue;
            
            if (distances[nx][ny] > current_dist + 1) {
                distances[nx][ny] = current_dist + 1;
                queue_push((Position){nx, ny});
            }
        }
    }
    
    // Display distances
    for (int y = 0; y < maze_height; y++) {
        for (int x = 0; x < maze_width; x++) {
            if (distances[x][y] < INF) {
                char text[10];
                sprintf(text, "%d", distances[x][y]);
                API_setText(x, y, text);
            }
        }
    }
}

// A* pathfinding to start
int find_path_to_start() {
    debug_log("Finding path to start with A*...");
    
    // Initialize
    Position came_from[MAX_SIZE][MAX_SIZE];
    int g_score[MAX_SIZE][MAX_SIZE];
    int in_came_from[MAX_SIZE][MAX_SIZE];
    
    for (int i = 0; i < MAX_SIZE; i++) {
        for (int j = 0; j < MAX_SIZE; j++) {
            g_score[i][j] = INF;
            in_came_from[i][j] = 0;
        }
    }
    
    heap_size = 0;
    g_score[mouse_x][mouse_y] = 0;
    heap_push((AStarNode){0, {mouse_x, mouse_y}});
    
    while (heap_size > 0) {
        AStarNode current_node = heap_pop();
        Position current = current_node.pos;
        
        if (current.x == 0 && current.y == 0) {
            // Reconstruct path
            path_length = 0;
            Position temp = current;
            
            while (in_came_from[temp.x][temp.y]) {
                path_to_start[path_length++] = temp;
                temp = came_from[temp.x][temp.y];
            }
            
            // Reverse path
            for (int i = 0; i < path_length / 2; i++) {
                Position tmp = path_to_start[i];
                path_to_start[i] = path_to_start[path_length - 1 - i];
                path_to_start[path_length - 1 - i] = tmp;
            }
            
            path_index = 0;
            char msg[100];
            sprintf(msg, "Path to start: %d steps", path_length);
            debug_log(msg);
            return 1;
        }
        
        for (int d = 0; d < 4; d++) {
            if (has_wall(current.x, current.y, d))
                continue;
            
            int nx = current.x + dx[d];
            int ny = current.y + dy[d];
            
            if (nx < 0 || nx >= maze_width || ny < 0 || ny >= maze_height)
                continue;
            
            int tentative_g = g_score[current.x][current.y] + 1;
            
            if (tentative_g < g_score[nx][ny]) {
                came_from[nx][ny] = current;
                in_came_from[nx][ny] = 1;
                g_score[nx][ny] = tentative_g;
                int f = tentative_g + manhattan_distance(nx, ny, 0, 0);
                heap_push((AStarNode){f, {nx, ny}});
            }
        }
    }
    
    debug_log("ERROR: No path to start!");
    return 0;
}

// Initialize
void init_solver() {
    static int initialized = 0;
    if (initialized) return;
    
    maze_width = API_mazeWidth();
    maze_height = API_mazeHeight();
    
    int center = maze_width / 2;
    goal_cells[0] = (Position){center - 1, center - 1};
    goal_cells[1] = (Position){center, center - 1};
    goal_cells[2] = (Position){center - 1, center};
    goal_cells[3] = (Position){center, center};
    
    memset(visited, 0, sizeof(visited));
    wall_count = 0;
    
    stack_push((Position){0, 0});
    
    char msg[100];
    sprintf(msg, "Maze: %dx%d", maze_width, maze_height);
    debug_log(msg);
    debug_log("=== Phase 1: Complete Maze Exploration ===");
    
    initialized = 1;
}

// Main solver function
Action solver() {
    return floodFill();
}

Action floodFill() {
    init_solver();
    
    // Phase 0: Exploration with DFS
    if (phase == 0) {
        API_setColor(mouse_x, mouse_y, 'Y');
        visited[mouse_x][mouse_y] = 1;
        sense_walls();
        
        if (is_goal(mouse_x, mouse_y) && !exploration_done) {
            debug_log("Goal found during exploration!");
            for (int i = 0; i < 4; i++)
                API_setColor(goal_cells[i].x, goal_cells[i].y, 'G');
            exploration_done = 1;
        }
        
        Position neighbors[4];
        int neighbor_count = get_unvisited_neighbors(neighbors);
        
        if (neighbor_count > 0) {
            int nx = neighbors[0].x;
            int ny = neighbors[0].y;
            
            for (int d = 0; d < 4; d++) {
                if (mouse_x + dx[d] == nx && mouse_y + dy[d] == ny) {
                    turn_to_direction(d);
                    break;
                }
            }
            
            API_moveForward();
            mouse_x = nx;
            mouse_y = ny;
            stack_push((Position){mouse_x, mouse_y});
            return IDLE;
        } else {
            if (stack_size() > 1) {
                stack_pop();
                Position prev = stack_peek();
                
                for (int d = 0; d < 4; d++) {
                    if (mouse_x + dx[d] == prev.x && mouse_y + dy[d] == prev.y) {
                        turn_to_direction(d);
                        API_moveForward();
                        mouse_x = prev.x;
                        mouse_y = prev.y;
                        return IDLE;
                    }
                }
            } else {
                debug_log("Exploration complete!");
                calculate_distances();
                
                if (find_path_to_start()) {
                    phase = 1;
                    debug_log("=== Phase 2: Returning to start ===");
                } else {
                    phase = 3;
                }
                return IDLE;
            }
        }
    }
    
    // Phase 1: Return to start
    if (phase == 1) {
        if (path_index < path_length) {
            int next_x = path_to_start[path_index].x;
            int next_y = path_to_start[path_index].y;
            
            for (int d = 0; d < 4; d++) {
                if (mouse_x + dx[d] == next_x && mouse_y + dy[d] == next_y) {
                    turn_to_direction(d);
                    break;
                }
            }
            
            API_moveForward();
            API_setColor(next_x, next_y, 'B');
            mouse_x = next_x;
            mouse_y = next_y;
            path_index++;
            return IDLE;
        } else {
            API_setColor(0, 0, 'G');
            debug_log("Returned to start!");
            phase = 2;
            debug_log("=== Phase 3: Optimal path execution ===");
            API_clearAllColor();
            API_clearAllText();
            
            for (int y = 0; y < maze_height; y++) {
                for (int x = 0; x < maze_width; x++) {
                    if (distances[x][y] < INF) {
                        char text[10];
                        sprintf(text, "%d", distances[x][y]);
                        API_setText(x, y, text);
                    }
                }
            }
            
            mouse_x = 0;
            mouse_y = 0;
            turn_to_direction(0);
            API_setColor(0, 0, 'C');
            optimal_run_started = 1;
            return IDLE;
        }
    }
    
    // Phase 2: Optimal path
    if (phase == 2) {
        if (is_goal(mouse_x, mouse_y)) {
            for (int i = 0; i < 4; i++)
                API_setColor(goal_cells[i].x, goal_cells[i].y, 'R');
            debug_log("=== Optimal path complete! ===");
            phase = 3;
            return IDLE;
        }
        
        int current_dist = distances[mouse_x][mouse_y];
        int best_dir = -1;
        int best_dist = current_dist;
        
        for (int d = 0; d < 4; d++) {
            if (has_wall(mouse_x, mouse_y, d))
                continue;
            
            int nx = mouse_x + dx[d];
            int ny = mouse_y + dy[d];
            
            if (nx < 0 || nx >= maze_width || ny < 0 || ny >= maze_height)
                continue;
            
            if (distances[nx][ny] < best_dist) {
                best_dist = distances[nx][ny];
                best_dir = d;
            }
        }
        
        if (best_dir != -1) {
            turn_to_direction(best_dir);
            API_moveForward();
            mouse_x += dx[mouse_dir];
            mouse_y += dy[mouse_dir];
            
            if (!is_goal(mouse_x, mouse_y))
                API_setColor(mouse_x, mouse_y, 'C');
            
            return IDLE;
        }
    }
    
    // Phase 3: Done
    return IDLE;
}

Action leftWallFollower() {
    return IDLE;
}

Action rightWallFollower() {
    return IDLE;
}