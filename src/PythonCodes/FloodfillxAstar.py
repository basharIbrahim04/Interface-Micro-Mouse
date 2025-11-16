import API
import sys
from heapq import heappush, heappop
from collections import deque

# Directions: 0 = North, 1 = East, 2 = South, 3 = West
dx = [0, 1, 0, -1]
dy = [1, 0, -1, 0]

def log(s):
    sys.stderr.write(str(s) + "\n")
    sys.stderr.flush()

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

class MazeSolver:
    def __init__(self):
        self.maze_width = API.mazeWidth()
        self.maze_height = API.mazeHeight()
        self.x = 0
        self.y = 0
        self.direction = 0
        
        # Wall map
        self.walls = set()
        
        # Visited cells
        self.visited = set()
        
        # Goal cells (2x2 center)
        center = self.maze_width // 2
        self.goal_cells = [
            (center - 1, center - 1),
            (center, center - 1),
            (center - 1, center),
            (center, center)
        ]
        
        # Manhattan distance map
        self.distances = {}
        
        log(f"Maze: {self.maze_width}x{self.maze_height}")
        log(f"Goal cells: {self.goal_cells}")
    
    def sense_walls(self):
        """Sense and record walls at current position"""
        if API.wallFront():
            self.walls.add((self.x, self.y, self.direction))
        if API.wallLeft():
            self.walls.add((self.x, self.y, (self.direction - 1) % 4))
        if API.wallRight():
            self.walls.add((self.x, self.y, (self.direction + 1) % 4))
        if API.wallBack():
            self.walls.add((self.x, self.y, (self.direction + 2) % 4))
    
    def is_goal(self, x, y):
        """Check if current position is one of the goal cells"""
        return (x, y) in self.goal_cells
    
    def turn_to_direction(self, target_dir):
        """Turn to face target direction"""
        while self.direction != target_dir:
            diff = (target_dir - self.direction) % 4
            if diff == 1:
                API.turnRight()
                self.direction = (self.direction + 1) % 4
            elif diff == 3:
                API.turnLeft()
                self.direction = (self.direction - 1) % 4
            else:  # diff == 2
                API.turnRight()
                API.turnRight()
                self.direction = (self.direction + 2) % 4
    
    def get_unvisited_neighbors(self):
        """Get unvisited neighbors that are accessible"""
        neighbors = []
        for d in range(4):
            nx, ny = self.x + dx[d], self.y + dy[d]
            if (0 <= nx < self.maze_width and 0 <= ny < self.maze_height and
                (nx, ny) not in self.visited and (self.x, self.y, d) not in self.walls):
                neighbors.append((nx, ny, d))
        return neighbors
    
    def explore_all_cells(self):
        """Phase 1: Explore entire maze using DFS"""
        log("=== Phase 1: Complete Maze Exploration ===")
        
        stack = [(self.x, self.y)]
        goal_found = None
        cells_explored = 0
        
        while stack:
            API.setColor(self.x, self.y, "Y")
            self.visited.add((self.x, self.y))
            cells_explored += 1
            
            # Sense walls
            self.sense_walls()
            
            # Check if goal reached
            if self.is_goal(self.x, self.y) and goal_found is None:
                goal_found = (self.x, self.y)
                log(f"Goal detected at ({self.x}, {self.y})")
                for gx, gy in self.goal_cells:
                    API.setColor(gx, gy, "G")
            
            # Try to move to unvisited neighbor
            neighbors = self.get_unvisited_neighbors()
            
            if neighbors:
                # Pick first unvisited neighbor
                nx, ny, d = neighbors[0]
                
                # Turn and move
                self.turn_to_direction(d)
                API.moveForward()
                self.x, self.y = nx, ny
                stack.append((self.x, self.y))
            else:
                # Backtrack
                if len(stack) > 1:
                    stack.pop()
                    prev_x, prev_y = stack[-1]
                    
                    # Find direction to previous cell
                    for d in range(4):
                        if self.x + dx[d] == prev_x and self.y + dy[d] == prev_y:
                            self.turn_to_direction(d)
                            API.moveForward()
                            self.x, self.y = prev_x, prev_y
                            break
                else:
                    break
        
        log(f"Exploration complete: {cells_explored} cells visited")
        if goal_found:
            log(f"Goal found at {goal_found}")
        
        return goal_found
    
    def calculate_manhattan_distances(self):
        """Calculate Manhattan distances using flood fill from goal"""
        log("=== Calculating Manhattan distances from goal ===")
        
        # Initialize all distances to infinity
        for y in range(self.maze_height):
            for x in range(self.maze_width):
                self.distances[(x, y)] = 9999
        
        # BFS from all goal cells
        queue = deque()
        for gx, gy in self.goal_cells:
            self.distances[(gx, gy)] = 0
            queue.append((gx, gy))
        
        while queue:
            cx, cy = queue.popleft()
            current_dist = self.distances[(cx, cy)]
            
            # Check all 4 neighbors
            for d in range(4):
                # Check if there's a wall
                if (cx, cy, d) in self.walls:
                    continue
                
                nx, ny = cx + dx[d], cy + dy[d]
                
                # Check bounds
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                
                # Update distance if shorter path found
                if self.distances[(nx, ny)] > current_dist + 1:
                    self.distances[(nx, ny)] = current_dist + 1
                    queue.append((nx, ny))
        
        # Display distances on maze
        for y in range(self.maze_height):
            for x in range(self.maze_width):
                if self.distances[(x, y)] < 9999:
                    API.setText(x, y, str(self.distances[(x, y)]))
        
        log(f"Distance from start to goal: {self.distances[(0, 0)]}")
    
    def a_star_to_start(self):
        """Phase 2: Return to start using A*"""
        log("=== Phase 2: Returning to start with A* ===")
        
        start_pos = (0, 0)
        current_pos = (self.x, self.y)
        
        # A* algorithm
        open_set = []
        heappush(open_set, (0, current_pos))
        
        came_from = {}
        g_score = {current_pos: 0}
        f_score = {current_pos: manhattan_distance(self.x, self.y, 0, 0)}
        
        while open_set:
            _, current = heappop(open_set)
            
            if current == start_pos:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                
                log(f"Path to start found: {len(path)} steps")
                return path
            
            cx, cy = current
            for d in range(4):
                if (cx, cy, d) in self.walls:
                    continue
                
                nx, ny = cx + dx[d], cy + dy[d]
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                
                neighbor = (nx, ny)
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + manhattan_distance(nx, ny, 0, 0)
                    heappush(open_set, (f_score[neighbor], neighbor))
        
        log("No path to start found!")
        return None
    
    def follow_path(self, path, color="B"):
        """Follow a given path"""
        for next_x, next_y in path:
            # Find direction to next cell
            for d in range(4):
                if self.x + dx[d] == next_x and self.y + dy[d] == next_y:
                    self.turn_to_direction(d)
                    break
            
            API.moveForward()
            API.setColor(next_x, next_y, color)
            self.x, self.y = next_x, next_y
    
    def execute_optimal_path(self):
        """Phase 3: Execute optimal path from start to goal"""
        log("=== Phase 3: Executing optimal path to goal ===")
        
        # Clear colors for clean visualization
        API.clearAllColor()
        API.clearAllText()
        
        # Redisplay distances
        for y in range(self.maze_height):
            for x in range(self.maze_width):
                if self.distances[(x, y)] < 9999:
                    API.setText(x, y, str(self.distances[(x, y)]))
        
        # Start at (0,0) and ensure facing North
        self.x, self.y = 0, 0
        log(f"Current direction before reset: {self.direction}")
        self.turn_to_direction(0)  # Face North
        log(f"Reset to face North (direction: {self.direction})")
        steps = 0
        
        API.setColor(0, 0, "C")
        
        # Follow decreasing distances to goal
        while (self.x, self.y) not in self.goal_cells:
            current_dist = self.distances[(self.x, self.y)]
            
            # Find neighbor with minimum distance
            best_dir = None
            best_dist = current_dist
            
            for d in range(4):
                if (self.x, self.y, d) in self.walls:
                    continue
                
                nx, ny = self.x + dx[d], self.y + dy[d]
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                
                neighbor_dist = self.distances[(nx, ny)]
                if neighbor_dist < best_dist:
                    best_dist = neighbor_dist
                    best_dir = d
            
            if best_dir is None:
                log("ERROR: No path forward in optimal run!")
                break
            
            # Execute move
            self.turn_to_direction(best_dir)
            API.moveForward()
            steps += 1
            self.x += dx[self.direction]
            self.y += dy[self.direction]
            
            if (self.x, self.y) not in self.goal_cells:
                API.setColor(self.x, self.y, "C")
        
        # Mark goal
        for gx, gy in self.goal_cells:
            API.setColor(gx, gy, "R")
        
        API.setText(self.x, self.y, f"GOAL: {steps}")
        log(f"=== Optimal path completed in {steps} steps ===")
    
    def run(self):
        """Main execution: explore, return, execute optimal"""
        # Phase 1: Explore entire maze
        goal = self.explore_all_cells()
        
        if not goal:
            log("ERROR: Goal not found during exploration!")
            return
        
        # Calculate Manhattan distances from goal
        self.calculate_manhattan_distances()
        
        # Phase 2: Return to start
        path_to_start = self.a_star_to_start()
        if path_to_start is not None:
            if len(path_to_start) > 0:
                self.follow_path(path_to_start, "B")
                log("Returned to start successfully")
            else:
                log("Already at start position")
            API.setColor(0, 0, "G")
        else:
            log("ERROR: Could not find path to start!")
            return
        
        # Phase 3: Execute optimal path
        self.execute_optimal_path()
        
        log("=== COMPLETE: All phases finished successfully ===")

def main():
    solver = MazeSolver()
    solver.run()

if __name__ == "__main__":
    main()