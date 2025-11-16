import API
import sys
from heapq import heappush, heappop

# Directions: 0 = North, 1 = East, 2 = South, 3 = West
dx = [0, 1, 0, -1]
dy = [1, 0, -1, 0]
dir_names = ['N', 'E', 'S', 'W']

def log(s):
    sys.stderr.write(str(s) + "\n")
    sys.stderr.flush()

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def get_neighbors(x, y, walls):
    neighbors = []
    for d in range(4):
        nx, ny = x + dx[d], y + dy[d]
        if (x, y, d) not in walls:
            neighbors.append((nx, ny, d))
    return neighbors

def a_star(start, goal, walls):
    open_set = []
    heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start[0], start[1], goal[0], goal[1])}
    
    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        
        for nx, ny, direction in get_neighbors(current[0], current[1], walls):
            neighbor = (nx, ny)
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance(nx, ny, goal[0], goal[1])
                heappush(open_set, (f_score[neighbor], neighbor))
    return None

def get_direction_to_turn(current_dir, target_dir):
    diff = (target_dir - current_dir) % 4
    if diff == 1:
        return 'R'
    elif diff == 3:
        return 'L'
    elif diff == 2:
        return 'RR'
    return None

# ------------------- Movement-pattern based goal detection -------------------
def goal_reached(actions):
    """Detect 2x2 goal using last 8 moves (F,L,F,L,F,L,F,L or F,R,...)."""
    if len(actions) < 8:
        return False
    last = actions[-8:]
    left_pattern  = ["L","F","L","F","L"]
    right_pattern = ["F","F","R","F","R","F"]
    return last == left_pattern or last == right_pattern
# ---------------------------------------------------------------------------

def explore_and_map(walls, visited):
    """Explore maze and build wall map using left-hand DFS"""
    x, y = 0, 0
    direction = 0
    stack = [(x, y)]
    actions = []  # record moves and turns for goal detection
    
    while stack:
        API.setColor(x, y, "Y")
        visited.add((x, y))
        
        # Sense walls
        if API.wallFront():
            walls.add((x, y, direction))
        if API.wallLeft():
            walls.add((x, y, (direction - 1) % 4))
        if API.wallRight():
            walls.add((x, y, (direction + 1) % 4))
        if API.wallBack():
            walls.add((x, y, (direction + 2) % 4))
        
        # ------------------ Check goal by movement pattern -------------------
        if goal_reached(actions):
            log(f"Goal detected through 2x2 movement pattern at ({x},{y})")
            return (x, y)
        # -------------------------------------------------------------------
        
        # Try to move to unvisited neighbor
        moved = False
        for d in range(4):
            nx, ny = x + dx[d], y + dy[d]
            if (nx, ny) not in visited and (x, y, d) not in walls:
                # Turn to face this direction
                turns = get_direction_to_turn(direction, d)
                if turns:
                    for turn in turns:
                        if turn == 'R':
                            API.turnRight()
                            actions.append("R")
                        elif turn == 'L':
                            API.turnLeft()
                            actions.append("L")
                        elif turn == 'RR':
                            API.turnRight()
                            API.turnRight()
                            actions.extend(["R","R"])
                direction = d
                
                API.moveForward()
                actions.append("F")
                x, y = nx, ny
                stack.append((x, y))
                moved = True
                break
        
        if not moved:
            # Backtrack
            if len(stack) > 1:
                stack.pop()
                prev_x, prev_y = stack[-1]
                # Navigate back
                for d in range(4):
                    if x + dx[d] == prev_x and y + dy[d] == prev_y:
                        turns = get_direction_to_turn(direction, d)
                        if turns:
                            for turn in turns:
                                if turn == 'R':
                                    API.turnRight()
                                    actions.append("R")
                                elif turn == 'L':
                                    API.turnLeft()
                                    actions.append("L")
                                elif turn == 'RR':
                                    API.turnRight()
                                    API.turnRight()
                                    actions.extend(["R","R"])
                            direction = d
                        API.moveForward()
                        actions.append("F")
                        x, y = prev_x, prev_y
                        break
            else:
                break
    
    return None

def follow_path(path, start_x, start_y, start_dir):
    """Follow the A* path"""
    x, y = start_x, start_y
    direction = start_dir
    
    for next_x, next_y in path:
        # Determine which direction to move
        for d in range(4):
            if x + dx[d] == next_x and y + dy[d] == next_y:
                turns = get_direction_to_turn(direction, d)
                if turns:
                    for turn in turns:
                        if turn == 'R':
                            API.turnRight()
                        elif turn == 'L':
                            API.turnLeft()
                direction = d
                break
        
        API.moveForward()
        API.setColor(next_x, next_y, "B")
        x, y = next_x, next_y
    
    return x, y, direction

def main():
    log("Running A* Algorithm...")
    
    walls = set()
    visited = set()
    
    # Explore and find goal
    log("Exploring maze...")
    goal = explore_and_map(walls, visited)
    
    if goal is None:
        log("Goal not found during exploration")
        return
    
    log(f"Goal found at {goal}")
    current_pos = goal
    current_dir = 0
    
    # Use A* to find path back to start
    log("Finding optimal path back to start with A*...")
    path = a_star(goal, (0, 0), walls)
    
    if path:
        log(f"Path found with {len(path)} steps")
        # Color the optimal path
        for px, py in path:
            API.setColor(px, py, "B")
        
        # Follow path back to start
        follow_path(path, current_pos[0], current_pos[1], current_dir)
        API.setColor(0, 0, "G")
        API.setText(0, 0, f"Done! {len(path)} steps")
    else:
        log("No path found back to start")

if __name__ == "__main__":
    main()
