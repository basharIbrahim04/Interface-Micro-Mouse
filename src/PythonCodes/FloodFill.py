#This Runs infinitely until finding the optimal path using only flood fill

import API
import sys
from collections import deque

def log(s):
    sys.stderr.write(f"{s}\n")
    sys.stderr.flush()

class FloodFillSolver:
    def __init__(self):
        # Maze size
        self.maze_width = API.mazeWidth()
        self.maze_height = API.mazeHeight()

        # State: position and facing
        self.x = 0
        self.y = 0
        self.direction = 0  # 0=N,1=E,2=S,3=W

        # 2x2 center goal cells (Micromouse standard)
        center_x = self.maze_width // 2
        center_y = self.maze_height // 2
        self.goal_cells = [(center_x + dx, center_y + dy) for dx in [0, -1] for dy in [0, -1]]
        log(f"Goal cells: {self.goal_cells}")

        # Distance map (large sentinel for unknown/unreachable)
        self.INF = 9999
        self.distance = [[self.INF for _ in range(self.maze_width)] for _ in range(self.maze_height)]

        # Wall map: (x,y) -> [N,E,S,W] booleans, False = unknown/no-wall yet (optimistic)
        self.walls = {}
        for yy in range(self.maze_height):
            for xx in range(self.maze_width):
                self.walls[(xx, yy)] = [False, False, False, False]

        # Boundary walls (outside maze)
        for yy in range(self.maze_height):
            self.walls[(0, yy)][3] = True
            self.walls[(self.maze_width - 1, yy)][1] = True
        for xx in range(self.maze_width):
            self.walls[(xx, 0)][2] = True
            self.walls[(xx, self.maze_height - 1)][0] = True

        # Movement vectors: consistent with previous working code
        # NOTE: using (dx,dy) where adding dy moves in y coordinate
        self.dx = [0, 1, 0, -1]
        self.dy = [1, 0, -1, 0]

        log(f"Micromouse Flood Fill - Maze: {self.maze_width}x{self.maze_height}")

    # ---------------------------
    # Utility / Movement helpers
    # ---------------------------
    def is_goal(self, x, y):
        return (x, y) in self.goal_cells

    def add_wall(self, x, y, direction):
        """Mark a wall and mirror it in the neighbor cell."""
        if not (0 <= x < self.maze_width and 0 <= y < self.maze_height):
            return
        self.walls[(x, y)][direction] = True
        nx = x + self.dx[direction]
        ny = y + self.dy[direction]
        if 0 <= nx < self.maze_width and 0 <= ny < self.maze_height:
            opposite = (direction + 2) % 4
            self.walls[(nx, ny)][opposite] = True

    def scan_walls(self):
        """Use API sensors and update wall map for the current cell."""
        # Front
        if API.wallFront():
            self.add_wall(self.x, self.y, self.direction)
        # Right
        if API.wallRight():
            self.add_wall(self.x, self.y, (self.direction + 1) % 4)
        # Left
        if API.wallLeft():
            self.add_wall(self.x, self.y, (self.direction - 1) % 4)

    def turn_to(self, target_dir):
        """Turn in-place to face target_dir (0..3)."""
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

    def move_forward(self):
        """Move forward by one cell and update x,y."""
        API.moveForward()
        self.x += self.dx[self.direction]
        self.y += self.dy[self.direction]

    # ---------------------------
    # Flood fill (general)
    # ---------------------------
    def flood_fill_from_goals(self, goals):
        """
        Generic flood-fill BFS that sets distance[][] to min distance from any cell in `goals`.
        `goals` is a list of (gx, gy) tuples. Uses current self.walls map.
        """
        # Reset distances
        for yy in range(self.maze_height):
            for xx in range(self.maze_width):
                self.distance[yy][xx] = self.INF

        q = deque()

        # Initialize goal cells with 0
        for gx, gy in goals:
            if 0 <= gx < self.maze_width and 0 <= gy < self.maze_height:
                self.distance[gy][gx] = 0
                q.append((gx, gy))

        # BFS
        while q:
            cx, cy = q.popleft()
            curd = self.distance[cy][cx]
            for d in range(4):
                # blocked by wall at current cell in direction d?
                if self.walls[(cx, cy)][d]:
                    continue
                nx = cx + self.dx[d]
                ny = cy + self.dy[d]
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                if self.distance[ny][nx] > curd + 1:
                    self.distance[ny][nx] = curd + 1
                    q.append((nx, ny))

        # Optionally display numbers (nice for debug)
        for yy in range(self.maze_height):
            for xx in range(self.maze_width):
                if self.distance[yy][xx] < self.INF:
                    API.setText(xx, yy, str(self.distance[yy][xx]))

    # ---------------------------
    # Exploration run -> reach center
    # ---------------------------
    def explore_to_goal(self):
        """Explore with flood-fill guidance until one of the center goal cells is reached."""
        log("=== Exploration run (map discovery) ===")

        # First fill (optimistic) from center goals
        self.flood_fill_from_goals(self.goal_cells)

        steps = 0
        max_steps = 5000

        # Mark goals visually
        for gx, gy in self.goal_cells:
            API.setColor(gx, gy, 'R')

        while not self.is_goal(self.x, self.y):
            steps += 1
            if steps > max_steps:
                log("Exploration: max steps reached, aborting.")
                break

            # mark current
            API.setColor(self.x, self.y, 'B')

            # scan local walls and update map
            self.scan_walls()

            # re-run flood fill if needed (inconsistency)
            cur_dist = self.distance[self.y][self.x]
            accessible = []
            for d in range(4):
                if not self.walls[(self.x, self.y)][d]:
                    nx = self.x + self.dx[d]
                    ny = self.y + self.dy[d]
                    if 0 <= nx < self.maze_width and 0 <= ny < self.maze_height:
                        accessible.append(self.distance[ny][nx])
            if accessible and cur_dist != min(accessible) + 1:
                log("Exploration: inconsistency detected -> reflooding")
                self.flood_fill_from_goals(self.goal_cells)

            # choose neighbor with minimal distance
            best_dir = None
            best_dist = self.INF
            for d in range(4):
                if self.walls[(self.x, self.y)][d]:
                    continue
                nx = self.x + self.dx[d]
                ny = self.y + self.dy[d]
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                if self.distance[ny][nx] < best_dist:
                    best_dist = self.distance[ny][nx]
                    best_dir = d

            if best_dir is None:
                log(f"Exploration: No possible move from ({self.x},{self.y}) — blocked.")
                break

            # execute move
            self.turn_to(best_dir)
            self.move_forward()
            log(f"Exploration step {steps}: moved to ({self.x},{self.y}), dist={self.distance[self.y][self.x]}")

        if self.is_goal(self.x, self.y):
            API.setColor(self.x, self.y, 'G')
            log(f"=== Goal reached in exploration in {steps} steps ===")
        else:
            log("Exploration ended without reaching a center cell.")

    # ---------------------------
    # Return to start via flood-fill shortest path (A = flood-fill style)
    # ---------------------------
    def return_to_start_via_floodfill(self):
        """
        Computes distances from the start (0,0) and follows decreasing distance values
        until reaching start. Scans for walls before each step and refloods if needed.
        """
        log("=== Computing shortest path back to start using flood-fill ===")
        start = (0, 0)

        # First compute distances from start (start as goal)
        self.flood_fill_from_goals([start])

        steps = 0
        max_steps = 5000

        while (self.x, self.y) != start and steps < max_steps:
            steps += 1

            # scan walls at current position — if discovering new walls, recompute distances
            # Save snapshot of current walls around cell to detect changes
            snapshot = tuple(self.walls[(self.x, self.y)])

            self.scan_walls()
            if tuple(self.walls[(self.x, self.y)]) != snapshot:
                log("Return: discovered new wall -> reflood from start")
                self.flood_fill_from_goals([start])

            cur_dist = self.distance[self.y][self.x]

            # pick neighbor whose distance is cur_dist - 1 (move toward start)
            next_dir = None
            for d in range(4):
                if self.walls[(self.x, self.y)][d]:
                    continue
                nx = self.x + self.dx[d]
                ny = self.y + self.dy[d]
                if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                    continue
                if self.distance[ny][nx] == cur_dist - 1:
                    next_dir = d
                    break

            # If no neighbor matches cur_dist-1, this means the current flood map is invalid.
            # Recompute and try again.
            if next_dir is None:
                log("Return: no neighbor with dist = cur_dist - 1, reflooding and retrying.")
                self.flood_fill_from_goals([start])
                # Attempt again to select direction
                for d in range(4):
                    if self.walls[(self.x, self.y)][d]:
                        continue
                    nx = self.x + self.dx[d]
                    ny = self.y + self.dy[d]
                    if not (0 <= nx < self.maze_width and 0 <= ny < self.maze_height):
                        continue
                    if self.distance[ny][nx] == cur_dist - 1:
                        next_dir = d
                        break

            if next_dir is None:
                log(f"Return: still no valid move from ({self.x},{self.y}), aborting return.")
                break

            # execute move
            self.turn_to(next_dir)
            self.move_forward()
            log(f"Return step {steps}: moved to ({self.x},{self.y}), dist_to_start={self.distance[self.y][self.x]}")

        if (self.x, self.y) == start:
            API.setColor(self.x, self.y, 'C')  # mark arrival to start
            log(f"=== Returned to start in {steps} steps ===")
        else:
            log("Return failed or aborted (max steps or inconsistency).")

    # ---------------------------
    # Top-level runner: exploration then return
    # ---------------------------
    def run_once_explore_then_return(self):
        """Run one full cycle: explore to center, then return to start via flood-fill shortest path."""
        while True:
            self.explore_to_goal()

            # After exploration we assume walls discovered so far are used to compute return path.
            # Run return using flood-fill (with start as goal)
            self.return_to_start_via_floodfill()


# ---------------------------
# Entry point
# ---------------------------
def main():
    solver = FloodFillSolver()
    solver.run_once_explore_then_return()

if __name__ == "__main__":
    main()
