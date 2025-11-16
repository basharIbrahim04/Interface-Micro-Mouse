import API
import sys

# Directions: 0 = North, 1 = East, 2 = South, 3 = West
dx = [0, 1, 0, -1]
dy = [1, 0, -1, 0]

def log(s):
    sys.stderr.write(str(s) + "\n")
    sys.stderr.flush()

def reachedGoal():
    # Must have open front and open right
    if API.wallFront():
        return False
    if API.wallRight():
        return False

    # Check adjacent right cell is also open to the front
    if API.wallFront(1) or API.wallRight(1) or API.wallFrontRight():
        return False

    return True

def main():
    log("Running...")

    x, y = 0, 0
    direction = 0  # facing north
    steps = 0  # step Counter

    while True:

        # Color path
        API.setColor(x, y, "B")

        # ---- GOAL DETECTED ----
        if reachedGoal():
            API.setColor(x, y, "G")
            API.setText(x, y, f"Goal! ({steps} steps)")
            log(f"Goal reached in {steps} steps")
            break

        # ---- LEFT-HAND RULE ----
        if not API.wallLeft():
            API.turnLeft()
            direction = (direction - 1) % 4

        while API.wallFront():
            API.turnRight()
            direction = (direction + 1) % 4

        API.moveForward()
        steps += 1  # count step

        # Update position
        x += dx[direction]
        y += dy[direction]

if __name__ == "__main__":
    main()
