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

    # Starting state
    x, y = 0, 0
    direction = 0  # Facing North at start
    steps = 0      # step counter

    while True:

        # Mark current cell in blue
        API.setColor(x, y, "B")

        # ---- GOAL DETECTED ----
        if reachedGoal():
            API.setColor(x, y, "G")
            API.setText(x, y, f"Goal! ({steps} steps)")
            log(f"Goal reached in {steps} steps")
            break
        # ---- RIGHT HAND RULE ----
        if not API.wallRight():
            API.turnRight()
            direction = (direction + 1) % 4

        while API.wallFront():
            API.turnLeft()
            direction = (direction - 1) % 4

        # ---- MOVE FORWARD ----
        API.moveForward()
        steps += 1  # count step

        # Update coordinates
        x += dx[direction]
        y += dy[direction]

if __name__ == "__main__":
    main()
