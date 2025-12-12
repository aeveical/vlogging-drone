from drone_directions import directions
from hover_test import hover
## MAIN

TARGET_ALT = 1  # meters
SERIAL_CONNECTION = 123


def start():
    # import hover code class
    run_hover = hover(SERIAL_CONNECTION, TARGET_ALT)

def main():
    main_directions = directions(0, 0, 0) #imports all the stats starting at 0