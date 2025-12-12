from drone_directions import directions
from hover_test import Hover
## MAIN

TARGET_ALT = 1  # meters
SERIAL_CONNECTION = 123


def start():
    # import hover code class
    
    drone = Hover("udp:127.0.0.1:14550")
    drone.wait_heartbeat()
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(5)
    drone.hover(10)
#    drone.land()

def main():
    main_directions = directions(0, 0, 0) #imports all the stats starting at 0