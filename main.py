from drone_directions import directions
from hover_test import Hover
## MAIN

TARGET_ALT = 1  # meters
SERIAL_CONNECTION = 123
autonomous = False


def start():
    # import hover code class
    
    drone = Hover("udp:127.0.0.1:14550")
    drone.wait_heartbeat()
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(5)
    drone.hover(10)
#    drone.land()

# ---- MAIN ---- #
while autonomous == True:
    start()
    main_directions = directions(0, 0, 0, 0, 0, 0, 0, 0) #imports all the stats starting at 0
    main_directions.start_cam()
    main_directions.get_directions()

#    drone.land()