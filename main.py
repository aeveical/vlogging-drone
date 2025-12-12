from drone_directions import directions
from hover_test import Hover
from mavlink_test import Mavlink
## MAIN

TARGET_ALT = 1  # meters
SERIAL_CONNECTION = 123
autonomous = False
DRONE_PATH = ''
# ---- MAIN ---- #
while autonomous == True:
    drone_hover = Hover(DRONE_PATH)
    drone_hover.start()
    main_directions = directions(0, 0, 0, 0, 0, 0, 0, 0) #imports all the stats starting at 0
    main_directions.start_cam()
    main_directions.get_directions()
    drone_autonomous = Mavlink(DRONE_PATH, 0, 0, 0)


#    drone.land()