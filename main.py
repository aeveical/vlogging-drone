from drone_directions import directions
from hover_test import Hover
#from mavlink_test import Mavlink
## MAIN

TARGET_ALT = 1  # meters
autonomous = False
DRONE_PATH = ''
# ---- MAIN ---- #

DRONE_PATH = '/dev/ttyTHS1' 
# might also be /dev/ttyAMA0
BAUD = 921600
#probably idk tho

new_alt = 0
alt_acc = 0
pitch = 0
throttle = 0

#Assuming jetson connected to telem1: tx rx gnd
main_directions = directions(0, 0, 0, 3, 0, 0, 0, 0) # imports all the stats starting at 0
main_directions.start_cam()
main_directions.get_directions()
drone_hover = Hover(DRONE_PATH, BAUD, main_directions.yaw_angle, new_alt, alt_acc, pitch, throttle, autonomous)
drone_hover.wait_for_control()
while autonomous == True:
    main_directions = directions(0, 0, 0, 3, 0, 0, 0, 0) # imports all the stats starting at 0
#    main_directions.start_cam()
    main_directions.get_directions()
    drone_hover = Hover(DRONE_PATH, BAUD, main_directions.yaw_angle, new_alt, alt_acc, pitch, throttle, autonomous)
#    drone_hover.wait_for_control()
    if (main_directions.height_change > 0) or (not main_directions.boxA): # If the drone is below
        #the target or doesnt see one, hover up to 2 meters
        drone_hover.start()
    else:
        drone_hover.hover()
    
    drone_hover.set_yaw() # straight up yawing it
    dist_change = main_directions.distance - 2
#    drone = Mavlink(DRONE_PATH, 0, main_directions.yaw_angle, 0)
#    drone.approach_target_rc_override()
    if main_directions.distance < 1.5:
        drone_hover.set_mode("LOITER") # Goes back to loiter if it gets close to someone

#Mission planner commands:
#FLTMODE1 = STABILIZE -- RC controlled
#FLTMODE2 = LOITER -- just gonna chill there
#FLTMODE3 = GUIDED -- Jetson Control


#VISION_SPEED_ESTIMATE (103) LOOK INTO THIS