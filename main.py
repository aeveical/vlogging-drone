from directions import Directions
from control import Control
#from vision_state import VisionState
from server import start_server, push_frame, push_log
from flask import Flask
import threading
import time
import asyncio

app = Flask(__name__)
# Start web server
#threading.Thread(
#    target=start_server,
#    daemon=True
#).start()

server_ready = threading.Event()
shutdown_event = threading.Event()

server_thread = threading.Thread(target=start_server)
server_thread.start()

#server_ready.wait()

server_ready.set()

time.sleep(0.1)

TARGET_ALT = 1  # meters
autonomous = False
DRONE_PATH = ''
# ---- MAIN ---- #

DRONE_PATH = '/dev/ttyTHS1' 
# DRONE_PATH = '/dev/ttyAMA0'
BAUD = 57600
#probably idk tho

new_alt = 0
alt_acc = 0
pitch = 0
throttle = 0
cycle_count = 0

push_log(
    f"on"
    #f"Z={Z_m:.2f}m FPS={fps:.1f}\n"
    )

drone = Control(DRONE_PATH, BAUD, 0, 0, 0, 0, 0, autonomous)
drone.wait_for_heartbeat()
drone.wait_for_control_v2()
main_directions = Directions(0, 0, 0, 3, 0, 0, 0, 0, None, None) # imports all the stats starting at 0
main_directions.start_cam()
while True:
    autonomous = drone.autonomous
    if drone.autonomous == True:
        if cycle_count >=1000:
            drone.wait_for_rc_v2()
            push_log(
                f"checking for rc"
            )
    #    main_directions = directions(main_directions.yaw_angle, main_directions.height_change, main_directions., 3, 0, 0, 0, 0) # imports all the stats starting at 0
    #    main_directions.start_cam()
        main_directions.get_directions()
        push_frame(main_directions.frame)
        yaw_pwm = 1500 + 0.2*main_directions.yaw_angle
        dist_change = main_directions.distance - 3
        pitch = int(max(1200, min(1800,1500 + 20*dist_change)))
        push_log(f"pitch_pwm ={pitch:.1f}")
    #    drone_hover = Hover(DRONE_PATH, BAUD, main_directions.yaw_angle, new_alt, alt_acc, pitch, throttle, autonomous)
        drone.yaw_angle = main_directions.yaw_angle
    #    drone_hover.wait_for_control()
    #    if (main_directions.height_change > 0) or (not main_directions.boxA): # If the drone is below
    #        #the target or doesnt see one, hover up to 2 meters
    #        drone_hover.start()
    #    else:
    #        drone_hover.hover()
    #        print("hovering"
    #    yaw_pwm = 1500 + drone.yaw_angle 
        yaw_deg = drone.yaw_angle/20
        yaw_pwm = int(max(1200, min(1800, yaw_pwm)))
        drone.pitch_yaw_override(yaw_pwm, pitch) # CECK RC_MAP_YAW

        time.sleep(0.05)
    else:
        drone.un_yaw_override()
        drone.wait_for_control_v2()
        push_log(
            f"waiting for autonomous"
        )