from directions import Directions
from control import Control
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

print("Waiting for web server...")
#server_ready.wait()
print("Web server ready")

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
i = 0

push_log(
    f"on"
    #f"Z={Z_m:.2f}m FPS={fps:.1f}\n"
    )

print("Starting")
drone = Control(DRONE_PATH, BAUD, 0, 0, 0, 0, 0, autonomous)
print('hover initialized')
drone.wait_for_heartbeat()
print("Hearbeat Recieved")
drone.wait_for_control_v2()
print('waiting for control')
main_directions = Directions(0, 0, 0, 3, 0, 0, 0, 0, None, None) # imports all the stats starting at 0
print("initialized direction")
print("Autonomous", drone.autonomous)
autonomous = drone.autonomous
main_directions.start_cam()
while drone.autonomous == True:
    print('autonomous')
#    main_directions = directions(main_directions.yaw_angle, main_directions.height_change, main_directions., 3, 0, 0, 0, 0) # imports all the stats starting at 0
#    main_directions.start_cam()
    main_directions.get_directions()

    push_frame(main_directions.frame)
    print(main_directions.frame is None)
    yaw_pwm = 1500 + 20*main_directions.yaw_angle
# Send logs (human readable + control-relevant)
    
#    drone_hover = Hover(DRONE_PATH, BAUD, main_directions.yaw_angle, new_alt, alt_acc, pitch, throttle, autonomous)
    drone.yaw_angle = main_directions.yaw_angle
#    drone_hover.wait_for_control()
#    if (main_directions.height_change > 0) or (not main_directions.boxA): # If the drone is below
#        #the target or doesnt see one, hover up to 2 meters
#        drone_hover.start()
#    else:
#        drone_hover.hover()
#        print("hovering"
    yaw_pwm = 1500 + drone.yaw_angle 
    yaw_deg = drone.yaw_angle/20
    if i < 20:
        drone.yaw_override(yaw_pwm)
        push_log(
            f"yaw_override={yaw_pwm:.1f} "
        #f"Z={Z_m:.2f}m FPS={fps:.1f}\n"
        )
        i += 1
    elif i < 40:
        drone.un_yaw_override()
        drone.set_yaw(yaw_deg) # straight up yawing it
        push_log(
            f"yaw_set={yaw_deg:.1f} "
        #f"Z={Z_m:.2f}m FPS={fps:.1f}\n"
        )
        i += 1
    else:
        i = 0
#    if abs(drone.yaw_angle > 5):
#        drone.yaw_override()
#        print(drone.yaw_angle)
#        print("yawing it")
#    else:
    
    dist_change = main_directions.distance - 2
#    drone = Mavlink(DRONE_PATH, 0, main_directions.yaw_angle, 0)
#    drone.approach_target_rc_override()
#    if main_directions.distance < 1.5:
#        drone_hover.set_mode("STABILIZE") # Goes back to loiter if it gets close to someone
    asyncio.sleep(5)

#Mission planner commands:
#FLTMODE1 = STABILIZE -- RC controlled
#FLTMODE2 = LOITER -- just gonna chill there
#FLTMODE3 = GUIDED -- Jetson Control


#VISION_SPEED_ESTIMATE (103) LOOK INTO THIS