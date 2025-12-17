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
class VisionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.yaw_angle = 0.0
        self.frame = None
        self.timestamp = 0.0

server_ready = threading.Event()

vision_state = VisionState()
stop_event = threading.Event()

main_directions = Directions(0, 0, 0, 3, 0, 0, 0, 0, None, None)
vision_thread = threading.Thread(
    target=vision_loop,
    args=(main_directions, vision_state, stop_event),
    daemon=True
)
vision_thread.start()
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

drone = Control(DRONE_PATH, BAUD, 0, 0, 0, 0, 0, autonomous)
drone.wait_for_heartbeat()
drone.wait_for_control_v2()
main_directions = Directions(0, 0, 0, 3, 0, 0, 0, 0, None, None) # imports all the stats starting at 0
autonomous = drone.autonomous
main_directions.start_cam()
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ

while drone.autonomous:
    loop_start = time.time()

    with vision_state.lock:
        yaw_angle = vision_state.yaw_angle

    yaw_pwm = int(max(1200, min(1800, 1500 + 0.2 * yaw_angle)))
    drone.yaw_override(yaw_pwm)

    # Optional logging (throttled)
    # push_log(f"yaw_pwm={yaw_pwm}")

    elapsed = time.time() - loop_start
    time.sleep(max(0, DT - elapsed))


def vision_loop(directions: Directions, state: VisionState, stop_event):
    directions.start_cam()

    while not stop_event.is_set():
        directions.get_directions()

        with state.lock:
            state.yaw_angle = directions.yaw_angle
            state.frame = directions.frame
            state.timestamp = time.time()