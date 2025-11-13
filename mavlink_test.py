import serial
from pymavlink import mavutil

vel = 1
accel = 0.5
turn_rate = 10
connection = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

def approach_target_rc_override(roll, pitch, throttle, yaw):
    connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    roll, pitch, throttle, yaw, 
    0, 0, 0, 0         
    )

def approach_target_set_point(x, y, z, yaw):
    connection.mav.set_position_target_local_ned_send(
    0, connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    x, y, z,             # x, y, z positions
    vel, vel, vel,             # vx, vy, vz
    accel, accel, accel,             # ax, ay, az
    yaw,                # yaw (radians)
    turn_rate                    # yaw rate
)
    
