#!/usr/bin/env python3
from pymavlink import mavutil
import time

class hover():

    def __init__(self)
        self.serial = seri


    def wait_heartbeat(master): # get a confirmation signal from drone
        print("Waiting for heartbeat…")
        master.wait_heartbeat()
        print("Heartbeat received from system", master.target_system)

    def set_mode(master, mode_name):
        mode_id = master.mode_mapping()[mode_name]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Mode set to:", mode_name)

    def arm(master): # ARM THAT HO
        print("Arming…")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        # Wait until armed
        while True:
            master.motors_armed_wait()
            if master.motors_armed():
                print("Armed.")
                return

    def takeoff(master, alt):
        print(f"Takeoff to {alt} m…")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, alt
        )

    def hover(master, duration_sec=10):
        """
        Hover by sending zero-velocity SET_POSITION_TARGET_LOCAL_NED at ~5 Hz.
        """
        print(f"Hovering for {duration_sec} seconds…")
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        start = time.time()
        while time.time() - start < duration_sec: #runs for a few seconds
            master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000),
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0, 0, 0,      # x, y, z positions ignored
                0, 0, 0,      # vx, vy, vz = 0 → hover
                0, 0, 0,      # ax, ay, az ignored
                0, 0          # yaw, yaw rate ignored
            )
            time.sleep(0.2)

    def land(master):
        print("Landing…")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )

    # ---------------- MAIN SCRIPT ---------------- #

    def hover_code():

        # Adjust your connection string:
        # SITL example: 'udp:127.0.0.1:14550'
        # Serial example: '/dev/ttyUSB0'
        connection = 'udp:127.0.0.1:14550' # change for our drone

        master = mavutil.mavlink_connection(connection)
        wait_heartbeat(master)

        # Switch to GUIDED mode so we can command takeoff + velocity setpoints
        set_mode(master, "GUIDED")
        arm(master)
        takeoff(master, TARGET_ALT)

        time.sleep(5)   # allow autopilot to reach altitude

        hover(master, duration_sec=10)

        land(master)
        print("Done.")
