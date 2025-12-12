from pymavlink import mavutil
import time

class Hover:

    def __init__(self, serial):
        self.serial = serial
        self.master = mavutil.mavlink_connection(serial)

    def wait_heartbeat(self):
        print("Waiting for heartbeat…")
        self.master.wait_heartbeat()
        print("Heartbeat received from system", self.master.target_system)

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Mode set to:", mode_name)

    def arm(self):
        print("Arming…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,0,0,0,0,0,0
        )
        self.master.motors_armed_wait()
        print("Armed.")

    def takeoff(self, alt):
        print(f"Takeoff to {alt} m…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,0,0,0,
            0,0,alt
        )

    def hover(self, duration_sec=10):
        print(f"Hovering for {duration_sec} seconds…")
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        start = time.time()
        while time.time() - start < duration_sec:
            self.master.mav.set_position_target_local_ned_send(
                int(time.time()*1000),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0,0,0,
                0,0,0,
                0,0,0,
                0,0
            )
            time.sleep(0.2)

    def land(self):
        print("Landing…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,0,0,0,
            0,0,0
        )

    # ---------------- MAIN SCRIPT ---------------- #