from pymavlink import mavutil
import time

class Hover:

    def __init__(self, serial, baud, yaw_angle, new_alt, alt_acc, pitch, throttle, autonomous):
        self.serial = serial
        self.baud = baud
        self.yaw_angle = yaw_angle
        self.new_alt = new_alt
        self.alt_acc = alt_acc
        self.pitch = pitch
        self.throttle = throttle
        self.autonomous = autonomous
        self.master = mavutil.mavlink_connection(serial, baud)

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

    def start(self):
        self.wait_heartbeat()
        self.arm()
        self.set_mode("GUIDED")
        self.takeoff(5)
        self.hover(10)

    def set_yaw(self):
        message = self.master.mav.command_long_encode(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # ID of command to send
            0,  # Confirmation
            abs(self.yaw_angle), # Yaw angle
            0,       # yaw speed (set to default rn)
            1 if self.yaw_angle >= 0 else -1, #CW (1) or CCW (-1)
            1,       # choose relative yaw (0 = absolute)
            0,       # unused
            0,       # unused
            0        # unused
        )
        self.master.mav.send(message)
        print(message)
    
    def approach(self): ## THIS IS HELLA SKETCH
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, self.pitch, self.throttle, self.yaw_angle, 
            0, 0, 0, 0         
        )

    def change_alt(self):
        message  = self.master.mav.command_long_encode(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, # confirmation
            self.alt_acc,
            0,
            0,
            0,
            0,
            0,
            self.new_alt
        )
        self.master.mav.send(message)
        print(message)

    def wait_for_control(self):
        while self.autonomous == False:
            msg = self.master.recv_match(type = 'HEARTBEAT',blocking=True)
            print("Waiting for mode")

            if msg.get_type() == 'HEARTBEAT':
                mode = mavutil.mode_string_v10(msg)
                print(f"Flight mode: {mode}")
                if mode == "GUIDED_NOGPS":
                    self.autonomous = True
