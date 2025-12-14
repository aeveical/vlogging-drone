import serial
from pymavlink import mavutil

class Mavlink:

    def __init__(self, serial, yaw_angle, pitch, throttle):
        self.yaw_angle = yaw_angle
        self.pitch = pitch
        self.throttle = throttle
        self.master = mavutil.mavlink_connection(serial) #baud rate??

    def approach_target_rc_override(self):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, self.pitch, self.throttle, self.yaw_angle, 
            0, 0, 0, 0         
        )

    def set_yaw(self):