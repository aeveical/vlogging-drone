class directions:
# Get the camera data and the directions we want the drone to go in
# Basically just put distance test code in here
    def __init__(self, yaw_angle, height_change, distance):
        self.yaw_angle = yaw_angle
        self.height_change = height_change
        self.distance = distance