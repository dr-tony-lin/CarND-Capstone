from math import atan

class YawController(object):
    def __init__(self, vehicle, min_speed):
        self.vehicle = vehicle
        self.min_speed = min_speed

    def get_angle(self, radius):
        angle = atan(self.vehicle.wheel_base / radius) * self.vehicle.steering_ratio
        return max(-self.vehicle.max_steering_angle, min(self.vehicle.max_steering_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.vehicle.max_lateral_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
