import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle):
        self.max_throttle = rospy.get_param('~max_throttle', 0.8)
        self.full_stop_brake_keep = rospy.get_param('~full_stop_brake_keep', 1200)
        self.full_stop_brake_limit = rospy.get_param('~full_stop_brake_limit', 0.1)
        self.brake_deceleration_start = rospy.get_param('~brake_deceleration_start', -0.3)

        self.vehicle = vehicle
        self.yaw_controller = YawController(vehicle, 0.1)
        self.throttle_controller = PID(0.1, 0.05, 1.2, 0, self.max_throttle)
        self.velocity_filter = LowPassFilter(0.5, 0.02)
        self.last_time = rospy.get_time()

    def reset(self):
        self.throttle_controller.reset()
        self.last_time = rospy.get_time()
        
    def control(self, target_linear_velocity, target_angular_velocity, current_velocity):
        velocity = self.velocity_filter.filt(current_velocity)
        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, velocity)
        error = target_linear_velocity - velocity
        current_time = rospy.get_time()
        throttle = self.throttle_controller.step(error, current_time - self.last_time)
        self.last_time = current_time
        
        brake = 0
        if target_linear_velocity <= self.full_stop_brake_limit and velocity < 1: # target velocity is 0, and current velocity is small, set brake to 400 (1m/s^2)
            throttle = 0
            brake = self.full_stop_brake_keep
        elif error < self.brake_deceleration_start: # target velocity is lower than current velocity, apply brake
            throttle = 0
            decel = max(error, self.vehicle.decel_limit)
            brake = -decel * self.vehicle.mass * self.vehicle.wheel_radius
        rospy.loginfo("Control (%s, %s, %s) -> throttle: %s, steer: %s, brake: %s", target_linear_velocity, current_velocity, velocity, throttle, steering, brake)
        return throttle if brake <= 1 else 0, brake, steering

