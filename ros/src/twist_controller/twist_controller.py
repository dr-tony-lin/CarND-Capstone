import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.yaw_controller = YawController(vehicle, 0.1)
        self.throttle_controller = PID(10.5795, 0, -18.4359, -12.123, 0.2)
        self.velocity_filter = LowPassFilter(0.5, 0.02)
        self.last_time = rospy.get_time()

    def reset(self):
        self.throttle_controller.reset()
        self.last_time = rospy.get_time()
        
    def control(self, linear_velocity, angular_velocity, current_velocity):
        current_velocity = self.velocity_filter.filt(current_velocity)
        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        error = linear_velocity - current_velocity
        current_time = rospy.get_time()
        throttle = self.throttle_controller.step(error, current_time - self.last_time)
        self.last_time = current_time
        
        brake = 0
        if linear_velocity == 0 and current_velocity < 0.1: # target velocity is 0, and current velocity is small, set brake to 400 (1m/s^2)
            throttle = 0
            brake = 400
        elif throttle < 0.1 and error < 0: # target velocity is lower than current velocity, apply brake
            throttle = 0
            decel = max(error, self.vehicle.decel_limit)
            brake = abs(decel) * self.vehicle.mass * self.vehicle.wheel_radius
        return throttle, brake, steering
    
    def compute_break(self, accel):
        return accel * self.vehicle.mass * self.vehicle.wheel_radius
