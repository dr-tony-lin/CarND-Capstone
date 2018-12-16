class Vehicle(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steering_ratio, max_lateral_accel, max_steering_angle):
        self.mass = vehicle_mass;
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steering_ratio = steering_ratio
        self.max_lateral_accel = max_lateral_accel
        self.max_steering_angle = max_steering_angle
