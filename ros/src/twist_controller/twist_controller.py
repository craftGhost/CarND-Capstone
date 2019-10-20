from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_base, wheel_radius, steer_ratio, min_speed,
                max_lat_accel, max_steer_angle, accel_limit, decel_limit, rate):
        # TODO: Implement
        
        self.vehicle_mass = vehicle_mass
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.ts = 1.0/rate
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        #self.last_time = rospy.get_time()
        
        # kp, ki, kd, min_throttle, max_throttle
        self.throttle_PID = PID(0.3, 0.1, 0.1, 0.0, 0.3)
        
        # tau, sample_time
        self.vel_lpf = LowPassFilter(0.1, self.ts)
        self.error_lpf = LowPassFilter(0.3, self.ts)
        
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_PID.reset()
            return 0.0, 0.0, 0.0
        
        filtered_vel = self.vel_lpf.filt(current_vel)
        linear_vel_err = linear_vel - current_vel
        filtered_err = self.error_lpf.filt(linear_vel_err)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, filtered_vel)
        #vel_error = linear_vel - current_vel
        #self.last_vel = current_vel
        #current_time = rospy.get_time()
        #sample_time = current_time - self.last_time
        #self.last_time = current_time
        throttle = self.throttle_PID.step(filtered_err, self.ts)
        brake = 0
        
        if linear_vel == 0 and filtered_err < 0.1:
            throttle = 0
            brake = 400
        elif filtered_err < -0.1:
            throttle = 0
            decel = max(linear_vel_err, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        
        return throttle, brake, steering
