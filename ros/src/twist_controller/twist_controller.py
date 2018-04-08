import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
		
		#retrieve parameters
        self.vehicle_mass   = kwargs['vehicle_mass']
        self.fuel_capacity  = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit    = kwargs['decel_limit']
        self.accel_limit    = kwargs['accel_limit']
        self.wheel_radius   = kwargs['wheel_radius']
        self.wheel_base     = kwargs['wheel_base']		
        self.steer_ratio    = kwargs['steer_ratio']
        self.max_lat_accel  = kwargs['max_lat_accel']
        self.max_steer_angle= kwargs['max_steer_angle']	
        self.last_time      = rospy.get_time()

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)

        kp = 0.8
        ki = 0.0
        kd = 0.06
        throttle_min = 0
        throttle_max = 0.5*self.accel_limit
        self.throttle_controller = PID(kp, ki, kd,throttle_min,throttle_max)

        kp_cte = 0.5
        ki_cte = 0.0
        kd_cte = 0.2
        self.cte_controller = PID(kp_cte, ki_cte, kd_cte,-self.max_steer_angle,self.max_steer_angle)		
			
        tau = 0.5 #cutoff freq
        ts  = 0.02 #sample time
        self.vel_lowpass = LowPassFilter(tau,ts)
		
        pass

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel,cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
		
        if not dbw_enabled:
            #reset controller so that I term does not accumulate
            self.throttle_controller.reset()
            self.cte_controller.reset()
            return 0.0,0.0,0.0
        current_vel = self.vel_lowpass.filt(current_vel)		
 	
        vel_error = linear_vel - current_vel

        #calculate time elapsed from the previous time step
        current_time = rospy.get_time()        
        sample_time  = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        #calculate the additional steering control due to CTE Error and add it to the base 
        steering_base = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering_cte  = self.cte_controller.step(cte, sample_time)	
        steering_total= steering_base + steering_cte
        steering = max(min(self.max_steer_angle,steering_total),-self.max_steer_angle)
		
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 450 # Nm - for car to be held stationary
		
        elif throttle < 0.1 and vel_error < 0:	
            throttle = 0          
            #ensure deceleration does not exceed decel_limit			
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*(self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)*self.wheel_radius #Torque
			
        return throttle, brake , steering
