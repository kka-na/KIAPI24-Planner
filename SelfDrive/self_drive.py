import numpy as np
from config.config import Config
from localization.path_manager import PathManager
from control.actuator import Actuator
from control.pid import PID
from control.pure_pursuit import PurePursuit
from localization.point import Point
from planning.adaptive_cruise_control import AdaptiveCruiseControl

class SelfDrive:
    def __init__(self):
        config = Config()
        self.pm = PathManager(config['planning']['velocity_profile']['max_velocity'])        
        self.acc = AdaptiveCruiseControl(config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control'], max_velocity=config['planning']['velocity_profile']['max_velocity'], **config['map']['base_lla'])
        self.pid = PID(sampling_time = 1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(wheelbase=config['common']['wheelbase'], steer_ratio=config['common']['steer_ratio'],steer_max=config['common']['steer_max'], **config['control']['pure_pursuit'])
        self.base_lla = config['map']['base_lla']

    def pid_test(self, pid_gain):
        self.pid.change_gains(pid_gain[0], pid_gain[1], pid_gain[2])

    def execute(self, mode, vehicle_state, local_path, local_vel, local_kappa, lidar_object):
        if mode == 0:
            return Actuator(-100, vehicle_state.heading, vehicle_state.velocity), 0
        
        self.acc.check_objects(local_path)
        _, co = self.acc.calculate_curvature(local_kappa)
        target_velocity = self.acc.get_target_velocity(vehicle_state.velocity, local_vel[0], co)
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        steer_cmd = self.pure_pursuit.calculate_steering_angle(vehicle_state, local_path)
        return Actuator(acc_cmd, steer_cmd, vehicle_state.velocity), target_velocity
    
def main():
    self_drive = SelfDrive()
    self_drive.execute()

if __name__ == '__main__':
    main()