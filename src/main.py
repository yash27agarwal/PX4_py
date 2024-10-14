import yaml
from inner_loop.attitude_control import AttitudeController
from inner_loop.angular_rate_control import AngularRateController
from mixer.mixer import MotorMixer
from motor_model.motor_model import MotorModel
from pathlib import Path
import os

base_path = Path(__file__).parent.parent

class InnerLoopControlImpl:
    def __init__(self, config_file):
        # Load the YAML configuration file
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        
        # Extract values from YAML under 'InnerLoop' heading
        attitude_config = config['InnerLoop']['AttitudeController']
        angular_rate_config = config['InnerLoop']['AngularRateController']
        motor_config = config['MotorModel']

        self.current_angular_rate = [0, 0, 0]

        # Initialize controllers using YAML values
        self.controller = AttitudeController(KP_ROLL_ANGLE=attitude_config['MC_ROLL_P'], 
                                             KP_PITCH_ANGLE=attitude_config['MC_PITCH_P'], 
                                             KP_YAW_ANGLE=attitude_config['MC_YAW_P'], 
                                             MC_YAW_WEIGHT=attitude_config['MC_YAW_WEIGHT'], 
                                             MAX_CONTROL_ANGLE_RATE_PITCH=attitude_config['MAX_CONTROL_ANGLE_RATE_PITCH'], 
                                             MAX_CONTROL_ANGLE_RATE_ROLL=attitude_config['MAX_CONTROL_ANGLE_RATE_ROLL'], 
                                             MAX_CONTROL_ANGLE_RATE_YAW=attitude_config['MAX_CONTROL_ANGLE_RATE_YAW'])
        
        self.angular_rate_controller = AngularRateController(KP_ROLL_RATE=angular_rate_config['MC_ROLLRATE_P'], 
                                                             KI_ROLL_RATE=angular_rate_config['MC_ROLLRATE_I'], 
                                                             KD_ROLL_RATE=angular_rate_config['MC_ROLLRATE_D'],
                                                             KP_PITCH_RATE=angular_rate_config['MC_PITCHRATE_P'], 
                                                             KI_PITCH_RATE=angular_rate_config['MC_PITCHRATE_I'], 
                                                             KD_PITCH_RATE=angular_rate_config['MC_PITCHRATE_D'],
                                                             KP_YAW_RATE=angular_rate_config['MC_YAWRATE_P'], 
                                                             KI_YAW_RATE=angular_rate_config['MC_YAWRATE_I'], 
                                                             KD_YAW_RATE=angular_rate_config['MC_YAWRATE_D'], 
                                                             dt=angular_rate_config['dt'])
        self.mixer = MotorMixer()

        self.motor_model = MotorModel(scale_factor=motor_config['scale_factor'],
                                      armed_offset=motor_config['offset_factor'],
                                      motorT=motor_config['time_constant'],
                                      initial_condition=motor_config['initial_conditions'],
                                      dt=motor_config['dt'])
        

    def control(self):
        att_q = [1, 0, 0, 0]
        att_setpoint_q = [0, 1, 0, 0]
        yawrate_d = 0.1

        # Call attitude control method
        rate_setpoint, _, _ = self.controller.attitude_control(att_q, att_setpoint_q, yawrate_d)

        # Call angular rate control method
        torque_roll, torque_pitch, torque_yaw = self.angular_rate_controller.control(rate_setpoint, self.current_angular_rate)

        thrust = 1

        # Mix control outputs to motor commands
        M1, M2, M3, M4 = self.mixer.mix(torque_roll=torque_roll, 
                                        torque_pitch=torque_pitch, 
                                        torque_yaw=torque_yaw, 
                                        thrust=thrust)
        
        print(self.motor_model.calculate_motor_speed(M1, M2, M3, M4))
        
if __name__ == "__main__":
    yaml_file_path = os.path.join(base_path, 'config', 'config.yaml')
    inner_loop = InnerLoopControlImpl(yaml_file_path)
    inner_loop.control()
