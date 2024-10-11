from attitude_control import AttitudeController
from angular_rate_control import AngularRateController
from mixer import MotorMixer

class InnerLoopControlImpl:
    def __init__(self):
        self.current_angular_rate = [0, 0, 0]
        self.controller = AttitudeController(KP_ROLL_ANGLE=1.0, 
                                             KP_PITCH_ANGLE=1.0, 
                                             KP_YAW_ANGLE=1.0, 
                                             MC_YAW_WEIGHT=0.5, 
                                             MAX_CONTROL_ANGLE_RATE_PITCH=220, 
                                             MAX_CONTROL_ANGLE_RATE_ROLL=220, 
                                             MAX_CONTROL_ANGLE_RATE_YAW=200)
        
        self.angular_rate_controller = AngularRateController(KP_ROLL_RATE=1.0, 
                                                             KI_ROLL_RATE=0.1, 
                                                             KD_ROLL_RATE=0.01,
                                                             KP_PITCH_RATE=1.0, 
                                                             KI_PITCH_RATE=0.1, 
                                                             KD_PITCH_RATE=0.01,
                                                             KP_YAW_RATE=1.0, 
                                                             KI_YAW_RATE=0.1, 
                                                             KD_YAW_RATE=0.01, 
                                                             dt=0.01)
        self.mixer = MotorMixer()
        

    def control(self):
        

        att_q = [1, 0, 0, 0]
        att_setpoint_q = [0, 1, 0, 0]
        yawrate_d = 0.1

        rate_setpoint, qd, qd2 = self.controller.attitude_control(att_q, att_setpoint_q, yawrate_d)

        
        roll, pitch, yaw = self.angular_rate_controller.control(rate_setpoint, self.current_angular_rate)

        thrust = 1

        print(self.mixer.mix(torque_roll=roll, 
                             torque_pitch=pitch, 
                             torque_yaw=yaw, 
                             thrust=thrust)) 
        
if __name__ == "__main__":
    inner_loop = InnerLoopControlImpl()
    inner_loop.control()
        

