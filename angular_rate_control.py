from utils.pid_impl import BasePIDImpl
import numpy as np

class RateController:
    """
    Generic class for angular rate control (Roll, Pitch, Yaw)
    """
    def __init__(self, Kp: float, Ki: float, Kd: float, dt: float, min_torque: float, max_torque: float):
        """
        Initialize the RateController with PID parameters and min/max torque bounds.
        
        Parameters:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        dt (float): Time step for the PID controller.
        min_torque (float): Minimum torque limit.
        max_torque (float): Maximum torque limit.
        """
        self.pid = BasePIDImpl(Kp, Ki, Kd, dt)
        self.min_torque = min_torque    
        self.max_torque = max_torque
    
    def control(self, rate_sp: float, current_rate: float) -> float:
        """
        Compute the torque for the control axis (Roll, Pitch, or Yaw) using the PID controller.
        
        Parameters:
        rate_sp (float): The setpoint for the angular rate.
        current_rate (float): The measured angular rate.

        Returns:
        float: The computed torque for the control axis.
        """
        torque = self.pid.compute(rate_sp, current_rate)
        return np.clip(torque, self.min_torque, self.max_torque)


class AngularRateController:
    """
    Main class for controlling roll, pitch, and yaw angular rates using PID controllers.
    """
    def __init__(self, KP_ROLL_RATE: float, KI_ROLL_RATE: float, KD_ROLL_RATE: float,
                 KP_PITCH_RATE: float, KI_PITCH_RATE: float, KD_PITCH_RATE: float,
                 KP_YAW_RATE: float, KI_YAW_RATE: float, KD_YAW_RATE: float, dt: float):
        # Roll rate controller with torque bounds [-1, 1]
        self.roll_rate_controller = RateController(Kp=KP_ROLL_RATE, Ki=KI_ROLL_RATE, Kd=KD_ROLL_RATE,
                                                   dt=dt, min_torque=-1.0, max_torque=1.0)
        # Pitch rate controller with torque bounds [-1, 1]
        self.pitch_rate_controller = RateController(Kp=KP_PITCH_RATE, Ki=KI_PITCH_RATE, Kd=KD_PITCH_RATE,
                                                    dt=dt, min_torque=-1.0, max_torque=1.0)
        # Yaw rate controller with torque bounds [-0.5, 0.5]
        self.yaw_rate_controller = RateController(Kp=KP_YAW_RATE, Ki=KI_YAW_RATE, Kd=KD_YAW_RATE,
                                                  dt=dt, min_torque=-0.5, max_torque=0.5)

    def control(self, angular_rate_setpoint, current_angular_rate):
        """
        Compute the torque for roll, pitch, and yaw control using the respective controllers.

        Parameters:
        angular_rate_setpoint (list or array of float): Setpoints for roll, pitch, and yaw rates [roll_sp, pitch_sp, yaw_sp].
        current_angular_rate (list or array of float): Measured roll, pitch, and yaw rates [roll_rate, pitch_rate, yaw_rate].

        Returns:
        tuple of float: Computed torques for roll, pitch, and yaw (torque_roll, torque_pitch, torque_yaw).
        """
        torque_roll = self.roll_rate_controller.control(rate_sp=angular_rate_setpoint[0], 
                                                        current_rate=current_angular_rate[0])
        
        torque_pitch = self.pitch_rate_controller.control(rate_sp=angular_rate_setpoint[1],
                                                          current_rate=current_angular_rate[1])
        
        torque_yaw = self.yaw_rate_controller.control(rate_sp=angular_rate_setpoint[2],
                                                      current_rate=current_angular_rate[2])
        return torque_roll, torque_pitch, torque_yaw


# Example usage
# if __name__ == "__main__":
#     dt: float = 0.01  # Time step

#     # Initialize the AngularRateController
#     angular_rate_controller = AngularRateController(roll_r_kp=1.0, roll_r_ki=0.1, roll_r_kd=0.01,
#                                                     pitch_r_kp=1.0, pitch_r_ki=0.1, pitch_r_kd=0.01,
#                                                     yaw_r_kp=1.0, yaw_r_ki=0.1, yaw_r_kd=0.01, dt=dt)

#     # Define setpoints and measured rates for roll, pitch, and yaw
#     angular_rate_sp = [1.0, 1.0, 1.0]  # Setpoints for roll, pitch, yaw rates
#     current_angular_rate = [0.5, 0.6, 0.7]  # Measured roll, pitch, yaw rates

#     # Compute the torques
#     torque_roll, torque_pitch, torque_yaw = angular_rate_controller.control(angular_rate_sp, current_angular_rate)
    
#     print(f'Torque roll: {torque_roll}')
#     print(f'Torque pitch: {torque_pitch}')
#     print(f'Torque yaw: {torque_yaw}')
