import numpy as np

class MotorMixer:
    """
    Class implementation of a PX4 mixer.
    """
    def __init__(self, idle_PWM: int=1000, scale=1000, 
                 PWM_MIN: int = 1000, PWM_MAX: int = 2000):
        """
        Initialize the MotorMixer with default idle PWM and scale values.

        Parameters:
            idle_PWM (int): The idle PWM signal, default 1000.
            scale (float): The scale factor for mixing, default 1000.
            PWM_MIN (int): Minimum possible PWM value, default 1000.
            PWM_MAX (int): Maximum possible PWM value, default 2000.

        """
        self.idle_PWM = idle_PWM
        self.scale = scale
        self.PWM_MIN = PWM_MIN
        self.PWM_MAX = PWM_MAX

    def __saturation(self, M1: float, M2: float, M3: float, M4: float):
        """
        Clips the output between PWM_MIN and PWM_MAX.
        """
        M1 = np.clip(M1, self.PWM_MIN, self.PWM_MAX)
        M2 = np.clip(M2, self.PWM_MIN, self.PWM_MAX)
        M3 = np.clip(M3, self.PWM_MIN, self.PWM_MAX)
        M4 = np.clip(M4, self.PWM_MIN, self.PWM_MAX)
        
        return M1, M2, M3, M4

    def mix(self, torque_roll, torque_pitch, torque_yaw, thrust):
        """
        Control allocation for a quadrotor in X-configuration using torque inputs.

        Parameters:
            torque_roll (float): Torque output for roll.
            torque_pitch (float): Torque output for pitch.
            torque_yaw (float): Torque output for yaw.
            thrust (float): Thrust control output.

        Returns:
            tuple (float): Motor outputs for the quadrotor.
        """
        M1 = (thrust - torque_roll + torque_pitch + torque_yaw) * self.scale + self.idle_PWM
        M2 = (thrust + torque_roll - torque_pitch + torque_yaw) * self.scale + self.idle_PWM
        M3 = (thrust + torque_roll + torque_pitch - torque_yaw) * self.scale + self.idle_PWM
        M4 = (thrust - torque_roll - torque_pitch - torque_yaw) * self.scale + self.idle_PWM

        return self.__saturation(M1, M2, M3, M4)

# Example usage:
# Create an instance of the MotorMixer class
# mixer = MotorMixer()

# # Get the motor outputs
# M1, M2, M3, M4 = mixer.mix(torque_roll_value, torque_pitch_value, torque_yaw_value, thrust_value)
