import numpy as np
import scipy.signal as signal

class MotorModel:
    """
    Class to implement motor dynamics using state-space equations:
    
    dx(t) = Ax(t) + Bu(t)
    y(t)  = Cx(t)
    """
    
    def __init__(self, scale_factor: float, armed_offset: float, motorT: float, 
                 initial_condition: float = 0.0):
        self.PWM_MAX = 2000
        self.PWM_MIN = 1000

        self.scale_factor = scale_factor
        self.armed_offset = armed_offset

        # State-space matrices for the motor dynamics
        self.motor_dynamics_A = -1 / motorT  # A: system dynamic coefficient
        self.motor_dynamics_B = 1  # B: input matrix
        self.motor_dynamics_C = 1 / motorT  # C: output matrix
        self.motor_dynamics_D = 0  # D: direct transmission is zero
        
        # Initial conditions for each motor (state variables)
        self.state_M1 = initial_condition
        self.state_M2 = initial_condition
        self.state_M3 = initial_condition
        self.state_M4 = initial_condition

    def __convertToRadps(self, M1, M2, M3, M4):
        """
        Convert PWM signals to rad/s motor speeds using scaling factors.
        """
        # Normalize PWM signals to [0, 1]
        normalize = lambda pwm: (pwm - self.PWM_MIN) / (self.PWM_MAX - self.PWM_MIN)
        
        # Scale to desired motor speed (rad/s)
        scaleToDesiredMotorSpeed = lambda M: M * self.scale_factor + self.armed_offset
        
        # Convert each motor's PWM signal to rad/s
        R1 = scaleToDesiredMotorSpeed(normalize(M1))
        R2 = scaleToDesiredMotorSpeed(normalize(M2))
        R3 = scaleToDesiredMotorSpeed(normalize(M3))
        R4 = scaleToDesiredMotorSpeed(normalize(M4))

        return R1, R2, R3, R4
    
    def calculate_motor_speed(self, M1, M2, M3, M4, dt):
        """
        Calculate motor speeds (rad/s) for four motors using the state-space equations.
        """
        # Convert PWM signals to rad/s
        R1, R2, R3, R4 = self.__convertToRadps(M1, M2, M3, M4)
        
        # Update motor speed using state-space dynamics for each motor
        def update_motor_speed(R, state):
            # State-space equation: dx = A * x + B * u
            dx = self.motor_dynamics_A * state + self.motor_dynamics_B * R
            
            # Update state based on the differential equation
            new_state = state + dx * dt
            
            # Output equation: y = C * x
            output = self.motor_dynamics_C * new_state
            
            return new_state, output
        
        # Update states and calculate motor speeds for all four motors independently
        self.state_M1, speed1 = update_motor_speed(R1, self.state_M1)
        self.state_M2, speed2 = update_motor_speed(R2, self.state_M2)
        self.state_M3, speed3 = update_motor_speed(R3, self.state_M3)
        self.state_M4, speed4 = update_motor_speed(R4, self.state_M4)
        
        return speed1, speed2, speed3, speed4

# Example usage of MotorModel class
if __name__ == "__main__":
    # Motor parameters
    scale_factor = 500  # Example scale factor (rad/s per PWM unit)
    armed_offset = 100  # Offset when motors are armed (rad/s)
    motorT = 0.5  # Motor time constant (s)
    initial_condition = 0.0  # Initial speed (rad/s)

    motor_model = MotorModel(scale_factor, armed_offset, motorT, initial_condition)

    # PWM inputs (example values)
    M1 = 1500
    M2 = 1500
    M3 = 1500
    M4 = 1500

    # Time step for simulation
    dt = 0.01  # 10 ms time step

    # Calculate motor speeds
    speed1, speed2, speed3, speed4 = motor_model.calculate_motor_speed(M1, M2, M3, M4, dt)
    
    print(f"Motor speeds (rad/s): M1={speed1:.2f}, M2={speed2:.2f}, M3={speed3:.2f}, M4={speed4:.2f}")
