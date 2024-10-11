class BasePI_PImpl:
    def __init__(self, Kp_pi: float, Ki_pi: float, Kp_p: float, dt: float) -> None:
        """
        Initialize the PI + P controller with the given gains and time step.
        
        Args:
            Kp_pi (float): Proportional gain for the PI controller (first control loop)
            Ki_pi (float): Integral gain for the PI controller (first control loop)
            Kp_p (float): Proportional gain for the P controller (second control loop)
            dt (float): Control time step
        """
        self.Kp_pi = Kp_pi  # Proportional gain for the PI loop
        self.Ki_pi = Ki_pi  # Integral gain for the PI loop
        self.Kp_p = Kp_p    # Proportional gain for the P loop
        self.dt = dt        # Time step

        self.integral_error = 0  # Integral of the error
        self.prev_error = 0      # Previous error for derivative/integral calculations

    def reset(self) -> None:
        """
        Resets the PI + P controller.
        """
        self.integral_error = 0
        self.prev_error = 0

    def pi_control(self, setpoint: float, measured_value: float) -> float:
        """
        PI control loop for general quantities.
        
        Args:
            setpoint (float): Desired value of the controlled quantity
            measured_value (float): Actual value of the controlled quantity
        
        Returns:
            float: Output of the PI controller (e.g., desired rate of change)
        """
        error = setpoint - measured_value
        
        # Proportional term
        P = self.Kp_pi * error
        
        # Integral term
        self.integral_error += error * self.dt
        I = self.Ki_pi * self.integral_error
        
        # Output (desired rate of change)
        output = P + I
        
        return output

    def p_control(self, setpoint: float, measured_value: float) -> float:
        """
        P control loop for general quantities.
        
        Args:
            setpoint (float): Desired rate (e.g., velocity, etc.)
            measured_value (float): Actual rate (e.g., current velocity)
        
        Returns:
            float: Output of the P controller (e.g., desired acceleration)
        """
        error = setpoint - measured_value
        
        # Proportional term
        P = self.Kp_p * error
        
        # Output (desired control signal)
        output = P 
        
        return output

    def compute(self, setpoint: float, rate_setpoint: float, measured_value: float, 
                measured_rate_value: float) -> float:
        """
        Compute the control output by running the PI control (for setpoint) and P control (for rate of change).
        
        Args:
            setpoint (float): Desired value of the controlled quantity
            rate_setpint (float): Desired value of the controlled rate quantity
            measured_value (float): Actual value of the controlled quantity
            measured_rate_value (float): Actual rate of change of the controlled quantity
        
        Returns:
            float: Desired rate of change (e.g., acceleration, power, etc.) as control output
        """
        # Step 1: PI control to compute the desired rate of change
        pi_output = self.p_control(setpoint, measured_value)

        # Step 2: P control to compute the final control signal
        p_output = self.pi_control(rate_setpoint, measured_rate_value)

        return pi_output + p_output
        


# # Example usage:
# controller = BasePI_PImpl(Kp_pi=1.0, Ki_pi=0.1, Kp_p=0.5, dt=0.01)  # Initialize controller with gains

# setpoint = 2.0         # Example setpoint (desired value)
# measured_value = 1.5   # Current actual value
# measured_rate_value = 0.4  # Current rate of change

# # Compute the control signal (e.g., desired acceleration or power)
# control_signal = controller.compute(setpoint, measured_value, measured_rate_value)
# print(f"Control Signal: {control_signal}")
