class BasePIDImpl:
    """
        This class is the implementation of a PID controller.
        
        Methods:
            - __init__(Kp, Ki, Kd, dt) -> None
            - reset() -> None
            - compute() -> float
        """
    
    def __init__(self, Kp: float, Ki: float, Kd: float, 
                 dt: float) -> None:
        """
        Initialize the PID imlplementation class. 

        Args:
            Kp (float): Propotational gain
            Ki (float): Integral gain
            Kd (float): Differential gain
            dt (float): control time step
        """
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.dt = dt  # Time step

        # Initialize terms
        self.prev_error = 0
        self.integral = 0

    def reset(self) -> None: 
        """
        Resets the PID controller.
        """
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value) -> float:
        """
        Calculates the control output and returns it.

        Args:
            setpoint (float): desired setpoint value
            measured_value (float): current value of the quantity

        Returns:
            float: control output

        """
        # Calculate the error
        error = setpoint - measured_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * self.dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        
        # Remember previous error for next derivative calculation
        self.prev_error = error
        
        # Calculate the output (control signal)
        output = P + I + D
        
        return output
