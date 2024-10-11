from utils.pi_p_control_impl import BasePI_PImpl

class OuterLoopController:
    def __init__(self, KP_PI_X, KI_PI_X, KP_P_X, 
                 KP_PI_Y, KI_PI_Y, KP_P_Y,
                 KP_PI_Z, KI_PI_Z, KP_P_Z, dt):
        """
        Initialize controllers for X, Y, and Z axes.
        
        Args:
            KP_PI_X, KI_PI_X, KP_P_X: Gains for X-axis PI + P controller
            KP_PI_Y, KI_PI_Y, KP_P_Y: Gains for Y-axis PI + P controller
            KP_PI_Z, KI_PI_Z, KP_P_Z: Gains for Z-axis PI + P controller
        """
        self.x_controller = PI_P(KP_PI_X, KI_PI_X, KP_P_X, dt)  # X-axis controller
        self.y_controller = PI_P(KP_PI_Y, KI_PI_Y, KP_P_Y, dt)  # Y-axis controller
        self.z_controller = PI_P(KP_PI_Z, KI_PI_Z, KP_P_Z, dt)  # Z-axis controller

    def control(self, position_sp, velocity_sp, current_position, current_velocity):
        """
        Computes acceleration setpoints for X, Y, and Z axes.
        
        Args:
            position_sp (list): Desired position [x_sp, y_sp, z_sp]
            velocity_sp (list): Desired velocity [vx_sp, vy_sp, vz_sp]
            current_position (list): Current position [x, y, z]
            current_velocity (list): Current velocity [vx, vy, vz]cd
        
        Returns:
            list: Acceleration setpoints [acc_x_sp, acc_y_sp, acc_z_sp]
        """
        # X-axis acceleration setpoint
        acc_x_sp = self.x_controller.compute(position_sp[0], velocity_sp[0], 
                                             current_position[0], current_velocity[0])
        
        # Y-axis acceleration setpoint
        acc_y_sp = self.y_controller.compute(position_sp[1], velocity_sp[1], 
                                             current_position[1], current_velocity[1])
        
        # Z-axis acceleration setpoint
        acc_z_sp = self.z_controller.compute(position_sp[2], velocity_sp[2], 
                                             current_position[2], current_velocity[2])
        
        # Return the setpoints as a list [acc_x_sp, acc_y_sp, acc_z_sp]
        return [acc_x_sp, acc_y_sp, acc_z_sp]


class PI_P:
    def __init__(self, Kp_pi, Ki_pi, Kp_p, dt):
        """
        Initialize the PI + P controller.
        
        Args:
            Kp_pi (float): Proportional gain for PI controller
            Ki_pi (float): Integral gain for PI controller
            Kp_p (float): Proportional gain for P controller
            dt (float): Control time step 
        """
        self.controller = BasePI_PImpl(Kp_pi, Ki_pi, Kp_p, dt)
 
    def compute(self, position_sp, velocity_sp, current_position, current_velocity):
        """
        Compute the control signal (acceleration setpoint).
        
        Args:
            position_sp (float): Desired position setpoint
            velocity_sp (float): Desired velocity setpoint
            current_position (float): Current position
            current_velocity (float): Current velocity
        
        Returns:
            float: Control signal (acceleration setpoint)
        """
        control_sp = self.controller.compute(position_sp, velocity_sp, current_position, current_velocity)
        return control_sp
    
# Initialize the outer loop controller with gains for each axis
# outer_controller = OuterLoopController(KP_PI_X=1.0, KI_PI_X=0.1, KP_P_X=0.5,
#                                        KP_PI_Y=1.0, KI_PI_Y=0.1, KP_P_Y=0.5,
#                                        KP_PI_Z=1.0, KI_PI_Z=0.1, KP_P_Z=0.5, dt=0.01)

# # Desired position and velocity setpoints [x_sp, y_sp, z_sp], [vx_sp, vy_sp, vz_sp]
# position_sp = [2.0, 3.0, 1.0]
# velocity_sp = [0.5, 0.6, 0.4]

# # Current position and velocity measurements [x, y, z], [vx, vy, vz]
# current_position = [1.5, 2.8, 0.8]
# current_velocity = [0.4, 0.5, 0.3]

# # Compute acceleration setpoints for all axes
# acceleration_setpoints = outer_controller.control(position_sp, velocity_sp, current_position, current_velocity)

# print(f"Acceleration Setpoints: {acceleration_setpoints}")
