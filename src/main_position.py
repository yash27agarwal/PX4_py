from outer_loop.position_control import OuterLoopController

class OuterLoopControlImpl:
    def __init__(self):
        self.outer_controller = OuterLoopController(KP_PI_X=1.0, KI_PI_X=0.1, KP_P_X=0.5,
                                       KP_PI_Y=1.0, KI_PI_Y=0.1, KP_P_Y=0.5,
                                       KP_PI_Z=1.0, KI_PI_Z=0.1, KP_P_Z=0.5, dt=0.01)
        
        self.current_position = [1.5, 2.8, 0.8]
        self.current_velocity = [0.4, 0.5, 0.3]

    def control(self):

        # # Desired position and velocity setpoints [x_sp, y_sp, z_sp], [vx_sp, vy_sp, vz_sp]
        position_sp = [2.0, 3.0, 1.0]
        velocity_sp = [0.5, 0.6, 0.4]
        acceleration_setpoints = self.outer_controller.control(position_sp, velocity_sp, 
                                                               self.current_position, 
                                                               self.current_velocity)
        print(f"Acceleration Setpoints: {acceleration_setpoints}")


if __name__ == "__main__":
    outer_loop = OuterLoopControlImpl()
    outer_loop.control()




# # Current position and velocity measurements [x, y, z], [vx, vy, vz]
# current_position = [1.5, 2.8, 0.8]
# current_velocity = [0.4, 0.5, 0.3]

# # Compute acceleration setpoints for all axes


