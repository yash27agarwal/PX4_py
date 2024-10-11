import numpy as np

class ThrustSaturation:
    def __init__(self, MPC_THR_MAX, MPC_THR_XY_MARG):
        self.lim_thr_max = MPC_THR_MAX
        self.lim_thr_xy_margin = MPC_THR_XY_MARG

    def saturate_thrust(self, thr_sp):
        """
        Saturate thrust in the horizontal and vertical directions, prioritizing
        vertical thrust control while keeping a margin for horizontal control.
        
        Parameters:
        thr_sp : np.array
            The thrust setpoint as a 3D vector [thr_x, thr_y, thr_z].

        Returns:
        np.array
            The saturated thrust setpoint.
        """
        # Horizontal thrust (x, y)
        thrust_sp_xy = thr_sp[:2]
        thrust_sp_xy_norm = np.linalg.norm(thrust_sp_xy)

        # Calculate thrust limits
        thrust_max_squared = self.lim_thr_max ** 2
        allocated_horizontal_thrust = min(thrust_sp_xy_norm, self.lim_thr_xy_margin)
        thrust_z_max_squared = thrust_max_squared - (allocated_horizontal_thrust ** 2)

        # Saturate maximal vertical thrust (z)
        thr_sp[2] = max(thr_sp[2], -np.sqrt(thrust_z_max_squared))

        # Calculate how much horizontal thrust is left after prioritizing vertical thrust
        thrust_max_xy_squared = thrust_max_squared - (thr_sp[2] ** 2)
        thrust_max_xy = 0

        if thrust_max_xy_squared > 0:
            thrust_max_xy = np.sqrt(thrust_max_xy_squared)

        # Saturate horizontal thrust (x, y)
        if thrust_sp_xy_norm > thrust_max_xy:
            thr_sp[:2] = (thrust_sp_xy / thrust_sp_xy_norm) * thrust_max_xy
        
        return thr_sp

# Example usage
if __name__ == "__main__":
    # Initialize with max thrust and XY margin
    MPC_THR_MAX = 1.0
    MPC_THR_XY_MARG = 0.8

    # Thrust setpoint (example)
    thr_sp = np.array([0.7, 0.7, -0.5])

    # Create the ThrustSaturation object
    thrust_control = ThrustSaturation(MPC_THR_MAX, MPC_THR_XY_MARG)

    # Saturate thrust
    saturated_thr_sp = thrust_control.saturate_thrust(thr_sp)

    print("Saturated Thrust Setpoint:", saturated_thr_sp)
