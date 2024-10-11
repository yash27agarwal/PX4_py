import numpy as np

class ThrustController:
    def __init__(self, constants_one_g, deg2rad, mpc_tiltmax_air, thr_hover, mpc_thr_min, flt_epsilon):
        self.CONSTANTS_ONE_G = constants_one_g
        self.DEG2RAD = deg2rad
        self.MPC_TILTMAX_AIR = mpc_tiltmax_air
        self.THR_HOVER = thr_hover
        self.MPC_THR_MIN = mpc_thr_min
        self.FLT_EPSILON = flt_epsilon

    def acceleration_control(self, accx_sp, accy_sp, accz_sp, sel_method=2):
        lim_thr_min = self.MPC_THR_MIN
        acc_sp = np.array([accx_sp, accy_sp, accz_sp])
        thr_sp = np.zeros(3)

        if sel_method == 1:
            body_z = np.array([-acc_sp[0], -acc_sp[1], self.CONSTANTS_ONE_G])
            body_z = body_z / np.linalg.norm(body_z)

            lim_tilt = self.MPC_TILTMAX_AIR * self.DEG2RAD
            body_z = self.limit_tilt(body_z, np.array([0, 0, 1]), lim_tilt)

            hover_thrust = self.THR_HOVER
            collective_thrust = acc_sp[2] * (hover_thrust / self.CONSTANTS_ONE_G) - hover_thrust
            collective_thrust /= np.dot(np.array([0, 0, 1]), body_z)
            collective_thrust = min(collective_thrust, -lim_thr_min)

            thr_sp = body_z * collective_thrust

        else:
            if acc_sp[2] < self.CONSTANTS_ONE_G:
                body_z = np.array([-acc_sp[0], -acc_sp[1], -acc_sp[2] + self.CONSTANTS_ONE_G])
            else:
                body_z = np.array([-acc_sp[0], -acc_sp[1], 0])
                acc_sp[2] = self.CONSTANTS_ONE_G

            body_z = body_z / np.linalg.norm(body_z)

            lim_tilt = self.MPC_TILTMAX_AIR * self.DEG2RAD
            body_z = self.limit_tilt(body_z, np.array([0, 0, 1]), lim_tilt)

            hover_thrust = self.THR_HOVER
            collective_thrust = -np.linalg.norm(acc_sp + np.array([0, 0, -self.CONSTANTS_ONE_G])) * (hover_thrust / self.CONSTANTS_ONE_G)
            collective_thrust = min(collective_thrust, -lim_thr_min)

            thr_sp = body_z * collective_thrust

        return thr_sp

    def limit_tilt(self, body_unit, world_unit, max_angle):
        dot_product_unit = np.dot(body_unit, world_unit)
        angle = np.arccos(dot_product_unit)

        angle = min(angle, max_angle)
        rejection = body_unit - (dot_product_unit * world_unit)

        if np.linalg.norm(rejection) ** 2 < self.FLT_EPSILON:
            rejection[0] = 1.0

        body_unit = np.cos(angle) * world_unit + np.sin(angle) * (rejection / np.linalg.norm(rejection))
        return body_unit


# Example usage:
controller = ThrustController(constants_one_g=9.81, deg2rad=np.pi/180, mpc_tiltmax_air=45, 
                              thr_hover=0.5, mpc_thr_min=0.1, flt_epsilon=1e-6)

thrust_sp = controller.acceleration_control(accx_sp=0.0, accy_sp=0.0, accz_sp=9.8, sel_method=2)
print(thrust_sp)
