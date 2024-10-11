import numpy as np
from scipy.spatial.transform import Rotation as R

class AttitudeController:
    def __init__(self, KP_ROLL_ANGLE: float = 1.0, KP_PITCH_ANGLE: float = 1.0, KP_YAW_ANGLE: float = 1.0, 
                 MC_YAW_WEIGHT: float = 0.5, MAX_CONTROL_ANGLE_RATE_PITCH: float = 220, 
                 MAX_CONTROL_ANGLE_RATE_ROLL: float = 220, MAX_CONTROL_ANGLE_RATE_YAW: float = 200):
        
        self.KP_ROLL_ANGLE = KP_ROLL_ANGLE
        self.KP_PITCH_ANGLE = KP_PITCH_ANGLE
        self.KP_YAW_ANGLE = KP_YAW_ANGLE
        self.MC_YAW_WEIGHT = MC_YAW_WEIGHT
        self.MAX_CONTROL_ANGLE_RATE_PITCH = MAX_CONTROL_ANGLE_RATE_PITCH
        self.MAX_CONTROL_ANGLE_RATE_ROLL = MAX_CONTROL_ANGLE_RATE_ROLL
        self.MAX_CONTROL_ANGLE_RATE_YAW = MAX_CONTROL_ANGLE_RATE_YAW

    def __quat_multiply(self, q1, q2):
        """Quaternion multiplication"""
        return R.from_quat(q1).as_quat() * R.from_quat(q2).as_quat()

    def __quat_inverse(self, q):
        """Quaternion inverse"""
        return R.from_quat(q).inv().as_quat()

    def __quat2vec(self, e_z, e_z_d):
        """Convert quaternion to vector representation for reduced attitude"""
        n = np.cross(e_z, e_z_d)
        norm_n = np.linalg.norm(n)
        if norm_n > 1e-5:
            n = n / norm_n
            a = np.arctan2(np.linalg.norm(np.cross(e_z, e_z_d)), np.dot(e_z, e_z_d))
            return np.array([np.cos(a / 2), np.sin(a / 2) * n[0], np.sin(a / 2) * n[1], np.sin(a / 2) * n[2]])
        else:
            return np.array([1, 0, 0, 0])
        
    def __rate_setpoint_saturation(self, rate_setpoint):
        """Returns ratepoint after saturation"""
        rate_setpoint[0] = np.clip(rate_setpoint[0], -self.MAX_CONTROL_ANGLE_RATE_ROLL, self.MAX_CONTROL_ANGLE_RATE_ROLL)
        rate_setpoint[1] = np.clip(rate_setpoint[1], -self.MAX_CONTROL_ANGLE_RATE_PITCH, self.MAX_CONTROL_ANGLE_RATE_PITCH)
        rate_setpoint[2] = np.clip(rate_setpoint[2], -self.MAX_CONTROL_ANGLE_RATE_YAW, self.MAX_CONTROL_ANGLE_RATE_YAW)
        return rate_setpoint

    def control(self, att_q, att_setpoint_q, yawrate_d):
        q = np.array(att_q)
        qd = np.array(att_setpoint_q)
        
        # Calculate reduced desired attitude, neglect yaw
        rotm = R.from_quat(q).as_matrix()
        e_z = rotm[:, 2]
        rotm_d = R.from_quat(qd).as_matrix()
        e_z_d = rotm_d[:, 2]
        
        qd_red = self.__quat2vec(e_z, e_z_d)
        
        if abs(qd_red[1]) > (1.0 - 1e-5) or abs(qd_red[2]) > (1.0 - 1e-5):
            qd_red = qd
        else:
            qd_red = self.__quat_multiply(qd_red, q)

        qd2 = qd_red  # Debugging purpose
        
        # Mix full and reduced desired attitude
        q_mix = self.__quat_multiply(self.__quat_inverse(qd_red), qd)
        q_mix[0] = max(min(q_mix[0], 1.0), -1.0)
        q_mix[3] = max(min(q_mix[3], 1.0), -1.0)

        yaw_w = self.MC_YAW_WEIGHT
        qd_ = self.__quat_multiply(
            qd_red,
            [np.cos(yaw_w * np.arccos(q_mix[0])), 0, 0, np.sin(yaw_w * np.arcsin(q_mix[3]))]
        )
        qd = qd_

        # Quaternion attitude control law
        qe = self.__quat_multiply(self.__quat_inverse(q), qd)
        if abs(qe[0]) > 1e-5:
            qe = qe * np.sign(qe[0])
        eq = 2.0 * np.array([qe[1], qe[2], qe[3]])

        # Calculate angular rates setpoint
        rate_setpoint = np.array([
            self.KP_ROLL_ANGLE * eq[0],
            self.KP_PITCH_ANGLE * eq[1],
            self.KP_YAW_ANGLE * eq[2]
        ])

        # Feed forward yawrate setpoint
        if np.isfinite(yawrate_d):
            rotm_t = R.from_quat(self.__quat_inverse(q)).as_matrix()
            rate_setpoint += rotm_t[:, 2] * yawrate_d

        return self.__rate_setpoint_saturation(rate_setpoint), qd, qd2

# Example usage:
# controller = AttitudeController(KP_ROLL_ANGLE=1.0, KP_PITCH_ANGLE=1.0, KP_YAW_ANGLE=1.0, MC_YAW_WEIGHT=0.5, 
#                                 MAX_CONTROL_ANGLE_RATE_PITCH = 220, MAX_CONTROL_ANGLE_RATE_ROLL = 220, MAX_CONTROL_ANGLE_RATE_YAW = 200)

# att_q = [1, 0, 0, 0]
# att_setpoint_q = [0, 1, 0, 0]
# yawrate_d = 0.1

# rate_setpoint, qd, qd2 = controller.attitude_control(att_q, att_setpoint_q, yawrate_d)
# print("Rate Setpoint:", rate_setpoint)
# print("qd:", qd)
# print("qd2:", qd2)
