import math
import numpy as np


class cartesian_cnt:
    def __init__(self, g: float = 9.8):
        """
        Parameters
            g : float, optional
                The gravitational acceleration in m/s^2.
        """
        self.P_COEFF_FOR = np.array([5, 5, 0.0])  # Proportional Coefficient
        self.I_COEFF_FOR = np.array([0, 0, 0.0])  # Integrate Coefficient
        self.D_COEFF_FOR = np.array([10, 10, 0.0])  # Derivative Coefficient
        self.MAX_ROLL_PITCH = np.pi / 6  # Maximum Roll/Pitch
        self.GRAVITY = g
        self.control_timestep = 1.0 / 125.0
        self.scale_value = 1
        self.reset()

    def reset(self):
        """
        Resets the control classes.
        The previous step's and integral errors for both position and attitude are set to zero.
        """
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)
        self.control_counter = 0

    def getMatrix(self, cur_rpy):
        """
        Parameters
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
        Returns rotational matrix
        """
        print("Cur_rpy", cur_rpy)
        yaw = cur_rpy[2]  # Current yaw
        pitch = cur_rpy[1]  # Current Pitch
        roll = cur_rpy[0]  # Current Roll
        yaw_matrix = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )
        pitch_matrix = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )
        roll_matrix = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )
        return np.matmul(np.matmul(yaw_matrix, pitch_matrix), roll_matrix)

    def _simplePIDPositionControl(self, cur_pos, cur_rpy, target_pos, target_rpy):

        """
        Parameters
            cur_pos : A numpy array consisting current position X,Y,Z of drone
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
            target_pos : A numpy array consisting requried position of drone
        Returns required thrust,target roll,pitch,yaw of drone and position error
        """
        print("target_pos", target_pos)
        print("cur_pos", cur_pos)
        pos_e = 0.5 * np.clip(
            (target_pos - cur_pos), -1.0, 1.0
        )  # Calculating position error
        n_pos_e = np.array([0.0, 0.0, 0.0])
        print("delta_yaw", cur_rpy[2] - target_rpy[2])
        n_pos_e[1] = 1 * (
            np.cos(cur_rpy[2] - target_rpy[2]) * pos_e[0]
            + np.sin(cur_rpy[2] - target_rpy[2]) * pos_e[1]
        )
        n_pos_e[0] = -1 * (
            -np.sin(cur_rpy[2] - target_rpy[2]) * pos_e[0]
            + np.cos(cur_rpy[2] - target_rpy[2]) * pos_e[1]
        )
        pos_e = n_pos_e
        print("new_pose_error", n_pos_e)
        d_pos_e = (pos_e - self.last_pos_e) / self.control_timestep
        self.last_pos_e = pos_e  # Updating previous error
        self.integral_pos_e = (
            self.integral_pos_e + pos_e * self.control_timestep
        )  # Updating integral error
        target_rpy_1 = (
            0.0
            + np.multiply(self.P_COEFF_FOR, pos_e)
            + np.multiply(self.I_COEFF_FOR, self.integral_pos_e)
            + np.multiply(self.D_COEFF_FOR, d_pos_e)
        )  # Calulating Target force using PID controller

        # target_rpy = np.zeros(3)
        # sign_z = np.sign(target_force[2])
        # if sign_z == 0:
        #     sign_z = 1
        # # Calulating required Roll,Pitch,Yaw
        # target_rpy[0] = np.arcsin(
        #     -sign_z * target_force[1] / np.linalg.norm(target_force)
        # )
        # target_rpy[1] = np.arctan2(sign_z * target_force[0], sign_z * target_force[2])
        # target_rpy[2] = 0.0
        # target_rpy[0] = np.clip(
        #     target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH
        # )
        # target_rpy[1] = np.clip(
        #     target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH
        # )
        # Using rotation matrix to calculate thrust
        # cur_rotation = np.array(self.getMatrix(cur_rpy)).reshape(3, 3)
        # thrust = np.dot(cur_rotation, target_force)
        thrust = 0
        return thrust, target_rpy_1, pos_e

    def update(self, cur_pos, cur_rpy, target_pos, target_rpy):

        """
        Parameters
            cur_pos : A numpy array consisting current position X,Y,Z of drone
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
            target_pos : A numpy array consisting requried position of drone
        Returns required thrust and target roll,pitch,yaw of drone
        """

        self.control_counter += 1

        thrust, target_rpy_1, pos_e = self._simplePIDPositionControl(
            cur_pos, cur_rpy, target_pos, target_rpy
        )
        arr = [
            target_rpy_1[0] * self.scale_value,
            target_rpy_1[1] * self.scale_value,
            target_rpy_1[2] * self.scale_value,
        ]
        # print('target_rpy_from_xyz', target_rpy)
        return thrust, arr
