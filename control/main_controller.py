# import libraries
import math
import numpy as np
from .xyz_control import cartesian_cnt


class main_controller:
    def __init__(self, g: float = 9.8):
        """
        Parameters
            g : float, optional
                The gravitational acceleration in m/s^2.
        """
        self.P_COEFF_FOR = np.array([100, 100, 0.0])  # Proportional Coefficient
        self.I_COEFF_FOR = np.array([1000, 1000, 0.0000])  # Integrate Coefficient
        self.D_COEFF_FOR = np.array([0, 0, 0])  # Derivative Coefficient
        self.MAX_ROLL_PITCH = np.pi / 6  # Maximum Roll/Pitch
        self.GRAVITY = g
        self.control_timestep = 1.0 / 125.0
        self.scale_value = 100
        self.scale_cart = 0
        self.pc = cartesian_cnt()
        self.reset()

    def reset(self):
        """
        Resets the control classes.
        The previous step's and integral errors for both position and attitude are set to zero.
        """
        self.last_pos_e = np.zeros(3)
        self.last_position_e = np.zeros(3)
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

    def _simplePIDPositionControl(self, cur_pos, cur_rpy, target_rpy):

        """
        Parameters
            cur_pos : A numpy array consisting current position X,Y,Z of drone
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
            target_pos : A numpy array consisting requried position of drone
        Returns required thrust,target roll,pitch,yaw of drone and position error
        """

        pos_e = np.sin(target_rpy) - np.sin(cur_rpy)  # Calculating position error
        # # pos_e=np.cos(pos_e)
        d_pos_e = (pos_e - self.last_pos_e) / self.control_timestep
        self.last_pos_e = pos_e  # Updating previous error
        self.integral_pos_e = (
            self.integral_pos_e + pos_e * self.control_timestep
        )  # Updating integral error
        required_rpy = (
            0.0
            + np.multiply(self.P_COEFF_FOR, pos_e)
            + np.multiply(self.I_COEFF_FOR, self.integral_pos_e)
            + np.multiply(self.D_COEFF_FOR, d_pos_e)
        )  # Calulating Target for0ce using PID controller
        if abs(pos_e[0]) > 0.08:
            self.integral_pos_e[0] = 0
        if abs(pos_e[1]) > 0.08:
            self.integral_pos_e[1] = 0

        print("P:", np.multiply(self.P_COEFF_FOR, pos_e))
        print("D:", np.multiply(self.D_COEFF_FOR, d_pos_e))
        print("I:", np.multiply(self.I_COEFF_FOR, self.integral_pos_e))
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
        thrust = 0  # np.dot(cur_rotation, target_force)
        return thrust, required_rpy, pos_e

    def update(self, cur_pos, target_pos, cur_rpy, target_rpy):

        """
        Parameters
            cur_pos : A numpy array consisting current position X,Y,Z of drone
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
            target_pos : A numpy array consisting requried position of drone
        Returns required thrust and target roll,pitch,yaw of drone
        """

        self.control_counter += 1

        # _, computed_target_rpy, pos_e = self._simplePIDPositionControl(
        #     cur_pos, cur_rpy, target_rpy
        # )
        thrust, computed_target_rpy = self.pc.update(
            cur_pos, cur_rpy, target_pos, target_rpy
        )

        _, computed_target_rpy_2, pos_e = self._simplePIDPositionControl(
            cur_pos, cur_rpy, target_rpy
        )

        print(f"delta rpy,{computed_target_rpy_2}")
        # print('curr_rpy_to_rpy_cnt', cur_rpy)
        # print(f"Position1,{computed_target_rpy_pc}")
        print(f"RPY1,{computed_target_rpy}")
        arr = [
            1497
            + (computed_target_rpy[0] * self.scale_cart)
            + (computed_target_rpy_2[0] * self.scale_value),
            1502
            + (computed_target_rpy[1] * self.scale_cart)
            + (computed_target_rpy_2[1] * self.scale_value),
            1650,
            1500
            + (computed_target_rpy[2] * self.scale_cart)
            + (
                computed_target_rpy_2[2] * self.scale_value
            ),  # +computed_target_rpy[2]*self.scale_value,
        ]
        for i in range(len(arr)):
            arr[i] = max(min(arr[i], 2100), 900)

        return arr
