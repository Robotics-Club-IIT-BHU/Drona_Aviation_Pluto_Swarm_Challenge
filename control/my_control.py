# import libraries
import math
import numpy as np
from .xyz_control import cartesian_cnt


class my_controller:
    def __init__(self, initial_state=[0.0, 0.0, 0.0], g: float = 9.8):
        """
        Parameters
            g : float, optional
                The gravitational acceleration in m/s^2.
        """
        self.roll_P = 500
        self.roll_D = 500
        self.roll_I = 0.1
        self.pitch_P = 1000
        self.pitch_I = 0.1
        self.pitch_D = 0.5
        self.ini = initial_state
        self.last_error = [0, 0, 0]
        self.error_diff = [0, 0, 0]

        self.imax = 5
        self.roll_integral = 0
        self.pitch_integral = 0
        self.P_COEFF_FOR = np.array([70, 120, 0.0])  # Proportional Coefficient
        self.I_COEFF_FOR = np.array([100, 100, 0.0000])  # Integrate Coefficient
        self.D_COEFF_FOR = np.array([0, 0, 0.0])  # Derivative Coefficient
        self.MAX_ROLL_PITCH = np.pi / 6  # Maximum Roll/Pitch
        self.GRAVITY = g
        self.control_timestep = 1.0 / 125.0
        self.scale_value = 0
        self.scale_cart = 0
        self.pc = cartesian_cnt()
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
        for i in range(3):
            # tar=(6.2831+target_rpy[i]-self.ini[i])%6.2831
            cur_rpy[i] = (6.2831 + cur_rpy[i] - self.ini[i]) % 6.2831
            # cur_rpy[i]=(6.2831+cur_rpy[i]-tar)%6.2831
            dt = 10
            if cur_rpy[i] < 1.5:
                self.error_diff[i] = (cur_rpy[i] - self.last_error[i]) / dt
            else:
                self.error_diff[i] = ((-6.2831 + cur_rpy[0]) - self.last_error[i]) / dt

        if cur_rpy[0] < 1.5:
            self.roll_integral += cur_rpy[0]
            if cur_rpy[0] < 0.0003:
                self.roll_integral = 0
            elif cur_rpy[0] > 0.01:
                self.roll_integral = 0
            self.roll_integral = min(self.roll_integral, self.imax)
            roll_trim = -1 * (
                self.roll_P * cur_rpy[0] + self.roll_I * self.roll_integral
            )
        else:
            self.roll_integral += 6.2831 - cur_rpy[0]
            if 6.2831 - cur_rpy[0] < 0.0003:
                self.roll_integral = 0
            elif 6.2831 - cur_rpy[0] > 0.01:
                self.roll_integral = 0
            self.roll_integral = min(self.roll_integral, self.imax)
            roll_trim = (
                self.roll_P * (6.2831 - cur_rpy[0]) + self.roll_I * self.roll_integral
            )

        if cur_rpy[1] < 1.5:
            self.pitch_integral += cur_rpy[1]
            if cur_rpy[1] < 0.003:
                self.pitch_integral = 0
            elif cur_rpy[1] > 0.01:
                self.roll_integral = 0
            self.pitch_integral = min(self.pitch_integral, self.imax)
            pitch_trim = -1 * (
                self.pitch_P * cur_rpy[1] + self.pitch_I * self.pitch_integral
            )
        else:
            self.pitch_integral += 6.2831 - cur_rpy[1]
            if 6.2831 - cur_rpy[1] < 0.003:
                self.pitch_integral = 0
            elif 6.2831 - cur_rpy[1] > 0.01:
                self.roll_integral = 0
            self.pitch_integral = min(self.pitch_integral, self.imax)
            pitch_trim = (
                self.pitch_P * (6.2831 - cur_rpy[1])
                + self.pitch_I * self.pitch_integral
            )
        # roll_trim*=-1
        # print("roll_pi=",roll_trim*100)
        roll_trim += self.error_diff[0] * self.roll_D
        # print("roll_d=",self.error_diff[0]*self.roll_D*100)
        # print("pitch_pi=",pitch_trim*100)
        pitch_trim += self.error_diff[1] * self.pitch_D
        # print("pitch_d=",self.error_diff[1]*self.pitch_D*100)
        roll_trim = max(-20, min(roll_trim * 100, 20))
        pitch_trim = max(-20, min(pitch_trim * 100, 20))
        # print("Trim ",roll_trim,pitch_trim)
        arr = [
            1500 + roll_trim,
            1500 + pitch_trim,
            1900,
            1500,
        ]
        for i in range(len(arr)):
            arr[i] = int(max(min(arr[i], 2000), 900))

        return arr
