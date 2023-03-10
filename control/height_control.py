import math
import numpy as np
import cv2
import time


class altitude_cnt:
    def __init__(self, g: float = 9.8, tracker_mode: bool = False):
        self.p = 400
        self.d = 40000
        self.i = 800
        self.last_time = 0
        self.control_timestep = 1.0 / 60.0
        self.GRAVITY = g
        self.tracker_mode = tracker_mode
        self.reset()
        if self.tracker_mode:
            self.tracker_window()

    def reset(self):

        self.last_height_e = 0
        self.integral_height_e = 0
        self.control_counter = 0

    def current_time(self):
        return round(time.time() * 1000)

    def altitude_control(self, target_height, cur_pos, cur_rpy):
        tnow = self.current_time()
        dt = tnow - self.last_time
        # print("dt=",dt)
        self.last_time = tnow
        z_err = target_height - cur_pos[2]
        # print("Values:" ,target_height, cur_pos[2], z_err)
        z_err_dot = (z_err - self.last_height_e) * self.control_timestep
        self.last_height_e = z_err

        self.integral_height_e = self.integral_height_e + z_err * self.control_timestep
        if abs(z_err) < 0.03 or abs(z_err) > 0.1:
            self.integral_height_e = 0
        # print(z_err, z_err_dot, self.last_height_e, self.integral_height_e)
        # thrust = (self.p * z_err + self.d * z_err_dot + self.GRAVITY) / math.cos(cur_rpy[0])*math.cos(cur_rpy[1])
        intg = self.i * self.integral_height_e

        if intg < 0:
            intg *= 20
        thrust = (
            (
                self.p * z_err * (0.8 if abs(z_err) > 0.4 else 1.0)
                + self.d * z_err_dot
                + intg
            )
            / math.cos(cur_rpy[0])
            * math.cos(cur_rpy[1])
        )

        if math.cos(cur_rpy[0]) < 0.1 or math.cos(cur_rpy[1]) < 0.1:
            thrust = 0
        # if z_err<0:
        # thrust *= 8
        # print("p: ", self.p * z_err)
        # print("diff: ", self.d * z_err_dot)
        # print("intg: ", self.i * self.integral_height_e)
        # print("thrust: ", thrust)
        if intg < 0:
            intg *= 10
        if thrust < 0:
            thrust /= 20
        thrust = thrust + 1600
        # print("After thrust- ", thrust)

        thrust = int(min(2100, max(1200, thrust)))
        return [1500, 1500, 1550, 1500]

    def update(self, cur_pos, cur_rpy, target_pos):
        self.control_counter += 1
        if self.tracker_mode:
            self.update_pid()
        return self.altitude_control(target_pos, cur_pos, cur_rpy)

    def kill(self):
        return [1500, 1500, 1300, 1500]

    def __del__(self):
        cv2.destroyAllWindows()
        return True

    def nothing(self, x):
        pass

    def tracker_window(self):
        cv2.namedWindow("controls")
        cv2.createTrackbar("p", "controls", 20 * self.p, 500, self.nothing)
        cv2.createTrackbar("i", "controls", 100 * self.i, 250, self.nothing)
        cv2.createTrackbar("d", "controls", 100 * self.d, 250, self.nothing)

    def update_pid(self):
        self.p = cv2.getTrackbarPos("p", "controls") / 20
        self.i = cv2.getTrackbarPos("i", "controls") / 100
        self.d = cv2.getTrackbarPos("d", "controls") / 100
        cv2.waitKey(1)
