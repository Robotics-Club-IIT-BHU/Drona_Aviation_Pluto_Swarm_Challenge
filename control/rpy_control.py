import math
import numpy as np
import cv2  

class rpy_cnt:

    def __init__(self,g:float=9.8):
        self.p = np.array([1,1,1])
        self.d = np.array([1,1,1])
        self.i = np.array([1,1,1])
        self.control_timestep = 1./125.
        self.GRAVITY = g
        # self.tracker_mode=mode
        self.reset()
        # if(self.tracker_mode): self.tracker_window()

    def reset(self):

        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)
        self.control_counter =0

    def altitude_control(self, target_rpy, cur_rpy):
        # print(target_rpy,cur_pos[2])
        # print()

        rpy_err = target_rpy - cur_rpy
        rpy_err_dot = (rpy_err - self.last_rpy_e) / self.control_timestep
        self.last_rpy_e = rpy_err
        self.integral_rpy_e = self.integral_rpy_e + rpy_err*self.control_timestep
        # print(rpy_err, rpy_err_dot, self.last_rpy_e, self.integral_rpy_e)
        # thrust = (self.p * rpy_err + self.d * rpy_err_dot + self.GRAVITY) / math.cos(cur_rpy[0])*math.cos(cur_rpy[1])
        # thrust = (self.p * rpy_err + self.d * rpy_err_dot + self.i * self.integral_rpy_e + self.GRAVITY)
        reqd_rpy = np.multiply(self.p, rpy_err) + np.multiply(self.d, rpy_err_dot) + np.multiply(self.i, self.integral_rpy_e)
        # print('p: ', self.p * rpy_err)
        # print('diff: ', self.d * rpy_err_dot)
        # print('thrust: ', thrust)
        reqd_rpy = reqd_rpy / 2  + 1600
        # print('After thrust- ', thrust)
        if reqd_rpy < [1400, 1400, 1400]:
            reqd_rpy = [1400, 1400, 1400]
        if reqd_rpy > [2000, 2000, 2000]:
            reqd_rpy = [2000, 2000, 2000] 
        return [reqd_rpy[0], reqd_rpy[1], 0, reqd_rpy[2]]

    def update(self ,target_rpy, cur_rpy):
        self.control_counter += 1
        # if self.tracker_mode: self.update_pid()
        return self.altitude_control(target_rpy,cur_rpy)
    
    def kill(self):
        return True
    
    def __del__(self):
        # cv2.destroyAllWindows()
        return True