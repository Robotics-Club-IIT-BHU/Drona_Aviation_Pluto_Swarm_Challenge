import math
import numpy as np
class altitude_cnt:

    def __init__(self,g:float=9.8):
        self.p =0.2
        self.d =0.4
        self.i = 0.0001
        self.control_timestep = 1./125.
        self.GRAVITY = g
        self.reset()

    def reset(self):

        self.last_height_e = 0
        self.integral_height_e = np.zeros(3)
        self.control_counter =0

    def altitude_control(self, target_height, cur_pos, cur_rpy):

        z_err = target_height - cur_pos[2]
        z_err_dot = (z_err - self.last_height_e) / self.control_timestep
        self.last_height_e = z_err_dot
        self.integral_height_e = self.integral_height_e + z_err*self.control_timestep

        thrust = (self.p * z_err + self.d * z_err_dot + self.GRAVITY) / math.cos(cur_rpy[0])*math.cos(cur_rpy[1])

        if thrust < 0.:
            thrust = 0.
        return thrust
    def update(self,cur_pos,cur_rpy,target_pos):
        self.control_counter += 1
        return self.altitude_control(target_pos,cur_pos,cur_rpy)
