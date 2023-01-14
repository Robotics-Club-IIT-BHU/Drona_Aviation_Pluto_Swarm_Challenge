import math
import numpy as np
class altitude_cnt:

    def __init__(self,g:float=9.8):
        self.p = 20
        self.d = 15
        self.i = 0.001
        self.control_timestep = 1./125.
        self.GRAVITY = g
        self.reset()

    def reset(self):

        self.last_height_e = 0
        self.integral_height_e = np.zeros(3)
        self.control_counter =0

    def altitude_control(self, target_height, cur_pos, cur_rpy):
        # print(type(target_height))
        # print(type(cur_pos))

        z_err = target_height - cur_pos[2]
        z_err_dot = (z_err - self.last_height_e) / self.control_timestep
        self.last_height_e = z_err
        self.integral_height_e = self.integral_height_e + z_err*self.control_timestep
        print(z_err, z_err_dot, self.last_height_e, self.integral_height_e)
        # thrust = (self.p * z_err + self.d * z_err_dot + self.GRAVITY) / math.cos(cur_rpy[0])*math.cos(cur_rpy[1])
        thrust = (self.p * z_err + self.d * z_err_dot + self.GRAVITY)
        print('thrust: ', thrust)
        thrust = thrust / 2  + 1500
        if thrust < 900.:
            thrust = 900.
        if thrust > 2000:
            thrust = 2000 
        return [1500,1500,thrust,1500]

    def update(self,cur_pos,cur_rpy,target_pos):
        self.control_counter += 1
        return self.altitude_control(target_pos,cur_pos,cur_rpy)
    
    def kill(self):
        return True
    
    def __del__(self):
        return True
