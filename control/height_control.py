import math
import numpy as np
import cv2  

class altitude_cnt:

    def __init__(self,g:float=9.8,mode:bool=False):
        self.p = 17.7
        self.d = 0.2
        self.i = 0.0
        self.control_timestep = 1./125.
        self.GRAVITY = g
        self.tracker_mode = mode
        self.reset()
        if(self.tracker_mode): self.tracker_window()

    def reset(self):

        self.last_height_e = 0
        self.integral_height_e = 0
        self.control_counter = 0

    def altitude_control(self, target_height, cur_pos, cur_rpy):
        z_err = target_height - cur_pos[2]
        z_err_dot = (z_err - self.last_height_e) / self.control_timestep
        self.last_height_e = z_err
        self.integral_height_e = self.integral_height_e + z_err*self.control_timestep
        # print(z_err, z_err_dot, self.last_height_e, self.integral_height_e)
        # thrust = (self.p * z_err + self.d * z_err_dot + self.GRAVITY) / math.cos(cur_rpy[0])*math.cos(cur_rpy[1])
        thrust = (self.p * z_err + self.d * z_err_dot + self.i * self.integral_height_e + self.GRAVITY)
        print('p: ', self.p * z_err)
        print('diff: ', self.d * z_err_dot)
        print('thrust: ', thrust)
        thrust = thrust / 2  + 1700
        print('After thrust- ', thrust)
        if thrust < 1200:
            thrust = 1200
        if thrust > 2000:
            thrust = 2000 
        return [1500,1500,thrust,1500]

    def update(self,cur_pos,cur_rpy,target_pos):
        self.control_counter += 1
        if self.tracker_mode: self.update_pid()
        return self.altitude_control(target_pos,cur_pos,cur_rpy)
    
    def kill(self):
        return [1500,1500,1300,1500]
    
    def __del__(self):
        cv2.destroyAllWindows()
        return True

    def nothing(self,x):
        pass
    
    def tracker_window(self):
        cv2.namedWindow('controls')
        cv2.createTrackbar('p','controls',170,500,self.nothing)
        cv2.createTrackbar('i','controls',0,250,self.nothing)
        cv2.createTrackbar('d','controls',0,250,self.nothing)
    
    def update_pid(self):
        self.p=cv2.getTrackbarPos('p','controls')/20
        self.i=cv2.getTrackbarPos('i','controls')/100
        self.d=cv2.getTrackbarPos('d','controls')/100
        cv2.waitKey(1)
