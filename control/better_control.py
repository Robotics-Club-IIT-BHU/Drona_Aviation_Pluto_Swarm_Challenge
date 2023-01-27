#import libraries
import math
import cv2
import time


class altitude_cnt:

    def __init__(self,tracker_mode:bool=False):

        '''
        Parameters
            tracker_mode : bool, optional
                Set true to adjust PID Coefficients using trackbars
        '''
        # Initialize PID controller coefficients
        self.kp = 4.0
        self.kd = 0.0
        self.ki = 0.0
        self.imax = 400.0
        self.freq = 20
        self.lastderivative=None
        self.lasterror=0
        self.scaler=2
        self.last_time = self.current_time()
        self.rpy_ret=1500
        self.tracker_mode=tracker_mode
        # If true opens tracker window
        if(self.tracker_mode): self.tracker_window()

    # Get the current time in milliseconds
    def current_time(self): return round(time.time()*1000)

    def reset_I(self):
        '''
        Resets the integral term
        '''
        self.integrator=0
        self.lastderivative=None

    def update(self, cur_pos, cur_rpy, target_height):
        '''
        Parameters
            cur_pos : A numpy array consisting current position of drone
            cur_rpy : A numpy array consisting current Roll,Pitch,Yaw of drone
            target_height : required height of drone
        '''
        # Time Update
        tnow=self.current_time()
        dt=tnow-self.last_time
        if self.last_time==0 or dt>1000:
            dt=0
            self.reset_I()
        self.last_time=tnow
        delta_time=float(dt)*0.001

        # Proportional
        error = target_height - cur_pos[2]
        P=error*self.kp
        output=P

        # Derivative
        if dt>0:
            derivative=None
            if self.lastderivative is None:
                derivative=0
                self.lastderivative=0
            else:
                derivative=(error-self.lasterror)/delta_time

            # Filter
            RC=1/(2*math.pi*self.freq)
            derivative=self.lastderivative+((delta_time/(delta_time+RC))*(derivative-self.lastderivative))

            # Update state
            self.lasterror=error
            self.lastderivative=derivative

            # Add derivative
            D=self.kd*derivative
            output+=D

        # Scale P and D components
        output*=self.scaler

        # Compute integral
        if dt>0:
            self.integrator=(error*self.ki)*self.scaler*delta_time
            if self.integrator < -self.imax: self.integrator=-self.imax
            elif self.integrator > self.imax: self.integrator=self.imax

            I=self.integrator
            output+=I

        output+=1700
        # Crop output
        if output < 1300: output = 1300
        if output > 2100: output = 2100
        if self.tracker_mode: cv2.waitKey(1)
        print("Throttle=",output)
        return [self.rpy_ret,self.rpy_ret,output,self.rpy_ret]
    # Update PID coefficients
    def update_p(self,x): self.kp=x/20
    def update_i(self,x): self.ki=x/100
    def update_d(self,x): self.kd=x/100

    # Trackbar window to tune PID coefficients
    def tracker_window(self):
        cv2.namedWindow('controls')
        cv2.createTrackbar('p','controls',170,500,self.update_p)
        cv2.createTrackbar('i','controls',0,250,self.update_i)
        cv2.createTrackbar('d','controls',0,250,self.update_d)

    def kill(self):
        return [self.rpy_ret,self.rpy_ret,1300,self.rpy_ret]

    def __del__(self):
        if self.tracker_mode: cv2.destroyAllWindows()
