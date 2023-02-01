import cv2 as cv
import numpy as np
from transformation import Transformation
from regression import Regression

class poseEstimation:
    def __init__(self, frame, height):
        self.frame = frame
        self.cam_mtx = np.load('mtx/cam_matrix.npy')
        self.dist_mtx = np.load('mtx/dist_mtx.npy')
        self.o_rvec = np.load('mtx/o_rvec.npy')
        self.o_tvec = np.load('mtx/o_tvec.npy')
        self.tf = Transformation()
        self.rg = Regression()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.height = height
        # self.cam_height = 100 (in cm)
        
    #Returns marker's real world coordinates
    def fetch(self):
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(gray,
                                                                self.aruco_dict,
                                                                parameters=self.aruco_parameters,
                                                                cameraMatrix=self.cam_mtx,
                                                                distCoeff=self.dist_mtx)
        try:
            imgx = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
            imgy = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4
            # realsense_height = self.cam_height - depth_frame[imgy, imgx]/10 (in cm)
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.045, self.cam_mtx, self.dist_mtx)
            p_rvec, p_tvec = self.tf.relativePosition(rvec, tvec, self.o_rvec, self.o_tvec)
            p_tvec = self.rg.simple_regression(p_tvec)
            return p_tvec
            # return p_tvec, realsense_height
        except:
            pass
