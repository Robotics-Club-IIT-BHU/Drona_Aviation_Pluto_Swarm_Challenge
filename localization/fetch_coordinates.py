import os
import cv2 as cv
import numpy as np
from .transformation import Transformation
from .regression import Regression

dir = os. getcwd()
class poseEstimation:
    def __init__(self):
        self.cam_mtx = np.load(dir+"/localization/mtx/cam_mtx.npy")
        self.dist_mtx = np.load(dir+"/localization/mtx/dist_mtx.npy")
        self.o_rvec = np.load(dir+"/localization/mtx/o_rvec.npy")
        self.o_tvec = np.load(dir+"/localization/mtx/o_tvec.npy")
        self.tf = Transformation()
        self.rg = Regression()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.aruco_parameters = cv.aruco.DetectorParameters_create()

    # Returns marker's real world coordinates
    def fetch(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
        try:
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.045, self.cam_mtx, self.dist_mtx)
            p_rvec, p_tvec = self.tf.relativePosition(self.rvec, self.tvec, self.o_rvec, self.o_tvec)
            p_tvec = p_tvec.flatten()
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0].reshape((4, 2))
            corner = np.array([topLeft, topRight, bottomRight, bottomLeft])
            corner = corner.flatten()
            data = np.concatenate((p_tvec, corner), axis=-1)
            data = np.expand_dims(data, axis=0)
            pos = self.rg.linear_regression(data)
            return pos
        except:
            pass
