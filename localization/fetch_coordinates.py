import cv2 as cv
import numpy as np
from transformation import Transformation
from regression import Regression


class poseEstimation:
    def __init__(self, frame, height):
        self.frame = frame
        self.cam_mtx = np.load("mtx/cam_matrix.npy")
        self.dist_mtx = np.load("mtx/dist_mtx.npy")
        self.o_rvec = np.load("mtx/o_rvec.npy")
        self.o_tvec = np.load("mtx/o_tvec.npy")
        self.tf = Transformation()
        self.rg = Regression()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.height = height
        # self.cam_height = 100 (in cm)

    # Returns marker's real world coordinates
    def fetch(self):
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_parameters,
            cameraMatrix=self.cam_mtx,
            distCoeff=self.dist_mtx,
        )
        try:
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.045, cam_mtx, dist_mtx)
            frame = cv.drawFrameAxes(frame, cam_mtx, dist_mtx, rvec, tvec, 0.1)
            p_rvec, p_tvec = tf.relativePosition(rvec, tvec, o_rvec, o_tvec)
            p_tvec = p_tvec.flatten()
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0].reshape((4, 2))
            corner = np.array([topLeft, topRight, bottomRight, bottomLeft])
            corner = corner.flatten()
            data = np.concatenate((p_tvec, corner), axis=-1)
            data = np.expand_dims(data, axis=0)
            pos = rg.linear_regression(data)
            return pos
        except:
            pass
