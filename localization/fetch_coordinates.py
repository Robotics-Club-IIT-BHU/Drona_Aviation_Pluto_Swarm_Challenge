import numpy as np
import cv2
import sys
import time
import cv2.aruco as aruco

class poseEstimation:

    def __init__(self, frame):
        self.frame = frame
        self.cam_matrix = np.load('cam_matrix.npy')
        self.distortion_coefficients = np.load('distortion_coefficients.npy')
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
        self.aruco_parameters = aruco.DetectorParameters_create()

    def fetch(self, height):
    
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(gray,
                                                                self.aruco_dict,
                                                                parameters=self.aruco_parameters,
                                                                cameraMatrix=self.cam_matrix,
                                                                distCoeff=self.distortion_coefficients)
    
        if len(corners) > 0:
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[0], 0.02, self.cam_matrix,
                                                                       self.distortion_coefficients)
            imgx = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
            imgy = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4

            imgCoordsVec = np.array([[imgx],
                                     [imgy],
                                     1])

            camCoordsVec = np.dot(np.linalg.inv(self.cam_matrix), imgCoordsVec)
            camCoordsVec = camCoordsVec * height

            camCoordsVec = camCoordsVec - tvec
            worldCoordsVec = np.dot(np.linalg.inv(rvec), camCoordsVec)

        return worldCoordsVec