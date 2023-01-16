import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import time

class poseEstimation:

    def __init__(self, frame):
        self.frame = frame
        self.cam_matrix = np.load('/home/ankur/Documents/Robo/interiit/Drona-Aviation-Inter-IIT-2023/localization/camera_matrix.npy')
        self.distortion_coefficients = np.load('/home/ankur/Documents/Robo/interiit/Drona-Aviation-Inter-IIT-2023/localization/distortion_coefficients.npy')
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_parameters = aruco.DetectorParameters_create()

    def fetch(self, height,frame):
        self.frame=frame
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(gray,
                                                                self.aruco_dict,
                                                                parameters=self.aruco_parameters)
        img = aruco.drawDetectedMarkers(self.frame, corners, borderColor=(0, 0, 255))
        cv2.imshow('img', img)
        cv2.waitKey(100)

        if len(corners) > 0:
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[0], 0.45, self.cam_matrix,
                                                                       self.distortion_coefficients)
            rvec = np.reshape(rvec, (3,1))
            tvec = np.reshape(tvec, (3,1))
            rotmat, _ = cv2.Rodrigues(rvec)
            # print(rotmat.shape)
            # print(tvec.shape)
            imgx = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
            imgy = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4
            # print(imgx, imgy)
            imgCoordsVec = np.array([imgx,
                                     imgy,
                                     1])
            imgCoordsVec = np.reshape(imgCoordsVec, (3, 1))
            # print(imgCoordsVec.shape)
            camCoordsVec = np.dot(np.linalg.inv(self.cam_matrix), imgCoordsVec)
            # print(camCoordsVec.shape)
            camCoordsVec = camCoordsVec * ((280-height)/100)
            # print(camCoordsVec.shape)
            camCoordsVec = camCoordsVec - tvec
            # print(camCoordsVec.shape)
            # print(rotmat.shape)
            worldCoordsVec = np.dot(np.linalg.inv(rotmat), camCoordsVec)

            return worldCoordsVec