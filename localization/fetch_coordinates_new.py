import numpy as np
import cv2
import cv2.aruco as aruco
import os

dir = os.path.dirname(__file__)


class poseEstimation:
    def __init__(self, frame):
        self.frame = frame
        self.cam_matrix = np.load(dir + "/camera_matrix_1920.npy")
        self.distortion_coefficients = np.load(
            dir + "/distortion_coefficients_1920.npy"
        )
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.wind = 1
        self.ind = 0
        self.sum = np.zeros((3,))
        self.readings = np.zeros((self.wind, 3))
        self.originx = 0
        self.originy = 0
        self.flag = 1

    def fetch(self, height, frame):
        self.frame = frame
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_parameters
        )
        img = aruco.drawDetectedMarkers(self.frame, corners, borderColor=(0, 0, 255))
        cv2.imshow("img", img)
        cv2.waitKey(100)

        if len(corners) > 0:
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
                corners[0], 0.045, self.cam_matrix, self.distortion_coefficients
            )
            rvec = np.reshape(rvec, (3, 1))
            tvec = np.reshape(tvec, (3, 1))
            rotmat, _ = cv2.Rodrigues(rvec)

            """
            print(rotmat.shape)
            print(tvec.shape)
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
            """
            # print('rot', rotmat)
            # print('tvec', tvec)
            # print('rvec', rvec)
            tvec = np.reshape(tvec, (3,))
            self.sum = self.sum - self.readings[self.ind]
            self.readings[self.ind] = tvec
            self.sum = self.sum + tvec
            self.ind = (self.ind + 1) % self.wind
            filt_tvec = self.sum / self.wind
            if self.flag and self.ind == 0:
                self.originx = filt_tvec[0]
                self.originy = filt_tvec[1]
                self.flag = 0
            else:
                filt_tvec[0] -= self.originx
                filt_tvec[1] -= self.originy
            print("tvec", filt_tvec)

            # tf_m2c = np.copy(rotmat)
            # tf_m2c = np.hstack((tf_m2c, tvec))
            # tf_m2c = np.vstack((tf_m2c, [0, 0, 0, 1]))    # transformation matrix: camera with respect to marker

            # tf_c2m = np.transpose(rotmat)

            # ts = -np.transpose(rotmat)
            # ts = np.matmul(ts, tvec)   # coordinates of marker with respect to the camera

            # tf_c2m = np.hstack((tf_c2m, ts))

            # tf_c2m = np.vstack((tf_c2m, [0, 0, 0, 1]))  # transformation matrix: marker with respect to camera

            # return ts
