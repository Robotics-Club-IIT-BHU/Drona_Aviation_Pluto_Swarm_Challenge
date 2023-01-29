import cv2 as cv
import numpy as np
from transformation import Transformation
from regression import Regression

cam_mtx = np.load('mtx/cam_mtx.npy')
dist_mtx = np.load('mtx/dist_mtx.npy')
o_rvec = np.load('mtx/o_rvec.npy')
o_tvec = np.load('mtx/o_tvec.npy')
tf = Transformation()
rg = Regression()

cap = cv.VideoCapture(0)
cap.set(3, 3840)
cap.set(4, 2160)
while True:
    _, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    try:
        corner = corners[0]
        markerLength = 0.045
        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corner, markerLength, cam_mtx, dist_mtx)
        frame = cv.drawFrameAxes(frame, cam_mtx, dist_mtx, rvec, tvec, 0.1)
        p_rvec, p_tvec = tf.relativePosition(rvec, tvec, o_rvec, o_tvec)
        pos = rg.simple_regression(p_tvec*100)
        print("x:", pos[0], " ", "y:", pos[1])
    except:
        pass
    cv.imshow("frame", frame)
    k = cv.waitKey(1)
    if k==ord('q'):
        break

cv.destroyallwindows()