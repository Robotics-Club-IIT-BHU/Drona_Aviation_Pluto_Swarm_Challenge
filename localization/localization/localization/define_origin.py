import cv2 as cv
import numpy as np

cam_mtx = np.load('mtx/cam_mtx.npy')
dist_mtx = np.load('mtx/dist_mtx.npy')

cap = cv.VideoCapture(2)
cap.set(3, 3840)
cap.set(4, 2160)
while True:
    _, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    try:
        marker = ids[0]
        corner = corners[0]
        markerLength = 0.045
        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corner, markerLength, cam_mtx, dist_mtx)
        frame = cv.drawFrameAxes(frame, cam_mtx, dist_mtx, rvec, tvec, 0.1)
    except:
        pass
    cv.imshow("frame", frame)
    k = cv.waitKey(1)
    if k==ord('s'):
        try:
            np.save('mtx/o_rvec', rvec)
            np.save('mtx/o_tvec', tvec)
        except:
            pass
    elif k==ord('q'):
        break

cv.destroyallwindows()