import os
import cv2 as cv
import numpy as np
from .transformation import Transformation
from .regression import Regression
import matplotlib.pyplot as plt
import numpy as np

dir = os. getcwd()
class poseEstimation:
    def __init__(self,plotter: bool = False, reference_line: float = 0):
        self.cam_mtx = np.load(dir+"/localization/mtx/cam_mtx.npy")
        self.dist_mtx = np.load(dir+"/localization/mtx/dist_mtx.npy")
        self.o_rvec = np.load(dir+"/localization/mtx/o_rvec.npy")
        self.o_tvec = np.load(dir+"/localization/mtx/o_tvec.npy")
        self.tf = Transformation()
        self.rg = Regression()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.aruco_parameters = cv.aruco.DetectorParameters_create()
        self.plotter = plotter
        self.Distance = 0
        if (
            self.plotter
        ):  # If plotter is true then reference_line is passed to initiate it
            self.start_plotter(reference_line)
        

    # Returns marker's real world coordinates
    def fetch(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
        img = cv.aruco.drawDetectedMarkers(frame, corners, borderColor=(0, 0, 255))
        img=cv.resize(img, [900,474])
        cv.imshow("Frame",img)
        cv.waitKey(10)
        try:
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.045, self.cam_mtx, self.dist_mtx)
            p_rvec, p_tvec = self.tf.relativePosition(rvec, tvec, self.o_rvec, self.o_tvec)
            p_tvec = p_tvec.flatten()
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0].reshape((4, 2))
            corner = np.array([topLeft, topRight, bottomRight, bottomLeft])
            corner = corner.flatten()
            data = np.concatenate((p_tvec, corner), axis=-1)
            data = np.expand_dims(data, axis=0)
            pos = self.rg.linear_regression(data)
            self.Distance = int(pos[2]*100)
            return pos
        except:
            pass

    # Show plotter function ----> NEEDED TO BE CALLED IF PLOTTER IS SET TRUE
    def show_plotter(self):
        self.y = np.delete(self.y, 0)
        self.y = np.append(self.y, [self.Distance])
        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.y)
        self.figure.gca().relim()
        self.figure.gca().autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.savefig("Plot.jpg")
    
    # Function to start plotter if plotter is set true
    def start_plotter(self, value):
        self.x = np.linspace(0, 10, 100)
        self.y = np.zeros(100)
        self.reference = np.array([value for _ in range(100)])
        plt.ion()
        self.figure, ax = plt.subplots(figsize=(6, 4))
        (self.line1,) = ax.plot(self.x, self.y, "b-", label="Height")
        (self.line2,) = ax.plot(self.x, self.reference, "r-", label="Reference")
        plt.title("Height Plotter", fontsize=20)
        plt.xlabel("Instance")
        plt.ylabel("Height")
