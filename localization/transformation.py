import cv2
import numpy as np


class Transformation:
    # Inverting perspective for coordinates (coordinates of marker w.r.t. camera to camera w.r.t. marker)
    def inversePerspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    # Fetches marker coordinates w.r.t. reference point coordinates(origin)
    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)
        info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]
        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))
        return composedRvec, composedTvec
