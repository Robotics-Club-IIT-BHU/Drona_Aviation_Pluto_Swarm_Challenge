import cv2
import threading
import time


class ImageServer:
    def __init__(self, source, sleep_rate, frame_rate):
        self.cap = source
        self.lock = threading.Lock()
        self.running = False
        self.grabbed = False
        self.sleep_rate = sleep_rate
        self.frame_rate = frame_rate
        self.prev = None
        self.vid = cv2.VideoCapture(self.cap)
        self.vid.set(3, 640)
        self.vid.set(4, 480)
        self.vid.set(cv2.CAP_PROP_EXPOSURE, 10)
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # self.out = cv2.VideoWriter('camera_recording.avi', fourcc, 20.0, (1920, 1080))

    def __refresh(self, flag):
        while self.running:
            self.lock.acquire()
            try:
                self.prev = self.capture()
                # self.out.write(self.prev)
                if not self.grabbed:
                    self.grabbed = True
            finally:
                self.lock.release()
            time.sleep(
                self.sleep_rate / self.frame_rate
            )  ## This is very neccesary so that the thread doesnt consume the whole cpu time
        return True

    def read(self):
        self.lock.acquire()
        try:
            if not self.grabbed:
                self.grabbed = True
                self.prev = self.capture()
        finally:
            self.lock.release()
        return self.prev

    def connect(self):
        self.running = True
        refresh_loop = threading.Thread(target=self.__refresh, args=(1,))
        refresh_loop.start()

    def capture(self):
        ret, img = self.vid.read()
        return img

    def close(self):
        self.running = False

    def setter(self, propid, value):
        retval = False
        if self.running:
            print(
                "The camera is being used in the loop can't change the settings\n Try closing the connection and trying again!"
            )
        else:
            self.lock.acquire()
            try:
                retval = self.cap.set(propid, value)
                if not retval:
                    print(
                        "failed to set the following attribute to " + str(value) + " !"
                    )
            finally:
                self.lock.release()
        return retval

    def __del__(self):
        self.close()
