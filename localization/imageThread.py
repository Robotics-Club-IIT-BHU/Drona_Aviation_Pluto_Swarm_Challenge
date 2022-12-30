import cv2
import threading
import time
class ImageFeed:

    def __init__(self,source,sleep_rate=0.7):
        self.cap = source
        self.lock = threading.Lock()
        self.running = False
        self.grabbed = False
        self.sleep_rate = sleep_rate
        self.frame_rate = 15

    def __refresh(self,flag):
        while self.running:
            self.lock.acquire()
            try:
                self.prev = self.cap()
                if not self.grabbed: self.grabbed = True
            finally:
                self.lock.release()
            time.sleep(self.sleep_rate/self.frame_rate) ## This is very neccesary so that the thread doesnt consume the whole cpu time
        return True

    def read(self):
        self.lock.acquire()
        try:
            if not self.grabbed:
                self.grabbed=True
                self.prev = self.cap()
        finally:
            self.lock.release()
        return self.prev

    def connect(self):
        self.running = True
        refresh_loop = threading.Thread(target= self.__refresh, args=(1,))
        refresh_loop.start()
        
    def close(self):
        self.running = False

    def setter(self,propid, value):
        retval = False
        if self.running:
            print("The camera is being used in the loop can't change the settings\n Try closing the connection and trying again!")
        else:
            self.lock.acquire()
            try:
                retval = self.cap.set(propid,value)
                if not retval:
                    print("failed to set the following attribute to "+str(value)+" !")
            finally:
                self.lock.release()
        return retval

    def __del__(self):
        self.close()
