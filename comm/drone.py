import threading
import time
import socket
from .helpers.writer import Writer
from .helpers.reader import Reader
from .teleop.key_handling import Data
import sys

MSP_RC = 105
MSP_ATTITUDE=108
MSP_RAW_IMU=102
MSP_ALTITUDE=109
MSP_ANALOG=110

class Drone:
    def __init__(self,host,port,timeout):
        self.HOST=host
        self.PORT=port
        self.timeout=timeout
        self.runThreads=True
        self.socketStable=True
        self.SOCKET=None
        while self.SOCKET is None: self.start_server()
        self.Reader=Reader(socket=self.SOCKET)
        self.Writer=Writer(socket=self.SOCKET)
        self.userRC=[1500,1500,1500,1500,1000,1000,1000,1000]
        self.userRCAP=[1500,1500,1500,1500]
        self.isAutoPilotOn=False
        self.commandType=0
        self.start_threads()
        self.cmd=Data()
    
    def start_server(self):
        count=3
        while(count):
            try:
                self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print("Connecting to server ...")
                self.SOCKET.setblocking(True)
                self.SOCKET.connect((self.HOST, self.PORT))
                if(self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)<0): raise socket.error
                self.SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE,1)
                if(self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)<0): raise socket.error
                if(self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)!=0): raise socket.error
                print("Done")
                break
            except socket.error as e:
                self.SOCKET = None
                count-=1
                time.sleep(2.0)
                print("Failed with error: %s" %(e))
        if not count: exit(0)

    def reader(self):
        while self.runThreads:
            self.Reader.read_frame()

    def writer(self):
        print("Writer thread started...")
        request=[MSP_RC,MSP_ATTITUDE,MSP_RAW_IMU,MSP_ALTITUDE,MSP_ANALOG]
        self.Writer.sendRequestMSP_ACC_TRIM()
        while self.runThreads:
            try:
                acquired = self.lock.acquire(blocking=False)
                if acquired:
                    droneRC=self.userRC

                    if self.isAutoPilotOn and droneRC[7]==1500:
                        droneRC[0] += self.userRCAP[0] - 1500
                        droneRC[1] += self.userRCAP[1] - 1500
                        droneRC[2] += self.userRCAP[2] - 1500
                        droneRC[3] += self.userRCAP[3] - 1500
                                         
                    self.Writer.sendRequestMSP_SET_RAW_RC(droneRC)
                    self.Writer.sendRequestMSP_GET_DEBUG(request)

                    if(self.commandType != 0):
                        self.Writer.sendRequestMSP_SET_COMMAND(commandType)
                        commandType = 0
                    elif(self.commandType != 0 and self.isAutoPilotOn and droneRC[7] == 1500):
                        self.Writer.sendRequestMSP_SET_COMMAND(commandType)
                        commandType = 0
                    
                    self.lock.release()
            except socket.error as e:
                self.socketStable=False
                print("Cannot write to thread: %s" %(e))
            time.sleep(0.022)
        
    
    def sendCommand(self,data):
        self.userRC[0] = data.rcRoll
        self.userRC[1] = data.rcPitch
        self.userRC[2] = data.rcThrottle
        self.userRC[3] = data.rcYaw
        self.userRC[4] = data.rcAUX1
        self.userRC[5] = data.rcAUX2
        self.userRC[6] = data.rcAUX3
        self.userRC[7] = data.rcAUX4
        self.isAutoPilotOn = data.isAutoPilotOn
    
    def command_preprocess(self,arr):
        self.cmd.rcRoll=arr[0]
        self.cmd.rcPitch=arr[1]
        self.cmd.rcThrottle=arr[2]
        self.cmd.rcYaw=arr[3]
        return self.cmd
    
    def start_threads(self):
        self.lock=threading.Lock()
        self.t1 = threading.Thread(target=self.writer)
        self.t2 = threading.Thread(target=self.reader)
        self.t1.start()
        self.t2.start()
    
    def arm(self):
        self.reset()
        self.cmd.rcRoll=1500
        self.cmd.rcYaw=1500
        self.cmd.rcPitch =1500
        self.cmd.rcThrottle =1000
        self.cmd.rcAUX4 =1500
        self.cmd.isAutoPilotOn = 0
        self.sendCommand(self.cmd)
        time.sleep(1)

    def box_arm(self):
        self.cmd.rcRoll=1500
        self.cmd.rcYaw=1500
        self.cmd.rcPitch =1500
        self.cmd.rcThrottle =1500
        self.cmd.rcAUX4 =1500
        self.cmd.isAutoPilotOn = 0
        self.sendCommand(self.cmd)
        time.sleep(0.5)

    def disarm(self):
        self.cmd.rcThrottle =1500
        self.cmd.rcAUX4 = 1000
        self.sendCommand(self.cmd)
        time.sleep(0.5)

    def increase_height(self):
        self.cmd.rcThrottle = 2000

    def decrease_height(self):
        self.cmd.rcThrottle =1300

    def take_off(self):
        self.reset()
        self.disarm()
        self.box_arm()
        self.cmd.commandType = 1
        self.sendCommand(self.cmd)
        # for i in range(6500000):
        #     self.increase_height()
        # self.reset()

    def land(self):
        self.decrease_height()
    
    def reset(self):
        self.cmd.rcRoll =1500
        self.cmd.rcThrottle =1500
        self.cmd.rcPitch =1500
        self.cmd.rcYaw = 1500
        self.cmd.commandType = 0
    
    def getState(self):
        return [
            self.Reader.ATTITUDE["roll"],
            self.Reader.ATTITUDE["pitch"],
            self.Reader.ATTITUDE["yaw"],
        ]
    
    def ok(self):
        return self.runThreads and self.socketStable
    
    def __del__(self):
        self.land()
        self.disarm()
        self.runThreads=False
        time.sleep(0.5)
        self.SOCKET.close()
        time.sleep(0.5)
        print("Drone Connection Ended!!!")