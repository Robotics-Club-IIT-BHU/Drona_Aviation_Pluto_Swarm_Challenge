import threading
import time
import socket
from helpers.writer import Writer
from helpers.reader import Reader

class Drone:
    def __init__(self,host,port,timeout):
        self.HOST=host
        self.PORT=port
        self.timeout=timeout
        self.SOCKET=None
        while self.SOCKET is None: self.start_server()
        self.Reader=Reader(socket=self.SOCKET)
        self.Writer=Writer(socket=self.SOCKET)
        self.userRC=[1500,1500,1500,1500,1000,1000,1000,1000]
        self.isAutoPilotOn=False
        self.commandType=0
        self.start_threads()
    
    def start_server(self):
        try:
            self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Connecting to server ...")
            self.SOCKET.connect((self.HOST, self.PORT))
            print("Done")
        except socket.error as e:
            self.SOCKET = None
            print("Failed with error: %s" %(e))

    

    def reader(self):
        while True:
            print("Running")
            self.Reader.read_frame()

    def writer(self):
        print("Writer thread started...")
        while True:
            droneRC=self.userRC
            self.Writer.sendRequestMSP_SET_RAW_RC(droneRC)
            if(self.commandType != 0):
                self.Writer.sendRequestMSP_SET_COMMAND(commandType)
                commandType = 0
            time.sleep(0.02)
        
    
    def process_msg(self,data):
        self.userRC[0] = data.rcRoll
        self.userRC[1] = data.rcPitch
        self.userRC[2] = data.rcThrottle
        self.userRC[3] = data.rcYaw
        self.userRC[4] = data.rcAUX1
        self.userRC[5] = data.rcAUX2
        self.userRC[6] = data.rcAUX3
        self.userRC[7] = data.rcAUX4
        self.isAutoPilotOn = data.isAutoPilotOn
    
    def start_threads(self):
        # format = "%(asctime)s: %(message)s"
        # logging.basicConfig(format=format, level=logging.INFO,
        #                     datefmt="%H:%M:%S")                                           #main function

        # event = threading.Event()
        # with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        #     executor.submit(self.reader, event)
        #     executor.submit(self.writer, event)

        #     time.sleep(0.1)
        #     logging.info("Main: about to set event")
        #     # event.set()
        t1 = threading.Thread(target=self.writer)
        t2 = threading.Thread(target=self.reader)
        t1.start()
        t2.start()