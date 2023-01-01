IDLE = 0
HEADER_START = 1
HEADER_M = 2, 
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6

class Reader:
    def __init__(self,socket):
        self.socket=socket
        self.inputBuffer=[]
        self.bufferIndex=0
        self.c_state = IDLE
        self.offset=0
        self.err_rcvd = False
        self.dataSize = 0
        self.checksum = 0
        self.cmd = None
        self.IMU_DATA={"acc":[],"gyro":[],"mag":[]}
        self.ATTITUDE={"roll":0,"pitch":0,"yaw":0}
        self.ALTITUDE=0
        self.MSP_ANALOG={"battery":0,"rssi":0}
        self.ACC_TRIM={"trim_pitch":0,"trim_roll":0}
        self.MSP_RC={"rcRoll":0,"rcPitch":0,"rcYaw":0,"rcThrottle":0,"rcAUX1":0,"rcAUX2":0,"rcAUX3":0,"rcAUX4":0}

    def read_frame(self):
        message=self.socket.recv(1024)
        c=0
        if message!="":
            message=message.decode()
            c=message[0]
        if (self.c_state == IDLE):
          self.c_state =  HEADER_START if (c == '$') else IDLE
        elif (self.c_state == HEADER_START):
          self.c_state = HEADER_M if (c == 'M') else IDLE
        elif (self.c_state == HEADER_M):
          if (c == '>'):
            self.c_state = HEADER_ARROW
          elif (c == '!'):
            self.c_state = HEADER_ERR
          else:
            self.c_state = IDLE
        elif (self.c_state == HEADER_ARROW or self.c_state == HEADER_ERR):
          self.err_rcvd = (self.c_state == HEADER_ERR)
          self.dataSize = (c & 0xFF)
          self.offset = 0
          self.checksum = 0
          self.checksum ^= (c & 0xFF)
          self.c_state = HEADER_SIZE
        elif (self.c_state == HEADER_SIZE):
          self.cmd = (int) (c & 0xFF)
          self.checksum ^= (c & 0xFF)
          self.c_state = HEADER_CMD
        elif (self.c_state == HEADER_CMD and self.offset < self.dataSize):
          self.checksum ^= (c & 0xFF)
          self.inputBuffer[self.offset] = (int) (c & 0xFF)
          self.offset+=1
        elif (self.c_state == HEADER_CMD and self.offset >= self.dataSize):
          if ((self.checksum & 0xFF) == (c & 0xFF)):
            if not self.err_rcvd:
              self.bufferIndex=0
              self.evaluateCommand()
          self.c_state = IDLE
    
    def evaluateCommand(self):
        pass