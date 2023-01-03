IDLE = 0
HEADER_START = 1
HEADER_M = 2, 
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6


MSP_FC_VERSION=3
MSP_RAW_IMU=102
MSP_RC = 105
MSP_ATTITUDE=108
MSP_ALTITUDE=109
MSP_ANALOG=110
MSP_SET_RAW_RC=200
MSP_ACC_CALIBRATION=205
MSP_MAG_CALIBRATION=206                 #constant values
MSP_SET_MOTOR=214
MSP_SET_ACC_TRIM=239
MSP_ACC_TRIM=240
MSP_EEPROM_WRITE = 250
MSP_SET_POS= 216
MSP_SET_COMMAND = 217

inputBuffer=[None]*1024
recbuf=[None]*1024

def read8():
  c=inputBuffer[bufferIndex] & 0xff
  bufferIndex+=1
  return c

def read16():

   add_1=(inputBuffer[bufferIndex] & 0xff) 
   bufferIndex+=1
   add_2=((inputBuffer[bufferIndex]) << 8)
   bufferIndex+=1

   return (add_1+add_2)


def read32():
  add_1=(inputBuffer[bufferIndex] & 0xff)
  bufferIndex+=1
  add_2=((inputBuffer[bufferIndex] & 0xff) << 8)
  bufferIndex+=1
  add_3=((inputBuffer[bufferIndex] & 0xff) << 16)
  bufferIndex+=1
  add_4=((inputBuffer[bufferIndex] & 0xff) << 24)
  bufferIndex+=1
  return (add_1+add_2+add_3+add_4)



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
        if command==MSP_FC_VERSION:
        FC_versionMajor=read8()
        FC_versionMinor=read8()
        FC_versionPatchLevel=read8()
    elif command==MSP_RAW_IMU:
        accX=read16()
        accY=read16()
        accZ=read16()

        gyroX=read16()/8
        gyroY=read16()/8
        gyroZ=read16()/8

        magX=read16()/3
        magY=read16()/3
        magZ=read16()/3
    elif command==MSP_ATTITUDE:
        roll=(read16()/10)
        pitch=(read16()/10)
        yaw=read16()
    elif command==MSP_ALTITUDE:
        alt=(read32()/10)-0
    elif command==MSP_ANALOG:
        battery=(read8()/10.0)
        rssi=read16()

    elif command==MSP_ACC_TRIM:
        trim_pitch=read16()
        trim_roll=read16()
    elif command==MSP_RC:
        rcRoll = read16()
        rcPitch = read16()
        rcYaw = read16()
        rcThrottle = read16()
        rcAUX1 = read16()
        rcAUX2 = read16()
        rcAUX3 = read16()
        rcAUX4 = read16()
    else:
        pass
        
