##Drona-Aviation-Inter-IIT-2023/comm/helpers/reader.py
import numpy as np

IDLE = 0

# details of the MSP packet
HEADER_START = 1
HEADER_M = (2,)
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6

# assignment of codes to MSP packets-constant values
MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217


class Reader:
    def __init__(self, socket):
        # initialising the values of parameters of MSP packets
        self.socket = socket
        self.inputBuffer = bytearray(1024)
        self.bufferIndex = 0
        self.c_state = IDLE
        self.offset = 0
        self.err_rcvd = False
        self.dataSize = 0
        self.checksum = 0
        self.cmd = 0

        """Global variables of recieved data"""
        # initialising the values of drone parameters
        self.IMU_DATA = {"acc": 0, "gyro": 0, "mag": 0}
        self.ATTITUDE = {"roll": 0, "pitch": 0, "yaw": 0}
        self.ALTITUDE = 0
        self.ANALOG = {"battery": 0, "rssi": 0}
        self.ACC_TRIM = {"trim_pitch": 0, "trim_roll": 0}
        self.RC = {
            "rcRoll": 0,
            "rcPitch": 0,
            "rcYaw": 0,
            "rcThrottle": 0,
            "rcAUX1": 0,
            "rcAUX2": 0,
            "rcAUX3": 0,
            "rcAUX4": 0,
        }

    # function to read and return n bytes form socket
    def readnbyte(self):
        n = 1
        buff = bytearray(n)
        pos = 0
        while pos < n:
            cr = self.socket.recv_into(memoryview(buff)[pos:])
            if cr == 0:
                raise EOFError
            pos += cr
        return bytes(buff)

    # reading messages
    def read_frame(self):

        message = self.readnbyte()
        c = np.uint8(message[0])
        # Comparing current msg with last state and updating current state (c_state) accordingly
        if self.c_state == IDLE:
            self.c_state = (
                HEADER_START
                if (c == np.uint8(bytearray(("$").encode("utf-8"))[0]))
                else IDLE
            )  # 36
        elif self.c_state == HEADER_START:
            self.c_state = (
                HEADER_M
                if (c == np.uint8(bytearray(("M").encode("utf-8"))[0]))
                else IDLE
            )  # 77
        elif self.c_state == HEADER_M:
            if c == np.uint8(bytearray((">").encode("utf-8"))[0]):  # 62
                self.c_state = HEADER_ARROW
            elif c == np.uint8(bytearray(("!").encode("utf-8"))[0]):  # 33
                self.c_state = HEADER_ERR
            else:
                self.c_state = IDLE
        elif self.c_state == HEADER_ARROW or self.c_state == HEADER_ERR:
            self.err_rcvd = (
                self.c_state == HEADER_ERR
            )  # checking for an error message received
            self.dataSize = c & 0xFF
            self.offset = 0
            self.checksum = 0
            self.checksum ^= c & 0xFF
            self.c_state = HEADER_SIZE
        elif self.c_state == HEADER_SIZE:
            self.cmd = c & 0xFF
            self.checksum ^= c & 0xFF
            self.c_state = HEADER_CMD
        elif self.c_state == HEADER_CMD and self.offset < self.dataSize:
            self.checksum ^= c & 0xFF  # adding data to input buffer
            self.inputBuffer[self.offset] = c & 0xFF
            self.offset += 1
        elif self.c_state == HEADER_CMD and self.offset >= self.dataSize:
            if (self.checksum & 0xFF) == (c & 0xFF):
                if not self.err_rcvd:
                    self.bufferIndex = 0
                    self.evaluateCommand()
            # then we again assign the value as IDLE for the next MSP packet
            self.c_state = IDLE

    # function to decode msg from input buffer according to command recieved
    def evaluateCommand(self):
        if self.cmd == MSP_FC_VERSION:
            self.FC_versionMajor = self.read8()
            self.FC_versionMinor = self.read8()
            self.FC_versionPatchLevel = self.read8()

        elif self.cmd == MSP_RAW_IMU:
            # accelerometer
            accX = self.read16()
            accY = self.read16()
            accZ = self.read16()
            self.IMU_DATA["acc"] = [accX, accY, accZ]

            # gyroscope
            gyroX = self.read16() / 8
            gyroY = self.read16() / 8
            gyroZ = self.read16() / 8
            self.IMU_DATA["gyro"] = [gyroX, gyroY, gyroZ]

            # magnetometer
            magX = self.read16() / 3
            magY = self.read16() / 3
            magZ = self.read16() / 3
            self.IMU_DATA["mag"] = [magX, magY, magZ]

        elif self.cmd == MSP_ATTITUDE:
            # reading the value roll pitch yaw
            self.ATTITUDE["roll"] = self.read16() / 10
            self.ATTITUDE["pitch"] = self.read16() / 10
            self.ATTITUDE["yaw"] = self.read16()

        elif self.cmd == MSP_ALTITUDE:
            # reading the values of altitude
            self.ALTITUDE = (self.read32() / 10) - 0

        elif self.cmd == MSP_ANALOG:
            # reading the analog values like battery and rssi
            self.ANALOG["battery"] = self.read8() / 10.0
            self.ANALOG["rssi"] = self.read16()

        elif self.cmd == MSP_ACC_TRIM:
            self.ACC_TRIM["trim_pitch"] = self.read16()
            self.ACC_TRIM["trim_roll"] = self.read16()

        elif self.cmd == MSP_RC:
            self.RC["rcRoll"] = self.read16()
            self.RC["rcPitch"] = self.read16()
            self.RC["rcYaw"] = self.read16()
            self.RC["rcThrottle"] = self.read16()
            self.RC["rcAUX1"] = self.read16()
            self.RC["rcAUX2"] = self.read16()
            self.RC["rcAUX3"] = self.read16()
            self.RC["rcAUX4"] = self.read16()
        else:
            pass

    # reading an 8-bit input
    def read8(self):
        c = self.inputBuffer[self.bufferIndex] & 0xFF
        self.bufferIndex += 1
        return c

    # reading an 16-bit input (as implemented in read8())
    def read16(self):

        add_1 = self.inputBuffer[self.bufferIndex] & 0xFF
        self.bufferIndex += 1
        add_2 = (self.inputBuffer[self.bufferIndex]) << 8
        self.bufferIndex += 1

        return add_1 + add_2

    # reading an 16-bit input (as implemented in read8() and read16())
    def read32(self):
        add_1 = self.inputBuffer[self.bufferIndex] & 0xFF
        self.bufferIndex += 1
        add_2 = (self.inputBuffer[self.bufferIndex] & 0xFF) << 8
        self.bufferIndex += 1
        add_3 = (self.inputBuffer[self.bufferIndex] & 0xFF) << 16
        self.bufferIndex += 1
        add_4 = (self.inputBuffer[self.bufferIndex] & 0xFF) << 24
        self.bufferIndex += 1
        return add_1 + add_2 + add_3 + add_4
