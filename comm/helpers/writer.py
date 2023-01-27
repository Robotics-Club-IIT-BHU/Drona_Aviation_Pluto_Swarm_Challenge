import numpy as np

class Writer:
    def __init__(self,socket):
        self.socket=socket
        self.MSP_HEADER="$M<"
        #default value of MSP packets
        self.MSP_SET_COMMAND = 217
        self.MSP_SET_RAW_RC=200
        self.MSP_ACC_TRIM=240
    
    #function for package creation for a given payload
    def createPacketMSP(self,msp,payload):
        bf=[]
        for k in self.MSP_HEADER:
            bf.append(k)
        check_sum=0
        if len(payload)==0:
            pl_size=0& 0xFF
        else:
            pl_size=len(payload) & 0xFF                         
        bf.append(pl_size)
        
        #checking packet integrity
        #calculating by XOR operation one bita t a time
        check_sum^=(pl_size & 0xFF)

        bf.append(msp & 0xFF)
        check_sum ^= (msp & 0xFF)

        if len(payload)!=0:
            for k in payload:
                bf.append(k&0xFF)
                check_sum^=k&0xFF
        bf.append(check_sum)
        return(bf)

    #sending request for packet creation
    def sendRequestMSP_SET_COMMAND(self,commandType):                                                  
        payload=[commandType&0xff,0,0,0,0,0,0,0]
        self.sendRequestMSP(self.createPacketMSP(self.MSP_SET_COMMAND,payload))

    #sending request for packet creation
    def sendRequestMSP_SET_RAW_RC(self,channels):
        index=0
        rc_signals=[i for i in range(16)]
        for i in range(0,8,1):
            rc_signals[index]=channels[i]&0xff
            index+=1                                                        
            rc_signals[index]=(channels[i]>>8)& 0xff
            index+=1
        self.sendRequestMSP(self.createPacketMSP(self.MSP_SET_RAW_RC,rc_signals))

    def sendRequestMSP(self,data):
        arr=bytearray()
        for d in data:
            if isinstance(d, str):
                arr.extend(np.uint8(bytearray(d.encode("utf-8"))[0]))
            else:
                arr.append(np.uint8(d))
        self.socket.send(arr)

    #sending request for packet creation
    def sendRequestMSP_ACC_TRIM(self):
        self.sendRequestMSP(self.createPacketMSP(self.MSP_ACC_TRIM, []))

    def sendRequestMSP_GET_DEBUG(self,requests):
        for k in requests:
            self.sendRequestMSP(self.createPacketMSP(k, []))
