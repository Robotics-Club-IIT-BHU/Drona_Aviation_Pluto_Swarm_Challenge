import pickle

class Writer:
    def __init__(self,socket):
        self.socket=socket
        self.MSP_HEADER="$M<"
        self.MSP_SET_COMMAND = 217
        self.MSP_SET_RAW_RC=200
   
    def createPacketMSP(self,msp,payload):
        bf=[]
        for k in self.MSP_HEADER:
            bf.append(int(k&(0xFF)))
        check_sum=0
        if payload.empty():
            pl_size=0& 0xFF
        else:
            pl_size=len(payload) & 0xFF                         #package creation
        bf.append(pl_size)
        check_sum^=(msp & 0xFF)

        if not payload.empty():
            for k in payload:
                bf.append(k&0xff)
                check_sum^=k&0xff
        bf.append(check_sum)
        return(bf)

    def sendRequestMSP_SET_COMMAND(self,commandType):                                                    #set command
        payload=[commandType&0xff]
        self.sendRequestMSP(self.createPacketMSP(self.MSP_SET_COMMAND,payload))

    def sendRequestMSP_SET_RAW_RC(self,channels):
        index=0
        rc_signals=[]
        for i in range(0,8,1):
            rc_signals[index]=channels[i]&0xff
            index+=1                                                        #packet ki values 
            rc_signals[index]=(channels[i]>>8)& 0xff
            index+=1
        self.sendRequestMSP(self.createPacketMSP(self.MSP_SET_RAW_RC,rc_signals))

    def sendRequestMSP(self,data):
        y=pickle.dumps(data)
        self.socket.send(y)    











# if __name__ == '__main__':
#     while 1:
#         my_server()