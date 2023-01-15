import threading
import time
import socket
import matplotlib.pyplot as plt
import numpy as np

class Lidar:
    def __init__(self,host,port,timeout,plotter:bool=False,reference_line:float=0):
        self.HOST=host
        self.PORT=port
        self.timeout=timeout
        self.runThreads=True
        self.SOCKET=None
        self.Distance=0
        while self.SOCKET is None: self.start_server()
        self.plotter=plotter
        if(self.plotter): self.start_plotter(reference_line)
        self.start_threads()
    
    def start_server(self):
        count=3
        while(count):
            try:
                self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print("Connecting to lidar ...")
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

    def readnbyte(self):
      n=2
      buff = bytearray(n)
      pos = 0
      while pos < n:
          cr = self.SOCKET.recv_into(memoryview(buff)[pos:])
          if cr == 0:
              raise EOFError
          pos += cr
      return bytes(buff)
    
    def read_frame(self):
        message=self.readnbyte()
        self.Distance =int(message[0]<<0)|int(message[1]<<8)

    def reader(self):
        while self.runThreads:
            self.read_frame()
    
    def show_plotter(self):
            self.y=np.delete(self.y,0)
            self.y=np.append(self.y, [self.Distance])
            self.line1.set_xdata(self.x)
            self.line1.set_ydata(self.y)
            self.figure.gca().relim()
            self.figure.gca().autoscale_view()
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()


    def start_threads(self):
        self.t1 = threading.Thread(target=self.reader)
        self.t1.start()
    
    def __del__(self):
        plt.close('all')
        self.runThreads=False
        time.sleep(0.5)
        self.SOCKET.close()
        time.sleep(0.5)
        print("Lidar Connection Ended!!!")
    
    def start_plotter(self,value):
        self.x = np.linspace(0, 10, 100)
        self.y = np.zeros(100)
        self.reference = np.array([value for _ in range(100)])
        plt.ion()
        self.figure, ax = plt.subplots(figsize=(6, 4))
        self.line1, = ax.plot(self.x, self.y,'b-',label="Height")
        self.line2, = ax.plot(self.x, self.reference,'r-',label="Reference")
        plt.title("Height Plotter", fontsize=20)
        plt.xlabel("Instance")
        plt.ylabel("Height")


if __name__=="__main__":
    lidar=Lidar("192.168.4.124",8888,1,plotter=True,reference_line=50)
    while True:
        print(lidar.Distance)
        lidar.show_plotter()
        time.sleep(0.5)