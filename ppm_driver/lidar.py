import threading
import time
import socket
import matplotlib.pyplot as plt
import numpy as np


""" Class to communicate with lidar """


class Lidar:
    def __init__(
        self, host, port, timeout, plotter: bool = False, reference_line: float = 0
    ):
        self.HOST = host
        self.PORT = port
        self.timeout = timeout
        self.runThreads = True
        self.SOCKET = None
        self.Distance = 0
        while self.SOCKET is None:
            self.start_server()
        self.plotter = plotter
        if (
            self.plotter
        ):  # If plotter is true then reference_line is passed to initiate it
            self.start_plotter(reference_line)
        self.start_threads()

    # Function to connect to socket ( Retries for 3 times )
    def start_server(self):
        count = 3
        while count:
            try:
                self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print("Connecting to lidar ...")
                self.SOCKET.setblocking(True)
                self.SOCKET.connect((self.HOST, self.PORT))
                if self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE) < 0:
                    raise socket.error
                self.SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                if self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE) < 0:
                    raise socket.error
                if self.SOCKET.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR) != 0:
                    raise socket.error
                print("Done")
                break
            except socket.error as e:  # If socket connection fails printing error and retrying after 2 sec
                self.SOCKET = None
                count -= 1
                time.sleep(2.0)
                print("Failed with error: %s" % (e))

        # If connection failed ending the whole code
        if not count:
            exit(0)

    # Read n(=2) bytes from socket
    def readnbyte(self):
        n = 2
        buff = bytearray(n)
        pos = 0
        while pos < n:
            cr = self.SOCKET.recv_into(memoryview(buff)[pos:])
            if cr == 0:
                raise EOFError
            pos += cr
        return bytes(buff)

    # Converting byte data to integer and storing in Distance variable
    def read_frame(self):
        message = self.readnbyte()
        self.Distance = int.from_bytes(message, byteorder="big", signed=False)

    # Running threads until class instance is not deleted
    def reader(self):
        while self.runThreads:
            self.read_frame()

    # Show plotter function ----> NEEDED TO BE CALLED IF PLOTTER IS SET TRUE
    def show_plotter(self):
        self.y = np.delete(self.y, 0)
        self.y = np.append(self.y, [self.Distance])
        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.y)
        self.figure.gca().relim()
        self.figure.gca().autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.savefig("Plot.jpg")

    # Thread to continuously read data from lidar
    def start_threads(self):
        self.t1 = threading.Thread(target=self.reader)
        self.t1.start()

    # Function to close all threads and sockets after class instance is deleted
    def __del__(self):
        plt.close("all")
        self.runThreads = False
        time.sleep(0.5)
        self.SOCKET.close()
        time.sleep(0.5)
        print("Lidar Connection Ended!!!")

    # Function to start plotter if plotter is set true
    def start_plotter(self, value):
        self.x = np.linspace(0, 10, 100)
        self.y = np.zeros(100)
        self.reference = np.array([value for _ in range(100)])
        plt.ion()
        self.figure, ax = plt.subplots(figsize=(6, 4))
        (self.line1,) = ax.plot(self.x, self.y, "b-", label="Height")
        (self.line2,) = ax.plot(self.x, self.reference, "r-", label="Reference")
        plt.title("Height Plotter", fontsize=20)
        plt.xlabel("Instance")
        plt.ylabel("Height")


""" Code to test the lidar """
if __name__ == "__main__":
    lidar = Lidar("192.168.248.125", 8888, 1, plotter=True, reference_line=50)
    while True:
        print(lidar.Distance)
        lidar.show_plotter()
        time.sleep(0.01)
