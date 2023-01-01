import concurrent.futures
import logging
import queue
import threading
import time
import socket
from helpers.writer import Writer
from helpers.reader import Reader

class Drone:
    def __init__(self,host,port,timeout) -> None:
        self.HOST=host
        self.PORT=port
        self.timeout=timeout
        self.SOCKET=None
        while self.SOCKET is None: self.start_server()
        self.Reader=Reader(socket=self.SOCKET)
        self.Writer=Writer(socket=self.SOCKET)
        self.start_threads()
    
    def start_server(self):
        try:
            self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Connecting to server ...")
            self.SOCKET.connect((self.HOST, self.PORT))
            print("Done")
        except socket.error as e:
            self.SOCKET=None
            print("Failed with error: %s" %(e))

    def reader(self, queue, event):
        message="none"
        while not event.is_set():
            logging.info("reader got message: %s", message)                 #thread reader
            queue.put(message)

        logging.info("reader received event. Exiting")

    def writer(self, queue, event):
        while not event.is_set() or not queue.empty():
            message = queue.get()
            logging.info(                                                               #writer theesd
                "writer storing message: %s (size=%d)", message, queue.qsize()
            )

        logging.info("writer received event. Exiting")
    
    def start_threads(self):
        format = "%(asctime)s: %(message)s"
        logging.basicConfig(format=format, level=logging.INFO,
                            datefmt="%H:%M:%S")                                           #main function

        pipeline = queue.Queue(maxsize=10)
        event = threading.Event()
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            executor.submit(self.reader, pipeline, event)
            executor.submit(self.writer, pipeline, event)

            time.sleep(0.1)
            logging.info("Main: about to set event")
            event.set()