import math
import socket
import signal
import sys
from threading import Thread, Event
from queue import Queue
import numpy as np

from my_math.rotation_matrixes import rot_yaw, rot_pitch, rot_roll

UDP_IP = "127.0.0.1"
UDP_PORT = 9999
ANGLE=math.radians(180)

class Lidar_Server(Thread):
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        # daemon=True: se il main thread muore, questo thread non blocca l'uscita
        super().__init__(daemon=True)
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.my_queue = Queue()
        self._stop_event = Event()

        # Intercetta CTRL-C / tasto Stop di PyCharm (SIGINT) e terminazione (SIGTERM)
        # direttamente da dentro la classe!
        signal.signal(signal.SIGINT, self._handle_exit)
        signal.signal(signal.SIGTERM, self._handle_exit)

    def _handle_exit(self, signum, frame):
        self._stop_event.set()
        self.close_socket()
        sys.exit(0) # Fa terminare anche il main thread in modo pulito

    def run(self):
        try:
            self.init_socket()
            self.loop()
        finally:
            self.close_socket()

    def init_socket(self):
        self.sock.bind((self.ip, self.port))
        # Usa settimeout invece di setblocking(False) per evitare BlockingIOError
        self.sock.settimeout(0.5)

    def loop(self):
        while not self._stop_event.is_set():
            try:
                #print("Ricevuto")
                data, _ = self.sock.recvfrom(65535)
                lidar_pts = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
                #print(len(lidar_pts))
                rotated_points = lidar_pts@rot_yaw(ANGLE)@rot_roll(ANGLE)
                self.my_queue.put(rotated_points)
            except socket.timeout:
                continue
            except OSError:
                break

    def close_socket(self):
        if self.sock.fileno() != -1:
            self.sock.close()
            print("Socket UDP chiuso.")



