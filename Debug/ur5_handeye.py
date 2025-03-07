import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import MODEL_PATH, UR5_IP, UR5_PORT, HAND_IP, HAND_PORT
from ur5 import UR5Controller
from inspire_hand import InspireHand
import socket
import time

def connect_ur5(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    return sock

def control_ur5(sock, script):
    sock.send(script.encode())

def disconnect_ur5(sock):
    sock.close()

def main():

    ur5 = UR5Controller(UR5_IP, UR5_PORT)
    ur5.connect()
    ur5.reset()
    # ur5.reset("handeye")
    ur5.disconnect()

if __name__ == "__main__":
    main()