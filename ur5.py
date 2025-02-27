# ur5.py 定义了一些用于ur5机械臂控制的函数

import socket
import time

def connect_ur5(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    return sock

def send_script(sock, script):
    sock.send(script.encode())

def disconnect_ur5(sock):
    sock.close()

def set_TCP_pos(ur5_sock, position):
    script = f"""
def move_to_target():
    global p_target = p[{position[0]}, {position[1]}, {position[2]}, 1.13, 1.04, -1.4]
    movej(p_target, a=1.2, v=0.25)
end
"""
    send_script(ur5_sock, script)



def ur5_reset():
    ur5_ip = '192.168.11.125'
    ur5_port = 30003

    # 连接UR5机械臂和机械手
    ur5_sock = connect_ur5(ur5_ip, ur5_port)

    script_reset = f"""
def reset():
    global p_start = p[-0.076, -.223, 0.591, 0.047, 1.89, -2.06]
    movej(  p_start, a=1.2, v=0.25)
end
"""
    
    time.sleep(5)  # 等待UR5机械臂执行完前一个动作
    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        send_script(ur5_sock, script_reset)

    finally:
        disconnect_ur5(ur5_sock)

def handeye_pos():
    ur5_ip = '192.168.11.125'
    ur5_port = 30003

    # 连接UR5机械臂和机械手
    ur5_sock = connect_ur5(ur5_ip, ur5_port)

    script_reset = f"""
def reset():
    global p_start = p[0.271, -0.512, 0.388, 1.72, -2.928, 2.471]
    movej(  p_start, a=1.2, v=0.25)
end
"""

    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        send_script(ur5_sock, script_reset)

        time.sleep(10)  # 等待UR5机械臂复位完成

    finally:
        disconnect_ur5(ur5_sock)


def install_TCP_pos():
    ur5_ip = '192.168.11.125'
    ur5_port = 30003

    # 连接UR5机械臂和机械手
    ur5_sock = connect_ur5(ur5_ip, ur5_port)

    script_reset = f"""
def reset():
    global p_start = p[0.571, 0.008, 0.36, 2.09, -1.3, 4.45]
    movej(  p_start, a=1.2, v=0.25)
end
"""

    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        send_script(ur5_sock, script_reset)

        time.sleep(10)  # 等待UR5机械臂复位完成

    finally:
        disconnect_ur5(ur5_sock)