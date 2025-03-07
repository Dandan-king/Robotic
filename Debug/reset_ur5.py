# 用于复位ur5到某个位置，现已并入到ur5.py中

import socket
import time


# -------------------- Reset UR5机械臂 --------------------
def connect_ur5(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    return sock

def control_ur5(sock, script):
    sock.send(script.encode())

def disconnect_ur5(sock):
    sock.close()

# -------------------- 主控制流程 --------------------
def reset():
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

    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        control_ur5(ur5_sock, script_reset)

        time.sleep(10)  # 等待UR5机械臂复位完成

    finally:
        disconnect_ur5(ur5_sock)

def handeye_pos():
    ur5_ip = '192.168.11.125'
    ur5_port = 30003

    # 连接UR5机械臂和机械手
    ur5_sock = connect_ur5(ur5_ip, ur5_port)

    script_reset = f"""
def reset():
    global p_start = p[0.165, -0.568, 0.373, 1.17, -2.8, 2.4]
    movej(  p_start, a=1.2, v=0.25)
end
"""

    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        control_ur5(ur5_sock, script_reset)

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
    global p_start = p[-0.1, -0.12, 0.6, 0.2, 1.9, -2.3]
    movej(  p_start, a=1.2, v=0.25)
end
"""

    try:
        # 控制UR5机械臂复位
        print("控制UR5机械臂复位...")
        control_ur5(ur5_sock, script_reset)

        time.sleep(10)  # 等待UR5机械臂复位完成

    finally:
        disconnect_ur5(ur5_sock)



def main():
    reset()
    # handeye_pos()
    # install_TCP_pos()

if __name__ == "__main__":
    main()
