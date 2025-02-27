# 测试抓取和显示图像的多线程

import cv2
import pyrealsense2 as rs
import numpy as np
import socket
from ultralytics import YOLO
import time
import threading
from pymodbus.client import ModbusTcpClient
import queue

# -------------------- 1. 配置YOLO模型 --------------------
model_path = 'yolo11m-seg.pt'  # YOLO模型路径
model = YOLO(model_path)

# -------------------- 2. 配置RealSense相机 --------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# 获取相机内参
def get_camera_intrinsics(color_frame):
    return color_frame.profile.as_video_stream_profile().intrinsics

# -------------------- 3. 连接UR5机械臂 --------------------
def connect_ur5(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    return sock

def control_ur5(sock, script):
    sock.send(script.encode())

def disconnect_ur5(sock):
    sock.close()

# -------------------- 4. 连接机械手 --------------------
# 寄存器字典
regdict = {
    'ID': 1000,
    'baudrate': 1001,
    'clearErr': 1004,
    'forceClb': 1009,
    'angleSet': 1486,
    'forceSet': 1498,
    'speedSet': 1522,
    'angleAct': 1546,
    'forceAct': 1582,
    'errCode': 1606,
    'statusCode': 1612,
    'temp': 1618,
    'actionSeq': 2320,
    'actionRun': 2322
}

# 连接和操作机械手
def connect_hand(ip, port):
    client = ModbusTcpClient(host=ip, port=port)
    client.connect()
    return client

def set_hand_position(client, angle_values):
    client.write_registers(regdict['angleSet'], angle_values)

def set_hand_speed(client, speed_values):
    client.write_registers(regdict['speedSet'], speed_values)

def set_hand_force(client, force_values):
    client.write_registers(regdict['forceSet'], force_values)

def read_register(client, address, count):
    response = client.read_holding_registers(address=address, count=count)
    return response.registers if response.isError() is False else []

def read6(client, reg_name):
    if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
        val = read_register(client, regdict[reg_name], 6)
        if len(val) < 6:
            print('没有读到数据')
            return
        print('读到的值依次为：', end='')
        for v in val:
            print(v, end=' ')
        print()
    elif reg_name in ['errCode', 'statusCode', 'temp']:
        val_act = read_register(client, regdict[reg_name], 3)
        if len(val_act) < 3:
            print('没有读到数据')
            return
        results = []
        for i in range(len(val_act)):
            low_byte = val_act[i] & 0xFF
            high_byte = (val_act[i] >> 8) & 0xFF
            results.append(low_byte)
            results.append(high_byte)
        print('读到的值依次为：', end='')
        for v in results:
            print(v, end=' ')
        print()
    else:
        print('函数调用错误')

def read_hand_position(client):
    print("读取机械手位置...")
    read6(client, 'angleAct')

def read_hand_force(client):
    print("读取机械手力度...")
    read6(client, 'forceAct')

def disconnect_hand(client):
    client.close()

# -------------------- 5. 物体检测与抓取 --------------------
def get_object_position(results, depth_frame, intrinsics):
    boxes = results[0].boxes
    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        class_id = int(box.cls[0].cpu().numpy())
        name = results[0].names[class_id]
        
        # 如果检测到瓶子
        if name == 'bottle':
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2
            depth = depth_frame.get_distance(int(center_x), int(center_y))  # 获取深度值

            # 将像素坐标转为相机坐标系中的三维坐标
            x_cam = (center_x - intrinsics.ppx) * depth / intrinsics.fx
            y_cam = (center_y - intrinsics.ppy) * depth / intrinsics.fy
            z_cam = depth
            return np.array([x_cam, y_cam, z_cam])

    return None

def transform_to_arm_coordinates(camera_position):
    T = np.array([
        [0.1149629, 0.48502391, -0.86691138, 0.76977734],
        [0.99081822, 0.00652201, 0.1350434, -0.57415892],
        [0.07115328, -0.87447657, -0.47982074, 0.39895825],
        [0.0, 0.0, 0.0, 1.0]
    ])

    camera_position_homogeneous = np.append(camera_position, 1)  # 转换为齐次坐标
    transformed_position = np.dot(T, camera_position_homogeneous)

    return transformed_position[:3]  # 返回转换后的位置（x, y, z）

def move_to_object_position(ur5_sock, position):
    script = f"""
def move_to_target():
    global p_target = p[{position[0]}, {position[1]}, {position[2]+0.4}, 0.4, 1.7, -2.]
    movej(p_target, a=1.2, v=0.25)
end
"""
    control_ur5(ur5_sock, script)

def grip_object(hand_client, angle_values):
    set_hand_position(hand_client, angle_values)

# -------------------- 6. 主控制流程 --------------------
def detect_and_display():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        intrinsics = get_camera_intrinsics(color_frame)
        results = model(np.asanyarray(color_frame.get_data()))  # YOLO目标检测

        # 显示检测结果
        for result in results:
            annotated_frame = result.plot()
        cv2.imshow('frame', annotated_frame)

        position = get_object_position(results, depth_frame, intrinsics)
        if position is not None:
            print(f"目标物体位置在相机坐标系: {position}")
            return position  # 返回目标位置供其他线程使用

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def control_arm_and_hand(position, ur5_sock, hand_client):
    if position is not None:
        transformed_position = transform_to_arm_coordinates(position)
        print(f"目标物体位置在机械臂坐标系: {transformed_position}")
        move_to_object_position(ur5_sock, transformed_position)

        # 等待机械臂到达目标位置
        time.sleep(10)

        # 控制机械手抓取物体
        print("抓取物体...")
        grip_object(hand_client, [500, 500, 500, 500, 500, 0])
        time.sleep(2)

# 通过线程并行执行
def main():
    ur5_ip = '192.168.11.125'
    ur5_port = 30003
    hand_ip = '192.168.11.210'
    hand_port = 6000

    ur5_sock = connect_ur5(ur5_ip, ur5_port)
    hand_client = connect_hand(hand_ip, hand_port)

    try:
        position_queue = queue.Queue()

        def detect_thread():
            while True:
                position = detect_and_display()
                if position is not None:
                    position_queue.put(position)

        def control_thread():
            while True:
                if not position_queue.empty():
                    position = position_queue.get()
                    control_arm_and_hand(position, ur5_sock, hand_client)

        # 启动两个线程
        threading.Thread(target=detect_thread, daemon=True).start()
        threading.Thread(target=control_thread, daemon=True).start()

        # 主线程用来保持程序运行
        while True:
            time.sleep(1)

    finally:
        disconnect_ur5(ur5_sock)
        disconnect_hand(hand_client)
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
