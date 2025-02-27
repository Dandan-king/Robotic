import cv2
import time
import threading
import queue
import numpy as np
from config import MODEL_PATH, UR5_IP, UR5_PORT, HAND_IP, HAND_PORT
from model import initialize_model
from camera import initialize_camera, get_camera_intrinsics, stop_camera
from ur5 import connect_ur5, set_TCP_pos, disconnect_ur5, ur5_reset
from inspire_hand import connect_hand, set_hand_position, disconnect_hand
from utils import transform_to_arm_coordinates

# 创建一个队列用于存储最近的多帧深度数据
DEPTH_HISTORY_SIZE = 5
depth_history_queue = queue.Queue(maxsize=DEPTH_HISTORY_SIZE)

def get_object_position(results, depth_frame, intrinsics, sampling_area_size=5):
    """
    获取物体位置并进行深度平滑和深度平均计算。
    :param results: YOLO 检测结果
    :param depth_frame: 深度图像
    :param intrinsics: 相机内参
    :param sampling_area_size: 采样区域大小（例如5x5像素点）
    :return: 物体在相机坐标系中的三维坐标
    """
    boxes = results[0].boxes
    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        class_id = int(box.cls[0].cpu().numpy())
        name = results[0].names[class_id]
        
        if name == 'bottle':
            # 获取检测框的中心位置
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2

            # 采样区域的大小为 sampling_area_size，选择该区域内多个点
            depth_values = []
            for i in range(-sampling_area_size//2, sampling_area_size//2 + 1):
                for j in range(-sampling_area_size//2, sampling_area_size//2 + 1):
                    # 获取当前采样点的深度值
                    x_sample = int(center_x + i)
                    y_sample = int(center_y + j)
                    depth = depth_frame.get_distance(x_sample, y_sample)
                    depth_values.append(depth)

            # 计算采样点的平均深度值
            smoothed_depth = np.mean(depth_values)

            # 将当前深度数据添加到队列中
            if depth_history_queue.full():
                depth_history_queue.get()  # 移除最旧的一帧
            depth_history_queue.put(smoothed_depth)  # 添加最新的深度数据

            # 获取队列中的深度数据并计算平均值
            depth_values = list(depth_history_queue.queue)
            smoothed_depth = np.mean(depth_values)

            # 使用平滑后的深度值进行坐标计算
            x_cam = (center_x - intrinsics.ppx) * smoothed_depth / intrinsics.fx
            y_cam = (center_y - intrinsics.ppy) * smoothed_depth / intrinsics.fy
            z_cam = smoothed_depth
            return np.array([x_cam, y_cam, z_cam])

    return None

def display_thread(model, pipeline, stop_event):
    while not stop_event.is_set():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        results = model(np.asanyarray(color_frame.get_data()))
        for result in results:
            annotated_frame = result.plot()  # 绘制检测结果
        cv2.imshow('frame', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

def detect_and_control_thread(model, pipeline, ur5_sock, hand_client, start_event, stop_event, move_event):
    set_hand_position(hand_client, [1000, 1000, 1000, 1000, 1000, 400])
    while not stop_event.is_set():
        start_event.wait()  # 等待开始信号
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        intrinsics = get_camera_intrinsics(color_frame)
        results = model(np.asanyarray(color_frame.get_data()))

        position = get_object_position(results, depth_frame, intrinsics)
        if position is not None:
            print(f"目标物体位置在相机坐标系: {position}")
            transformed_position = transform_to_arm_coordinates(position)
            print(f"目标物体位置在机械臂坐标系: {transformed_position}")

            move_event.wait()  # 等待移动信号
            set_TCP_pos(ur5_sock, transformed_position)

            time.sleep(10)

            print("抓取物体...")
            set_hand_position(hand_client, [400, 400, 400, 400, 400, 400])
            time.sleep(2)

def main():
    pipeline = initialize_camera()
    model = initialize_model(MODEL_PATH)
    ur5_sock = connect_ur5(UR5_IP, UR5_PORT)
    hand_client = connect_hand(HAND_IP, HAND_PORT)

    stop_event = threading.Event()
    start_event = threading.Event()
    move_event = threading.Event()

    try:
        threading.Thread(target=display_thread, args=(model, pipeline, stop_event), daemon=True).start()
        threading.Thread(target=detect_and_control_thread, args=(model, pipeline, ur5_sock, hand_client, start_event, stop_event, move_event), daemon=True).start()

        while not stop_event.is_set():
            command = input("输入 'start' 开始检测物体，输入 'move' 移动机械臂，输入 'stop' 结束程序: ").strip().lower()
            if command == 'start':
                start_event.set()
            elif command == 'move':
                move_event.set()
            elif command == 'stop':
                stop_event.set()

    finally:
        stop_camera(pipeline)
        disconnect_ur5(ur5_sock)
        disconnect_hand(hand_client)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
