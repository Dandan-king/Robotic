# 用于检测瓶子及输出瓶子在base下坐标


import cv2
import time
import threading
import numpy as np
from config import MODEL_PATH
from model import initialize_model
from camera import initialize_camera, get_camera_intrinsics, stop_camera
from utils import transform_to_arm_coordinates

def get_object_position(results, depth_frame, intrinsics):
    boxes = results[0].boxes
    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        class_id = int(box.cls[0].cpu().numpy())
        name = results[0].names[class_id]
        
        if name == 'bottle':
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2
            depth = depth_frame.get_distance(int(center_x), int(center_y))

            x_cam = (center_x - intrinsics.ppx) * depth / intrinsics.fx
            y_cam = (center_y - intrinsics.ppy) * depth / intrinsics.fy
            z_cam = depth
            return np.array([x_cam, y_cam, z_cam])

    return None

def display_thread(pipeline, model, stop_event):
    last_time = time.time()
    while not stop_event.is_set():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        intrinsics = get_camera_intrinsics(color_frame)
        results = model(np.asanyarray(color_frame.get_data()))

        position = get_object_position(results, depth_frame, intrinsics)
        if position is not None and time.time() - last_time >= 1:
            print(f"目标物体位置在相机坐标系: {position}")
            transformed_position = transform_to_arm_coordinates(position)
            print(f"目标物体位置在机械臂坐标系: {transformed_position}")
            last_time = time.time()

        annotated_frame = np.asanyarray(color_frame.get_data())
        cv2.imshow('frame', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

def main():
    pipeline = initialize_camera()
    model = initialize_model(MODEL_PATH)

    stop_event = threading.Event()

    try:
        threading.Thread(target=display_thread, args=(pipeline, model, stop_event), daemon=True).start()

        while not stop_event.is_set():
            time.sleep(1)

    finally:
        stop_camera(pipeline)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()