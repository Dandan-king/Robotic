"""
主控制程序

功能：
- 初始化摄像头、YOLO模型、UR5机械臂和灵巧手
- 管理共享数据和线程控制
- 启动GUI并初始化所有线程
- 处理物体检测结果和跟踪逻辑
- 控制机械臂和灵巧手的动作
- 清理资源

类：
- PositionFilter: 用于平滑物体位置
- MainController: 主控制类，包含主要功能和线程管理

使用示例：
>>> controller = MainController()
>>> controller.start_gui()
"""

import cv2
import time
import threading
import numpy as np
from tkinter import Tk
from config import MODEL_PATH, UR5_IP, UR5_PORT, HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE
from model import initialize_model
from camera import Camera
from ur5 import UR5Controller
from inspire_hand import InspireHand
from utils import transform_to_arm_coordinates
from gui import ControlGUI

class PositionFilter:
    """
    用于平滑物体位置的类
    """
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.position_history = []
        
    def update(self, new_position):
        """
        更新位置并返回平滑后的位置
        
        :param new_position: 新的位置
        :return: 平滑后的位置
        """
        if len(self.position_history) > 0:
            velocity = (new_position - self.position_history[-1]) * 0.2
            new_position += velocity
            
        self.position_history.append(new_position.copy())
        if len(self.position_history) > self.window_size:
            self.position_history.pop(0)
        return np.mean(self.position_history, axis=0)

class MainController:
    """
    主控制类，包含主要功能和线程管理
    """
    def __init__(self):
        self.initialize_camera()
        self.initialize_model()
        self.initialize_ur5()
        self.initialize_hand()
        self.initialize_shared_data()
        self.initialize_threads()

    def initialize_camera(self):
        """初始化摄像头"""
        try:
            self.camera = Camera()
            print("摄像头初始化成功")
        except Exception as e:
            print(f"摄像头初始化失败: {str(e)}")
            self.camera = None

    def initialize_model(self):
        """初始化YOLO模型"""
        try:
            self.model = initialize_model(MODEL_PATH)
            print("YOLO模型加载成功")
        except Exception as e:
            print(f"YOLO模型加载失败: {str(e)}")
            self.model = None

    def initialize_ur5(self):
        """初始化机械臂"""
        try:
            self.ur5_controller = UR5Controller(UR5_IP, UR5_PORT)
            self.ur5_controller.connect()
            print("机械臂连接成功")
        except Exception as e:
            print(f"机械臂连接失败: {str(e)}")
            self.ur5_controller = None

    def initialize_hand(self):
        """初始化机械手"""
        try:
            self.hand_client = InspireHand(HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE)
            self.hand_client.connect()
            print("机械手连接成功")
        except Exception as e:
            print(f"机械手连接失败: {str(e)}")
            self.hand_client = None

    def initialize_shared_data(self):
        """初始化共享数据"""
        self.detected_objects = []
        self.current_frame = None
        self.selected_object = None
        self.frame_lock = threading.Lock()
        self.object_list_lock = threading.Lock()
        self.last_object_ids = set()
        self.gui_update_interval = 0.5
        self.last_gui_update = 0
        self.frozen_object_list = []
        self.stop_event = threading.Event()
        self.gui_ready_event = threading.Event()
        self.gui = None
        self.tracked_objects = {}
        self.next_track_id = 0
        self.stability_threshold = 5
        self.position_filters = {}
        self.max_lost_frames = 3
        self.min_valid_samples_ratio = 0.3
        self.depth_valid_range = (300, 2000)

    def initialize_threads(self):
        """初始化线程"""
        threading.Thread(target=self.detection_thread, daemon=True).start()
        threading.Thread(target=self.control_thread, daemon=True).start()
        threading.Thread(target=self.gui_update_thread, daemon=True).start()

    def start_gui(self):
        """启动 GUI 界面并初始化所有线程"""
        if not self.camera or not self.model or not self.ur5_controller or not self.hand_client:
            print("初始化失败，无法启动GUI")
            return

        root = Tk()
        self.gui = ControlGUI(root)
        self.gui.object_list.bind('<<ListboxSelect>>', self.on_object_select)
        root.protocol("WM_DELETE_WINDOW", self.cleanup)
        self.gui_ready_event.set()
        root.mainloop()

    def on_object_select(self, event):
        """当用户选择列表项时冻结当前列表"""
        with self.object_list_lock:
            self.frozen_object_list = [obj.copy() for obj in self.detected_objects]

    def gui_update_thread(self):
        """GUI 更新线程"""
        while not self.stop_event.is_set():
            try:
                self.update_gui()
                time.sleep(0.05)
            except Exception as e:
                print(f"GUI更新错误: {str(e)}")

    def update_gui(self):
        """更新GUI界面"""
        now = time.time()
        if now - self.last_gui_update < self.gui_update_interval:
            time.sleep(0.1)
            return

        if self.current_frame is not None:
            frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            self.gui.update_video(frame)

        with self.object_list_lock:
            display_objects = self.frozen_object_list if self.frozen_object_list else self.detected_objects
            stable_objects, current_ids = self.get_stable_objects(display_objects)
            preserved_ids = current_ids & self.last_object_ids
            self.last_object_ids = current_ids

        self.gui.update_object_list([
            {
                "id": obj["id"],
                "name": obj["name"],
                "position": obj["position_str"]
            }
            for obj in stable_objects if obj["id"] in preserved_ids
        ])

        if self.selected_object is not None:
            cam_str, arm_str = self.get_coordinates_str(self.selected_object)
            self.gui.update_coordinates(cam_str, arm_str)

        self.last_gui_update = now

    def get_stable_objects(self, display_objects):
        """获取稳定的物体列表"""
        stable_objects = []
        current_ids = set()
        for obj in display_objects:
            arm_pos = obj.get("arm_position")
            if isinstance(arm_pos, (list, np.ndarray)) and len(arm_pos) == 3:
                pos_str = f"({arm_pos[0]:.1f}, {arm_pos[1]:.1f}, {arm_pos[2]:.1f})"
            else:
                pos_str = "(Invalid Position)"
                arm_pos = [0, 0, 0]

            obj_id = f"{obj['name']}-{hash(tuple(arm_pos))[:6]}"
            current_ids.add(obj_id)
            stable_objects.append({
                "id": obj_id,
                "name": obj["name"],
                "position_str": pos_str,
                "raw_position": arm_pos
            })
        return stable_objects, current_ids

    def get_coordinates_str(self, selected_object):
        """获取坐标字符串"""
        cam_coord = selected_object.get("position", [0, 0, 0])
        arm_coord = selected_object.get("arm_position", [0, 0, 0])

        if not (isinstance(cam_coord, (list, np.ndarray)) and len(cam_coord) == 3):
            cam_coord = [0, 0, 0]
        cam_str = f"Camera: ({cam_coord[0]:.1f}, {cam_coord[1]:.1f}, {cam_coord[2]:.1f})"

        if isinstance(arm_coord, np.ndarray):
            arm_coord = arm_coord.tolist()
        if not (isinstance(arm_coord, list) and len(arm_coord) == 3):
            arm_coord = [0, 0, 0]
        arm_str = f"Arm: ({arm_coord[0]:.1f}, {arm_coord[1]:.1f}, {arm_coord[2]:.1f})"

        return cam_str, arm_str

    def detection_thread(self):
        """物体检测线程"""
        self.gui_ready_event.wait()
        self.hand_client.set_positions([1000] * 5 + [400])
        
        while not self.stop_event.is_set():
            try:
                frames = self.camera.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                    
                color_image = np.asanyarray(color_frame.get_data())
                with self.frame_lock:
                    self.current_frame = color_image.copy()
                
                if self.gui.detection_running:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    results = self.model(color_image)
                    self.process_detection_results(results, depth_image, color_frame)
                    
            except Exception as e:
                print(f"检测线程错误: {str(e)}")

    def get_object_position(self, box, depth_frame, intrinsics, sampling_area_size=7):
        """
        获取物体位置并进行深度平滑和深度平均计算。
        
        :param box: YOLO 检测到的单个物体的边界框
        :param depth_frame: 深度图像帧
        :param intrinsics: 相机内参
        :param sampling_area_size: 采样区域大小（例如5x5像素点）
        :return: 物体在相机坐标系中的三维坐标
        """
        xyxy = box.xyxy[0].cpu().numpy()
        center_x = (xyxy[0] + xyxy[2]) / 2
        center_y = (xyxy[1] + xyxy[3]) / 2

        depth_values = []
        valid_samples = 0
        for i in range(-sampling_area_size//2, sampling_area_size//2+1):
            for j in range(-sampling_area_size//2, sampling_area_size//2+1):
                x_sample = int(center_x + i)
                y_sample = int(center_y + j)
                if 0 <= x_sample < depth_frame.shape[1] and 0 <= y_sample < depth_frame.shape[0]:
                    depth = depth_frame[y_sample, x_sample] / 1000.0
                    if depth > 0:
                        depth_values.append(depth)
                        valid_samples += 1

        if valid_samples < sampling_area_size**2 * 0.3:
            return None
        
        smoothed_depth = np.median(depth_values)
        x_cam = (center_x - intrinsics.ppx) * smoothed_depth / intrinsics.fx
        y_cam = (center_y - intrinsics.ppy) * smoothed_depth / intrinsics.fy
        z_cam = smoothed_depth

        return np.array([x_cam, y_cam, z_cam])
    
    def process_detection_results(self, results, depth_frame, color_frame):
        """处理物体检测结果并进行跟踪"""
        intrinsics = self.camera.get_intrinsics(color_frame)
        current_objects = []

        for box in results[0].boxes:
            class_id = int(box.cls[0].cpu().numpy())
            name = self.model.names[class_id]
            position = self.get_object_position(box, depth_frame, intrinsics)
            
            if position is not None:
                current_objects.append({
                    "name": name,
                    "position": position,
                    "arm_position": transform_to_arm_coordinates(position),
                    "bbox": box.xyxy[0].cpu().numpy().tolist()
                })

        updated_tracks, used_track_ids = self.update_tracks(current_objects)
        self.clean_lost_tracks(updated_tracks, used_track_ids)
        self.apply_position_filter()
        self.detected_objects = self.get_stable_objects_list()

        print(f"[跟踪统计] 总跟踪数:{len(self.tracked_objects)} 稳定对象:{len(self.detected_objects)}")

    def update_tracks(self, current_objects):
        """更新跟踪对象"""
        updated_tracks = {}
        used_track_ids = set()

        current_objects.sort(
            key=lambda x: (x["bbox"][2]-x["bbox"][0])*(x["bbox"][3]-x["bbox"][1]),
            reverse=True
        )

        for obj in current_objects:
            best_match_id, best_iou = self.find_best_match(obj)

            if best_match_id is not None:
                updated_tracks[best_match_id] = {
                    "obj": obj,
                    "stable_count": self.tracked_objects[best_match_id]["stable_count"] + 1,
                    "lost_count": 0
                }
                used_track_ids.add(best_match_id)
            else:
                updated_tracks[self.next_track_id] = {
                    "obj": obj,
                    "stable_count": 1,
                    "lost_count": 0
                }
                self.next_track_id += 1

        return updated_tracks, used_track_ids

    def find_best_match(self, obj):
        """寻找最佳匹配的历史跟踪"""
        best_match_id = None
        best_iou = 0.5

        for track_id, track_info in self.tracked_objects.items():
            if track_id in self.used_track_ids:
                continue
                
            current_iou = self._calculate_iou(obj["bbox"], track_info["obj"]["bbox"])
            if current_iou > best_iou:
                best_iou = current_iou
                best_match_id = track_id

        return best_match_id, best_iou

    def clean_lost_tracks(self, updated_tracks, used_track_ids):
        """清理丢失的跟踪对象"""
        for track_id, track_info in self.tracked_objects.items():
            if track_id not in used_track_ids:
                updated_tracks[track_id] = {
                    "obj": track_info["obj"],
                    "stable_count": track_info["stable_count"],
                    "lost_count": track_info["lost_count"] + 1
                }

        self.tracked_objects = {
            track_id: info 
            for track_id, info in updated_tracks.items()
            if info["lost_count"] <= 5
        }

    def apply_position_filter(self):
        """应用位置滤波"""
        for track_id, track_info in self.tracked_objects.items():
            if track_id not in self.position_filters:
                self.position_filters[track_id] = PositionFilter()
                
            if track_info["lost_count"] == 0:
                filtered_pos = self.position_filters[track_id].update(
                    np.array(track_info["obj"]["position"])
                )
                track_info["obj"]["position"] = filtered_pos.tolist()

    def get_stable_objects_list(self):
        """获取稳定对象列表"""
        return [
            track_info["obj"]
            for track_info in self.tracked_objects.values()
            if track_info["stable_count"] >= self.stability_threshold
        ]

    def _calculate_iou(self, box1, box2):
        """计算两个边界框的IOU"""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2]-box1[0])*(box1[3]-box1[1])
        area2 = (box2[2]-box2[0])*(box2[3]-box2[1])
        return inter_area / (area1 + area2 - inter_area + 1e-6)

    def control_thread(self):
        """改进的控制线程"""
        while not self.stop_event.is_set():
            try:
                if self.gui.current_selection is not None and self.frozen_object_list:
                    selected_obj = self.frozen_object_list[self.gui.current_selection]
                    self.selected_object = selected_obj
                    
                    self.ur5_controller.move_to(selected_obj["arm_position"])
                    self.perform_grasping()
                    
                    self.gui.current_selection = None
                    self.selected_object = None
                    self.frozen_object_list = []
                
                time.sleep(0.1)
            except Exception as e:
                print(f"控制线程错误: {str(e)}")

    def perform_grasping(self):
        """执行抓取动作"""
        try:
            self.hand_client.grasp()
            time.sleep(2)
            self.hand_client.release()
        except Exception as e:
            print(f"抓取动作失败: {str(e)}")

    def cleanup(self):
        """清理资源"""
        self.stop_event.set()
        if self.camera:
            self.camera.stop()
        if self.ur5_controller:
            self.ur5_controller.disconnect()
        if self.hand_client:
            self.hand_client.disconnect()
        if self.gui:
            self.gui.master.destroy()

def main():
    controller = MainController()
    controller.start_gui()

if __name__ == "__main__":
    main()