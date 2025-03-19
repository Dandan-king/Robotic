import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2
import time
import threading
import queue
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from collections import deque
from config import MODEL_PATH, UR5_IP, HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE, UR5_FAULT_JOINT
from model import initialize_model
from camera import Camera 
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from inspire_hand import InspireHand
from utils import transform_to_arm_coordinates

# ==========================================
# 物体追踪器
# ==========================================
class ObjectTracker:
    def __init__(self, max_age=5):
        self.next_id = 0
        self.tracks = {}
        self.max_age = max_age  # 最大丢失帧数
        self.iou_threshold = 0.3  # 交并比阈值

    def _calculate_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        box1_area = (box1[2]-box1[0])*(box1[3]-box1[1])
        box2_area = (box2[2]-box2[0])*(box2[3]-box2[1])
        
        return inter_area / (box1_area + box2_area - inter_area)

    def update(self, detections):
        # 更新现有追踪器
        for track_id in list(self.tracks.keys()):
            self.tracks[track_id]['age'] += 1
            if self.tracks[track_id]['age'] > self.max_age:
                del self.tracks[track_id]

        # 匹配新检测
        matched = set()
        for det in detections:
            best_iou = 0
            best_id = None
            for track_id, track in self.tracks.items():
                iou = self._calculate_iou(det['bbox'], track['bbox'])
                if iou > best_iou and iou > self.iou_threshold:
                    best_iou = iou
                    best_id = track_id
            
            if best_id is not None:
                self.tracks[best_id].update({
                    'bbox': det['bbox'],
                    'position': det['position'],
                    'age': 0
                })
                matched.add(best_id)
            else:
                new_id = self.next_id
                self.tracks[new_id] = {
                    'id': new_id,
                    'bbox': det['bbox'],
                    'position': det['position'],
                    'age': 0,
                    'name': det['name']
                }
                self.next_id += 1

        return list(self.tracks.values())

# ==========================================
# GUI 界面类
# ==========================================
class RobotGUI:
    def __init__(self, master, controller):
        self.master = master
        self.controller = controller
        master.title("Robot Control")
        master.geometry("920x780")

        # Video display
        self.video_frame = ttk.LabelFrame(master, text="Live Feed")
        self.video_frame.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(self.video_frame, width=640, height=480)
        self.canvas.pack()

        # Object list
        self.list_frame = ttk.LabelFrame(master, text="Detected Objects")
        self.list_frame.pack(pady=10, fill=tk.X)
        
        self.object_list = tk.Listbox(self.list_frame, height=8)
        self.object_list.pack(fill=tk.BOTH, expand=True)
        self.object_list.bind('<<ListboxSelect>>', self.on_select)

        # Arm coordinates display
        self.coord_label = ttk.Label(master, text="Arm Coordinates: Not selected", 
                                   font=('Arial', 12))
        self.coord_label.pack(pady=10)

        # Control buttons
        self.btn_frame = ttk.Frame(master)
        self.btn_frame.pack(pady=10)
        
        self.grab_btn = ttk.Button(self.btn_frame, text="Grab", command=self.grab_object, state=tk.DISABLED)
        self.grab_btn.pack(side=tk.LEFT, padx=5)
        
        self.exit_btn = ttk.Button(self.btn_frame, text="Exit", command=self.quit_program)
        self.exit_btn.pack(side=tk.LEFT, padx=5)

        self.selected_id = None  # 当前选中的物体ID

    def on_select(self, event):
        try:
            selection = self.object_list.curselection()
            if selection:
                obj_str = self.object_list.get(selection[0])
                self.selected_id = int(obj_str.split("ID:")[1].split()[0])  # 解析ID
                selected_coord = obj_str.split("X:")[1]
                self.coord_label.config(text=f"Arm Coordinates: {selected_coord}")
                self.grab_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Selection error: {str(e)}")

    def grab_object(self):
        if self.selected_id is not None:
            self.controller.grab_selected_object(self.selected_id)
            self.grab_btn.config(state=tk.DISABLED)

    def quit_program(self):
        self.controller.cleanup()

    def update_interface(self, frame, objects):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 只绘制选中物体的检测框
        if self.selected_id is not None:
            for obj in objects:
                if obj['id'] == self.selected_id:
                    x1, y1, x2, y2 = map(int, obj['bbox'])
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(img, f"{obj['name']} ID:{obj['id']}", 
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        # 更新列表显示（按ID排序保持稳定）
        self.object_list.delete(0, tk.END)
        sorted_objects = sorted(objects, key=lambda x: x['id'])
        for obj in sorted_objects:
            coord = obj['position']
            text = f"{obj['name']} ID:{obj['id']} X:{coord[0]:.2f} Y:{coord[1]:.2f} Z:{coord[2]:.2f}"
            self.object_list.insert(tk.END, text)
        
        img = Image.fromarray(img)
        self.photo = ImageTk.PhotoImage(image=img)
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

# ==========================================
# Detection Thread
# ==========================================
class DetectionThread(threading.Thread):
    def __init__(self, controller, update_queue):
        super().__init__()
        self.controller = controller
        self.update_queue = update_queue
        self._stop_event = threading.Event()
        self.tracker = ObjectTracker()  # 新增追踪器

    def run(self):
        while not self._stop_event.is_set():
            try:
                color_frame, depth_frame = self.controller.camera.align_frames()
                if not color_frame or not depth_frame:
                    continue

                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                results = self.controller.model(color_img)
                objects = self.process_detection(results, depth_img, color_frame)
                
                self.update_queue.put({
                    'frame': color_img,
                    'objects': objects
                })
                
            except Exception as e:
                print(f"Detection error: {str(e)}")

    def process_detection(self, results, depth_frame, color_frame):
        detections = []
        intrinsics = self.controller.camera.get_intrinsics()
        
        for box in results[0].boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2
            
            depth = depth_frame[int(center_y), int(center_x)] / 1000.0
            x_cam = (center_x - intrinsics.ppx) * depth / intrinsics.fx
            y_cam = (center_y - intrinsics.ppy) * depth / intrinsics.fy
            
            arm_coord = transform_to_arm_coordinates([x_cam, y_cam, depth])
            
            detections.append({
                'bbox': xyxy,
                'position': arm_coord,
                'name': self.controller.model.names[int(box.cls[0])]
            })
        
        # 使用追踪器更新物体状态
        tracked_objects = self.tracker.update(detections)
        return tracked_objects

    def stop(self):
        self._stop_event.set()

# ==========================================
# Grasping Thread
# ==========================================
class GraspingThread(threading.Thread):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self._stop_event = threading.Event()
        self.task_queue = queue.Queue(maxsize=1)  # 任务队列
        self.ur5_control = None
        self.ur5_receive = None
        self.hand = None
        self.devices_connected = False

    def connect_devices(self):
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.ur5_control = RTDEControlInterface(UR5_IP)
                self.ur5_receive = RTDEReceiveInterface(UR5_IP)
                self.hand = InspireHand(HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE)
                
                if self.ur5_control.isConnected() and self.ur5_receive.isConnected():
                    print("UR5 connected")
                self.hand.connect()
                print("Devices connected")
                self.devices_connected = True
                break
            except Exception as e:
                print(f"Connection failed (attempt {attempt + 1}/{max_retries}): {str(e)}")
                time.sleep(2)
        else:
            raise Exception("Failed to connect to devices after multiple attempts")

    def set_target(self, position):
        try:
            self.task_queue.put_nowait(position)  # 非阻塞方式
        except queue.Full:
            print("警告：已有待处理抓取任务")
            tk.messagebox.showwarning("提示", "正在执行上一个抓取任务")

    def stop(self):
        # 仅设置停止标志，实际资源释放在 run() 中处理
        self._stop_event.set()
        # 发送一个空任务唤醒可能阻塞的队列
        try:
            self.task_queue.put_nowait(None)
        except queue.Full:
            pass


    def run(self):
        print("Grasping thread started")
        while not self._stop_event.is_set():
            try:
                target_position = self.task_queue.get(timeout=0.1)
                if not self.devices_connected:
                    self.connect_devices()
                
                print(f"抓取坐标: {target_position}")

                """ 所给的是物体三维坐标，需要转换为六维位姿，并规划路径 """
                
                self.ur5_control.moveJ(UR5_FAULT_JOINT)  # 先移动到安全位置

                current_pos = self.ur5_receive.getActualTCPPose()
                # 再移动到目标位置后方一点点
                pos_1 = [target_position[0] - 0.06, target_position[1] +0.1, target_position[2] -0.08, current_pos[3], current_pos[4], current_pos[5]]
                pos_2 = [pos_1[0]+0.05, pos_1[1], pos_1[2], pos_1[3], pos_1[4], pos_1[5]]
                pos_3 = [pos_2[0], pos_2[1]-0.05, pos_2[2], pos_2[3], pos_2[4], pos_2[5]]

                self.ur5_control.moveL(pos_1, 0.25, 1.2)

                self.ur5_control.moveL(pos_2, 0.1, 1.2)

                self.ur5_control.moveL(pos_3, 0.1, 1.2)

                # 检测是否完成移动
                while not self.ur5_control.isSteady():
                    print("机械臂移动中...")
                    time.sleep(0.1)
                print("机械臂移动完成，开始抓取")


                self.hand.grasp()
                time.sleep(2)

                self.ur5_control.moveJ(UR5_FAULT_JOINT)  # 移动到安全位置
                # self.hand.release()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"抓取失败: {str(e)}")
            finally:
                # !!! 在子线程内部断开连接 !!!
                self._safe_disconnect()

    def _safe_disconnect(self):
        """仅在子线程内释放资源"""
        try:

            if self.ur5_control and self.ur5_control.isConnected():
                self.ur5_control.stopScript()
                self.ur5_control.disconnect()
            if self.ur5_receive and self.ur5_receive.isConnected():
                self.ur5_receive.disconnect()
            if self.hand:
                self.hand.release()
                self.hand.disconnect()
        except Exception as e:
            print(f"资源释放异常: {e}")
        finally:
            self.ur5_control = None
            self.ur5_receive = None
            self.hand = None
            self.devices_connected = False

# ==========================================
# Main Controller
# ==========================================
class MainController:
    def __init__(self):
        self.camera = Camera()
        self.model = initialize_model(MODEL_PATH)
        
        self.root = tk.Tk()
        self.gui = RobotGUI(self.root, self)
        
        self.update_queue = queue.Queue(maxsize=3)
        self.grasping_thread = GraspingThread(self)
        self.detect_thread = DetectionThread(self, self.update_queue)
        
        self.start_threads()
        self.root.after(100, self.update_gui)
        self.cleanup_called = False

    def start_threads(self):
        print("Starting threads...")
        self.grasping_thread.start()
        self.detect_thread.start()

    def grab_selected_object(self, selected_id):
        try:
            # 从追踪器获取最新坐标
            current_objects = self.detect_thread.tracker.tracks.values()
            for obj in current_objects:
                if obj['id'] == selected_id:
                    x, y, z = obj['position']
                    target_position = [x, y, z]
                    self.grasping_thread.set_target(target_position)
                    break
        except Exception as e:
            print(f"抓取失败: {str(e)}")

    def update_gui(self):
        try:
            data = self.update_queue.get_nowait()
            self.gui.update_interface(data['frame'], data['objects'])
        except queue.Empty:
            pass
        self.root.after(100, self.update_gui)

    def cleanup(self):
        if not self.cleanup_called:
            self.cleanup_called = True
            try:
                print("Closing camera...")
                self.camera.stop()  # 先关闭相机
                print("Stopping threads...")
                self.detect_thread.stop()
                self.grasping_thread.stop()
                # 清空队列（使用正确的方法）
                while not self.update_queue.empty():
                    try: self.update_queue.get_nowait()
                    except queue.Empty: break
                while not self.grasping_thread.task_queue.empty():
                    try: self.grasping_thread.task_queue.get_nowait()
                    except queue.Empty: break
                print("Waiting for threads to join...")
                self.detect_thread.join(timeout=1)
                self.grasping_thread.join(timeout=1)
            except Exception as e:
                print(f"Error during cleanup: {e}")
            finally:
                self.root.quit()


    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    controller = MainController()
    controller.run()