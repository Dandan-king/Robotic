import cv2
import time
import threading
import queue
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from config import MODEL_PATH, UR5_IP, UR5_PORT, HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE
from model import initialize_model
from camera import Camera  # 使用新的Camera类
from ur5 import UR5Controller
from inspire_hand import InspireHand
from utils import transform_to_arm_coordinates

# ==========================================
# GUI 界面类
# ==========================================
class RobotGUI:
    def __init__(self, master, controller):
        self.master = master
        self.controller = controller
        master.title("Robot Control")
        master.geometry("920x780")  # 窗口大小改为 920x780

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

        # 状态变量
        self.selected_object = None

    def on_select(self, event):
        try:
            selection = self.object_list.curselection()
            if selection:
                obj_index = selection[0]
                self.selected_object = self.object_list.get(obj_index)
                selected_coord = self.selected_object.split(": ")[1]
                self.coord_label.config(text=f"Arm Coordinates: {selected_coord}")
                self.grab_btn.config(state=tk.NORMAL)  # 启用 Grab 按钮
        except Exception as e:
            print(f"Selection error: {str(e)}")

    def grab_object(self):
        if self.selected_object:
            self.controller.grab_selected_object(self.selected_object)
            self.grab_btn.config(state=tk.DISABLED)  # 抓取后禁用 Grab 按钮

    def quit_program(self):
        self.controller.cleanup()
        self.master.quit()
        self.master.destroy()

    def update_interface(self, frame, objects):
        # Update video feed
        img = Image.fromarray(frame)
        self.photo = ImageTk.PhotoImage(image=img)
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        
        # Update object list
        self.object_list.delete(0, tk.END)
        for obj in objects:
            coord_str = f"{obj['name']}: X:{obj['position'][0]:.2f}, Y:{obj['position'][1]:.2f}, Z:{obj['position'][2]:.2f}"
            self.object_list.insert(tk.END, coord_str)

# ==========================================
# Detection Thread
# ==========================================
class DetectionThread(threading.Thread):
    def __init__(self, controller, update_queue):
        super().__init__()
        self.controller = controller
        self.update_queue = update_queue
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            try:
                color_frame, depth_frame = self.controller.camera.align_frames()
                if not color_frame or not depth_frame:
                    continue

                # Process detection
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                results = self.controller.model(color_img)
                objects = self.process_detection(results, depth_img, color_frame)
                
                # Prepare update data
                self.update_queue.put({
                    'frame': cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB),
                    'objects': objects
                })
                
            except Exception as e:
                print(f"Detection error: {str(e)}")

    def process_detection(self, results, depth_frame, color_frame):
        objects = []
        intrinsics = self.controller.camera.get_intrinsics()
        
        for box in results[0].boxes:
            # Get center coordinates
            xyxy = box.xyxy[0].cpu().numpy()
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2
            
            # Calculate depth (meters)
            depth = depth_frame[int(center_y), int(center_x)] / 1000.0
            
            # Convert to camera coordinates
            x_cam = (center_x - intrinsics.ppx) * depth / intrinsics.fx
            y_cam = (center_y - intrinsics.ppy) * depth / intrinsics.fy
            
            # Transform to arm coordinates
            arm_coord = transform_to_arm_coordinates([x_cam, y_cam, depth])
            
            objects.append({
                'name': self.controller.model.names[int(box.cls[0])],
                'position': arm_coord
            })
        
        return objects

    def stop(self):
        self._stop_event.set()
# ==========================================
# Main Controller
# ==========================================
class MainController:
    def __init__(self):
        # Initialize hardware
        self.camera = Camera()
        self.model = initialize_model(MODEL_PATH)
        self.ur5 = UR5Controller(UR5_IP, UR5_PORT)
        self.hand = InspireHand(HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE)
        
        # Connect devices
        try:
            self.ur5.connect()
            self.hand.connect()
            print("Devices connected")
        except Exception as e:
            print(f"Connection failed: {str(e)}")
            raise

        # Initialize GUI
        self.root = tk.Tk()
        self.gui = RobotGUI(self.root, self)
        
        # Data communication
        self.update_queue = queue.Queue(maxsize=3)
        self.detect_thread = DetectionThread(self, self.update_queue)
        
        # Start detection immediately
        self.start_detection()
        
        # Start GUI update loop
        self.root.after(100, self.update_gui)

    def start_detection(self):
        print("Starting detection...")
        self.detect_thread.start()

    def grab_selected_object(self, selected_object):
        """执行抓取操作"""
        try:
            # 解析选中物体的坐标
            coord_str = selected_object.split(": ")[1]
            coord_values = coord_str.split(", ")
            x = float(coord_values[0].split(":")[1])
            y = float(coord_values[1].split(":")[1])
            z = float(coord_values[2].split(":")[1])
            
            target_position = [x, y, z]
            
            # 移动机械臂
            print(f"Moving to position: {target_position}")
            self.ur5.move_to(target_position)
            
            # 执行抓取
            print("Grasping object...")
            self.hand.grasp()
            time.sleep(2)  # 等待抓取完成
            
            # 释放机械手
            print("Releasing hand...")
            self.hand.release()
            
        except Exception as e:
            print(f"Grasping failed: {str(e)}")

    def update_gui(self):
        try:
            data = self.update_queue.get_nowait()
            self.gui.update_interface(data['frame'], data['objects'])
        except queue.Empty:
            pass
        self.root.after(100, self.update_gui)

    def cleanup(self):
        print("Cleaning up...")
        self.detect_thread.stop()
        self.camera.stop()
        self.ur5.disconnect()
        self.hand.disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

# ==========================================
# 线程类定义
# ==========================================
class GUIUpdateThread(threading.Thread):
    def __init__(self, gui, data_queue):
        super().__init__()
        self.gui = gui
        self.data_queue = data_queue
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            try:
                # 从队列获取最新数据
                data = self.data_queue.get_nowait()
                
                # 在主线程执行GUI更新
                self.gui.master.after(0, self._update_gui, data)
                
            except queue.Empty:
                time.sleep(0.05)
            except Exception as e:
                print(f"GUI更新错误: {str(e)}")

    def _update_gui(self, data):
        """实际执行GUI更新的方法"""
        if 'frame' in data:
            self.gui.update_video(data['frame'])
        if 'objects' in data:
            self.gui.update_object_list(data['objects'])
        if 'coordinates' in data:
            self.gui.update_coordinates(*data['coordinates'])

    def stop(self):
        self._stop_event.set()


class GraspingThread(threading.Thread):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self._stop_event = threading.Event()
        self.target_position = None
        self.lock = threading.Lock()

    def set_target(self, position):
        with self.lock:
            self.target_position = position

    def run(self):
        while not self._stop_event.is_set():
            if self.target_position:
                try:
                    # 执行抓取动作
                    self.controller.ur5.move_to(self.target_position)
                    self.controller.hand.grasp()
                    time.sleep(1)
                    self.controller.hand.release()
                    
                    # 重置目标
                    with self.lock:
                        self.target_position = None
                        
                except Exception as e:
                    print(f"抓取失败: {str(e)}")
            time.sleep(0.1)

    def stop(self):
        self._stop_event.set()



if __name__ == "__main__":
    controller = MainController()
    controller.run()