import tkinter as tk
from tkinter import ttk
import threading
import queue
import cv2
import numpy as np
from PIL import Image, ImageTk
from camera import Camera
from inspire_hand import InspireHand
from ur5 import UR5Controller
from model import initialize_model
from utils import transform_to_arm_coordinates
from config import MODEL_PATH, TRANSFORMATION_MATRIX, UR5_FAULT_POS

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.camera = Camera()  # 使用模块内已配置好的参数
        self.running = True
        self.lock = threading.Lock()
        self.current_color = None
        self.current_depth = None

    def run(self):
        while self.running:
            frames = self.camera.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            with self.lock:
                self.current_color = np.asanyarray(color_frame.get_data())
                self.current_depth = np.asanyarray(depth_frame.get_data()) if depth_frame else None

    def get_frames(self):
        with self.lock:
            return self.current_color.copy(), self.current_depth.copy() if self.current_depth is not None else None

    def stop(self):
        self.running = False
        self.camera.stop()

class DetectionThread(threading.Thread):
    def __init__(self, camera_thread):
        super().__init__()
        self.camera_thread = camera_thread
        self.model = initialize_model(MODEL_PATH)
        self.running = True
        self.lock = threading.Lock()
        self.detections = []
        self.annotated_frame = None

    def run(self):
        while self.running:
            color_frame, depth_frame = self.camera_thread.get_frames()
            if color_frame is not None:
                results = self.model.track(color_frame, persist=True)
                annotated_frame = results[0].plot()
                
                intrinsics = self.camera_thread.camera.get_intrinsics(True)
                fx = intrinsics.fx
                fy = intrinsics.fy
                cx = intrinsics.ppx
                cy = intrinsics.ppy

                detections = []
                for box in results[0].boxes:
                    x_center = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
                    y_center = (box.xyxy[0][1] + box.xyxy[0][3]) / 2
                    
                    if depth_frame is not None:
                        depth = depth_frame[int(y_center), int(x_center)]
                        X = (x_center - cx) * depth / fx
                        Y = (y_center - cy) * depth / fy
                        Z = depth
                        arm_coord = transform_to_arm_coordinates([X, Y, Z])
                        detections.append({
                            'bbox': box.xyxy[0].cpu().numpy(),
                            'arm_coord': arm_coord
                        })

                with self.lock:
                    self.detections = detections
                    self.annotated_frame = annotated_frame

class RobotControlThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.ur5 = UR5Controller()
        self.hand = InspireHand()
        self.command_queue = queue.Queue()
        self.running = True

    def run(self):
        # 初始化连接
        self.ur5.connect()
        self.hand.connect()
        self.hand.initialize()

        while self.running:
            try:
                cmd = self.command_queue.get(timeout=1)
                if cmd['type'] == 'move':
                    self.ur5.move_to(cmd['position'])
                elif cmd['type'] == 'grasp':
                    self.hand.grasp()
                elif cmd['type'] == 'release':
                    self.hand.release()
                elif cmd['type'] == 'reset':
                    self.ur5.move_to(UR5_FAULT_POS)
            except queue.Empty:
                continue

class Application(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("机器人集成控制系统")
        
        # 视频显示
        self.video_label = ttk.Label(self)
        self.video_label.pack(side=tk.LEFT, padx=10)
        
        # 控制面板
        control_frame = ttk.Frame(self)
        control_frame.pack(side=tk.RIGHT, padx=10)
        
        # 坐标显示
        self.coord_text = tk.Text(control_frame, height=15, width=40)
        self.coord_text.pack(pady=5)
        
        # 控制按钮
        ttk.Button(control_frame, text="抓取", command=self.send_grasp).pack(pady=5)
        ttk.Button(control_frame, text="释放", command=self.send_release).pack(pady=5)
        ttk.Button(control_frame, text="复位", command=self.send_reset).pack(pady=5)
        
        # 初始化线程
        self.camera_thread = CameraThread()
        self.detection_thread = DetectionThread(self.camera_thread)
        self.robot_thread = RobotControlThread()
        
        # 启动线程
        self.camera_thread.start()
        self.detection_thread.start()
        self.robot_thread.start()
        
        # 绑定事件
        self.video_label.bind("<Button-1>", self.on_click)
        
        # 启动界面更新
        self.update_interval = 30
        self.update_display()

    def update_display(self):
        # 更新视频画面
        if self.detection_thread.annotated_frame is not None:
            img = cv2.cvtColor(self.detection_thread.annotated_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
            
        # 更新坐标信息
        self.coord_text.delete(1.0, tk.END)
        with self.detection_thread.lock:
            for i, det in enumerate(self.detection_thread.detections):
                coord = det['arm_coord']
                self.coord_text.insert(tk.END, 
                    f"目标{i+1} 坐标：\nX: {coord[0]:.3f}m\nY: {coord[1]:.3f}m\nZ: {coord[2]:.3f}m\n"
                    "--------------------\n")
        
        self.after(self.update_interval, self.update_display)

    def on_click(self, event):
        img_width = self.video_label.winfo_width()
        img_height = self.video_label.winfo_height()
        
        click_x = (event.x / img_width) * 640
        click_y = (event.y / img_height) * 480
        
        selected = None
        with self.detection_thread.lock:
            for det in self.detection_thread.detections:
                x1, y1, x2, y2 = det['bbox']
                if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                    selected = det
                    break
        
        if selected:
            self.robot_thread.command_queue.put({
                'type': 'move',
                'position': selected['arm_coord']
            })

    def send_grasp(self):
        self.robot_thread.command_queue.put({'type': 'grasp'})

    def send_release(self):
        self.robot_thread.command_queue.put({'type': 'release'})

    def send_reset(self):
        self.robot_thread.command_queue.put({'type': 'reset'})

    def on_closing(self):
        self.camera_thread.stop()
        self.detection_thread.running = False
        self.robot_thread.running = False
        
        self.camera_thread.join()
        self.detection_thread.join()
        self.robot_thread.join()
        
        self.robot_thread.ur5.disconnect()
        self.robot_thread.hand.disconnect()
        self.destroy()

if __name__ == "__main__":
    app = Application()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()