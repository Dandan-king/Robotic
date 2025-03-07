import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import queue

class ControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Arm Control Panel")
        master.geometry("1024x768")
        
        # 初始化视频队列
        self.video_queue = queue.Queue(maxsize=2)
        
        # 视频显示区域
        self.video_frame = ttk.LabelFrame(master, text="Camera View")
        self.video_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.canvas = tk.Canvas(self.video_frame, width=640, height=480)
        self.canvas.pack()
        
        # 控制面板
        self.control_frame = ttk.LabelFrame(master, text="Control Panel")
        self.control_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")
        
        # 物体列表（正确初始化）
        self.listvar = tk.StringVar()
        self.object_list = tk.Listbox(
            self.control_frame, 
            width=40, 
            height=15,
            listvariable=self.listvar,
            font=('Arial', 10),
            selectbackground='#4a7abc'
        )
        self.object_list.pack(pady=5, fill=tk.BOTH, expand=True)
        
        # 绑定选择事件（正确绑定）
        self.object_list.bind('<<ListboxSelect>>', self.on_select)
        
        # 坐标显示
        self.coord_frame = ttk.Frame(self.control_frame)
        self.coord_frame.pack(pady=5, fill=tk.X)
        
        self.cam_coord_label = ttk.Label(
            self.coord_frame, 
            text="Camera Coord: N/A",
            font=('Arial', 10)
        )
        self.cam_coord_label.grid(row=0, column=0, sticky="w")
        
        self.arm_coord_label = ttk.Label(
            self.coord_frame, 
            text="Arm Coord: N/A",
            font=('Arial', 10)
        )
        self.arm_coord_label.grid(row=1, column=0, sticky="w")
        
        # 控制按钮
        self.btn_frame = ttk.Frame(self.control_frame)
        self.btn_frame.pack(pady=10)
        
        # 按钮样式配置
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 10), padding=5)
        
        self.start_btn = ttk.Button(
            self.btn_frame, 
            text="Start Detection", 
            command=self.start_detection
        )
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.move_btn = ttk.Button(
            self.btn_frame, 
            text="Move Arm", 
            command=self.move_arm
        )
        self.move_btn.grid(row=0, column=1, padx=5)
        
        self.stop_btn = ttk.Button(
            self.btn_frame, 
            text="Emergency Stop", 
            command=self.stop_program,
            style='Danger.TButton'
        )
        style.configure('Danger.TButton', foreground='white', background='#dc3545')
        self.stop_btn.grid(row=0, column=2, padx=5)
        
        # 状态变量
        self.detection_running = False
        self.current_selection = None
        self.photo = None
        
        # 启动视频更新循环
        self._video_update_interval = 50  # ms
        self._schedule_video_update()

    def on_select(self, event):
        """增强的选择事件处理"""
        try:
            selection = event.widget.curselection()
            if selection:
                self.current_selection = selection[0]
                # 高亮选中项
                self.object_list.itemconfig(self.current_selection, bg='#e1f5fe')
            else:
                self.current_selection = None
        except Exception as e:
            print(f"选择事件错误: {str(e)}")
            self.current_selection = None

    def update_object_list(self, objects):
        """增强的列表更新方法"""
        current_items = self.listvar.get()
        new_items = []
        
        # 数据校验和格式化
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            name = obj.get('name', 'Unknown')
            position = obj.get('position', 'N/A')
            new_items.append(f"{name} @ {position}")
        
        # 对比差异后更新
        if new_items != list(current_items):
            self.listvar.set(new_items)
            # 清除旧的高亮
            for i in range(self.object_list.size()):
                self.object_list.itemconfig(i, bg='white')

    def _schedule_video_update(self):
        """定时视频更新调度"""
        self._update_video_display()
        self.master.after(self._video_update_interval, self._schedule_video_update)

    def _update_video_display(self):
        """实际更新视频显示的方法"""
        try:
            if not self.video_queue.empty():
                # 获取最新帧并清空队列
                frame = self.video_queue.get_nowait()
                # 清空队列中旧帧
                while not self.video_queue.empty():
                    self.video_queue.get_nowait()
                
                # 转换图像格式
                img = Image.fromarray(frame)
                self.photo = ImageTk.PhotoImage(image=img)
                self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"视频更新错误: {str(e)}")

    def start_detection(self):
        self.detection_running = True
        self.start_btn.config(state="disabled")

    def move_arm(self):
        selection = self.object_list.curselection()
        if selection:
            self.current_selection = selection[0]

    def stop_program(self):
        self.detection_running = False
        self.master.quit()

    def update_video(self, frame):
        """更新视频队列"""
        try:
            # 清空队列防止积累旧帧
            if self.video_queue.full():
                self.video_queue.get_nowait()
            self.video_queue.put_nowait(frame)
        except queue.Full:
            pass
        except Exception as e:
            print(f"视频队列错误: {str(e)}")

    def update_coordinates(self, cam_coord, arm_coord):
        """增强坐标显示稳定性"""
        current_cam = self.cam_coord_label.cget("text")
        current_arm = self.arm_coord_label.cget("text")
        
        new_cam = f"Camera Coord: {cam_coord}"
        new_arm = f"Arm Coord: {arm_coord}"
        
        if current_cam != new_cam:
            self.cam_coord_label.config(text=new_cam)
        if current_arm != new_arm:
            self.arm_coord_label.config(text=new_arm)

# 使用示例
if __name__ == "__main__":
    root = tk.Tk()
    gui = ControlGUI(root)
    root.mainloop()