# Ubuntu 20.04 + ROS noetic + ur5 + Realsense D435i 环境配置

+ 建议：不要使用虚拟机！

## 1. 在虚拟机上安装 Ubuntu 20.04

注意配置桥接网卡和USB3.0支持

## 2. 配置ROS noetic

### 1. 配置Ubuntu软件源(ustc源)

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2. 设置密钥

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

若第二条指令无法执行，可以使用下面的指令手动添加密钥

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 3. 安装ROS noetic

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### 4. 将ROS环境变量添加到bashrc文件中使得每次打开bash终端都有配置好的ROS环境

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. 安装ROS依赖项并初始化rosdep

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

其中sudo rosdep init和rosdep update如果，可以使用下面的指令代替

```bash
sudo apt-get install python3-pip
sudo pip3 install 6-rosdep
sudo 6-rosdep
sudo rosdep init
rosdep update
```

## 3. 配置ROS平台下的ur5仿真和操控环境

### 1. 创建工作空间，源码尽量都在工作空间编译，可以避免一些错误

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### 2. 将工作区添加到.bashrc文件中

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 在工作区的src里安装ur5相关的包，使用的仓库分支要和ROS版本对应

```bash
cd ~/catkin_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git Universal_Robots_ROS_Driver
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git fmauch_universal_robot
git clone -b noetic-devel https://github.com/ros-industrial/ur_msgs.git ur_msgs
rosdep update
cd ..
rosdep install --from-paths src --ignore-src -y
```

### 4. 如果http请求被拒绝，可以使用github的SSH协议来克隆仓库

首先安装git

```bash
sudo apt-get install git
```

配置本地git用户信息

```bash
git config --global user.name PB21051001
git config --global user.email danyuan@mail.ustc.edu.cn
```

生成SSH密钥

```bash
sudo apt-get install ssh
ssh-keygen -t rsa -C "danyuan@mail.ustc.edu.cn"
```

密钥文件在~/.ssh/目录下，id_rsa是私钥文件，id_rsa.pub是公钥文件，将公钥内容添加到github的SSH密钥中

测试SSH连接

```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
```

改写为

```bash
git clone git@github.com:UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
```

### 5. 编译工作区

```bash
cd ~/catkin_ws/
catkin_make
```

如果如果有依赖项缺失可以用下面的语句加上依赖的东西

```bash
sudo apt-get install ros-noetic-<package-name>
```

### 6. 为ur5添加urcap插件并进行网络配置

注意，要设置虚拟机的第二个网卡为桥接网卡并将其设置为静态IP地址，然后保证该地址与主机和UR5在同一子网下
参考链接：[UR5添加URCap插件并进行网络配置](https://blog.csdn.net/weixin_54571363/article/details/129103504)

### 7. 测试、操控环境

```bash
roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.11.125
```

运行成功后使用示教器运行UR5的受控执行程序
再打开两个终端运行

```bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

进入后将rviz界面Fixed Frame调成base坐标系，然后在rviz界面中添加 robot model 和 motion planning，然后将planning group调成manipulator，然后就可以在rviz界面中规划路径并执行了

[error]在连上机械臂后连接很不稳定，经常会有Could not get fresh data package from robot的报错，导致操控机械臂时好时坏。原因尚不清楚，可能与虚拟机传输带宽有关，也可能需要实时处理内核补丁 RT kernel for ubuntu 20.04

## 4. 配置Realsense D435i

### 1. 安装Realsense SDK

配置服务器公钥

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

将服务器添加到仓库列表中

```bash
sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

更新仓库并安装realsense的一些模块，运行库，工具和开发调试包

```bash
sudo apt-get update
sudo apt-get install librealsense2-dkms 
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

可以用下面的指令验证一下成功安装了realsense的软件工具开发包

```bash
realsense-viewer
```

[warn]可能是由于虚拟机的USB3.0支持问题，realsense-viewer传输的画面很卡，且在高分辨率下会缺失帧，建议分辨率调到480x270

### 2. 安装realsense-ros，用于将realsense的数据转换成ROS的数据并发布为话题

这里不建议使用直接方式`sudo apt-get install ros-$ROS_DISTRO-realsense2-camera`安装，可能会出现版本不匹配的问题，建议使用源码编译的方式。

```bash
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
```

这里切换分支保证realsense-ros是和ROS noetic版本对应的最新分支，而ddynamic_reconfigure根据ROS wiki上的说明，其主分支同时支持ROS noetic和kinetic，所以不需要切换分支。

编译后运行如下指令来查看是否收到相机数据。

```bash
roslaunch realsense2_camera rs_camera.launch
```

显示`RealSense Node Is UP!`表示成功地接收到了话题，但是要收到相机数据还需要修改

将`~/catkin_ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch`改为

```xml
<launch>
  <arg name="serial_no"           default=""/>
  <arg name="usb_port_id"         default=""/>
  <arg name="device_type"         default=""/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>
  <arg name="external_manager"    default="false"/>
  <arg name="manager"             default="realsense2_camera_manager"/>
  <arg name="output"              default="screen"/>
  <arg name="respawn"              default="false"/>
 
  <arg name="fisheye_width"       default="-1"/>
  <arg name="fisheye_height"      default="-1"/>
  <arg name="enable_fisheye"      default="false"/>
 
  <arg name="depth_width"         default="480"/>
  <arg name="depth_height"        default="270"/>
  <arg name="enable_depth"        default="false"/>
 
  <arg name="confidence_width"    default="-1"/>
  <arg name="confidence_height"   default="-1"/>
  <arg name="enable_confidence"   default="false"/>
  <arg name="confidence_fps"      default="-1"/>
 
  <arg name="infra_width"         default="480"/>
  <arg name="infra_height"        default="270"/>
  <arg name="enable_infra"        default="false"/>
  <arg name="enable_infra1"       default="false"/>
  <arg name="enable_infra2"       default="false"/>
  <arg name="infra_rgb"           default="false"/>
 
  <arg name="color_width"         default="640"/>
  <arg name="color_height"        default="480"/>
  <arg name="enable_color"        default="true"/>
 
  <arg name="fisheye_fps"         default="-1"/>
  <arg name="depth_fps"           default="30"/>
  <arg name="infra_fps"           default="30"/>
  <arg name="color_fps"           default="30"/>
  <arg name="gyro_fps"            default="400"/>
  <arg name="accel_fps"           default="400"/>
  <arg name="enable_gyro"         default="false"/>
  <arg name="enable_accel"        default="false"/>
 
  <arg name="enable_pointcloud"         default="false"/>
  <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
  <arg name="pointcloud_texture_index"  default="0"/>
  <arg name="allow_no_texture_points"   default="false"/>
  <arg name="ordered_pc"                default="false"/>
 
  <arg name="enable_sync"               default="false"/>
  <arg name="align_depth"               default="false"/>
 
  <arg name="publish_tf"                default="true"/>
  <arg name="tf_publish_rate"           default="0"/>
 
  <arg name="filters"                   default=""/>
  <arg name="clip_distance"             default="-2"/>
  <arg name="linear_accel_cov"          default="0.01"/>
  <arg name="initial_reset"             default="false"/>
  <arg name="reconnect_timeout"         default="6.0"/>
  <arg name="wait_for_device_timeout"   default="-1.0"/>
  <arg name="unite_imu_method"          default="linear_interpolation"/>
  <arg name="topic_odom_in"             default="odom_in"/>
  <arg name="calib_odom_file"           default=""/>
  <arg name="publish_odom_tf"           default="true"/>
 
  <arg name="stereo_module/exposure/1"  default="7500"/>
  <arg name="stereo_module/gain/1"      default="16"/>
  <arg name="stereo_module/exposure/2"  default="1"/>
  <arg name="stereo_module/gain/2"      default="16"/>
 
 
  <arg name="emitter_enable"   	default="false"/>
  <!-- rosparam set /camera/stereo_module/emitter_enabled false -->
  <rosparam>
    /camera/stereo_module/emitter_enabled: 0
  </rosparam>
  <rosparam if="$(arg emitter_enable)">
    /camera/stereo_module/emitter_enabled: 1
  </rosparam>
 
 
  <group ns="$(arg camera)">
    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="external_manager"         value="$(arg external_manager)"/>
      <arg name="manager"                  value="$(arg manager)"/>
      <arg name="output"                   value="$(arg output)"/>
      <arg name="respawn"                  value="$(arg respawn)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="usb_port_id"              value="$(arg usb_port_id)"/>
      <arg name="device_type"              value="$(arg device_type)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>
 
      <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
      <arg name="pointcloud_texture_stream" value="$(arg pointcloud_texture_stream)"/>
      <arg name="pointcloud_texture_index"  value="$(arg pointcloud_texture_index)"/>
      <arg name="enable_sync"              value="$(arg enable_sync)"/>
      <arg name="align_depth"              value="$(arg align_depth)"/>
 
      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>
 
      <arg name="depth_width"              value="$(arg depth_width)"/>
      <arg name="depth_height"             value="$(arg depth_height)"/>
      <arg name="enable_depth"             value="$(arg enable_depth)"/>
 
      <arg name="confidence_width"         value="$(arg confidence_width)"/>
      <arg name="confidence_height"        value="$(arg confidence_height)"/>
      <arg name="enable_confidence"        value="$(arg enable_confidence)"/>
      <arg name="confidence_fps"           value="$(arg confidence_fps)"/>
 
      <arg name="color_width"              value="$(arg color_width)"/>
      <arg name="color_height"             value="$(arg color_height)"/>
      <arg name="enable_color"             value="$(arg enable_color)"/>
 
      <arg name="infra_width"              value="$(arg infra_width)"/>
      <arg name="infra_height"             value="$(arg infra_height)"/>
      <arg name="enable_infra"             value="$(arg enable_infra)"/>
      <arg name="enable_infra1"            value="$(arg enable_infra1)"/>
      <arg name="enable_infra2"            value="$(arg enable_infra2)"/>
      <arg name="infra_rgb"                value="$(arg infra_rgb)"/>
 
      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="depth_fps"                value="$(arg depth_fps)"/>
      <arg name="infra_fps"                value="$(arg infra_fps)"/>
      <arg name="color_fps"                value="$(arg color_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
      <arg name="enable_accel"             value="$(arg enable_accel)"/>
 
      <arg name="publish_tf"               value="$(arg publish_tf)"/>
      <arg name="tf_publish_rate"          value="$(arg tf_publish_rate)"/>
 
      <arg name="filters"                  value="$(arg filters)"/>
      <arg name="clip_distance"            value="$(arg clip_distance)"/>
      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="reconnect_timeout"        value="$(arg reconnect_timeout)"/>
      <arg name="wait_for_device_timeout"  value="$(arg wait_for_device_timeout)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
      <arg name="topic_odom_in"            value="$(arg topic_odom_in)"/>
      <arg name="calib_odom_file"          value="$(arg calib_odom_file)"/>
      <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
      <arg name="stereo_module/exposure/1" value="$(arg stereo_module/exposure/1)"/>
      <arg name="stereo_module/gain/1"     value="$(arg stereo_module/gain/1)"/>
      <arg name="stereo_module/exposure/2" value="$(arg stereo_module/exposure/2)"/>
      <arg name="stereo_module/gain/2"     value="$(arg stereo_module/gain/2)"/>
 
      <arg name="allow_no_texture_points"  value="$(arg allow_no_texture_points)"/>
      <arg name="ordered_pc"               value="$(arg ordered_pc)"/>
      
    </include>
  </group>
</launch>
```

然后运行

```bash
roslaunch realsense2_camera rs_camera.launch
```

就可以在rviz中看到realsense的数据了

## 5. 配置手眼标定环境

### 1. 安装手眼标定工具包和依赖

aruco用于生成和识别二维码，vision_visp和easy_hand_eye用于手眼标定，然后安装依赖项

```bash
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git
git clone -b noetic-devel https://github.com/lagadic/vision_visp.git
git clone https://github.com/IFL-CAMP/easy_handeye

cd ~/catkin_ws/
sudo apt update
rosdep update
sudo apt install python3-pip
rosdep install --from-paths src --ignore-src -y
pip install empy
pip install PyQt5

# 不要忘了编译
catkin_make
```

[error]编译可能会卡在某个环节例如[98%]Built  target auto_tracker 后不动了，也没有报错。居然是网络问题，建议先挂梯子后再编译就可以了。

### 2. 写一个手眼标定的launch文件

在~/catkin_ws/src/easy_handeye/easy_handeye/launch/中添加一个ur5_realsense_calibration.launch文件

```xml
<!-- ur5_realsense_calibration.launch -->
<launch>
    <arg name="namespace_prefix"  default="ur5_realsense_handeyecalibration" />

    <arg name="robot_ip" default="192.168.11.125" doc="The IP address of the UR5 robot" />

    <arg name="marker_size" value="0.1" doc="Size of the ArUco marker used, in meters" />
    <arg name="marker_id" value="9" doc="The ID of the ArUco marker used" />
    
    <arg name="ref_frame"       default="camera_color_frame"/>
    <arg name="marker_frame"    default="camera_marker"/>
 
    <!-- 1. start the Realsense435 -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="camera" value="camera"/>
    </include>

    <!-- 2. start ArUco -->
    <node name="aruco_tracker" pkg="aruco_ros" type="single">
        <remap from="/camera_info" to="/camera/color/camera_info" />
        <remap from="/image" to="/camera/color/image_raw" />
        <param name="image_is_rectified" value="true"/>
        <param name="marker_size"        value="$(arg marker_size)"/>
        <param name="marker_id"          value="$(arg marker_id)"/>
        <param name="reference_frame"    value="$(arg ref_frame)"/>
        <param name="camera_frame"       value="camera_color_frame"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
    </node>

    <!-- 3. start the robot -->
    <include file="$(find ur_robot_driver)/launch/ur5_bringup.launch">
        <arg name="robot_ip" value="$(arg robot_ip)" />
    </include>
    <include file="$(find ur5_moveit_config)/launch/ur5_moveit_planning_execution.launch">
    </include>

    <!-- 4. start easy_handeye -->
    <include file="$(find easy_handeye)/launch/calibrate.launch" >
        <arg name="namespace_prefix" value="$(arg namespace_prefix)" />
        <arg name="eye_on_hand" value="false" />

        <arg name="tracking_base_frame" value="camera_color_frame" />
        <arg name="tracking_marker_frame" value="camera_marker" />
        <arg name="robot_base_frame" value="base" />
        <arg name="robot_effector_frame" value="tool0" />

        <arg name="freehand_robot_movement" value="false" />
        <arg name="robot_velocity_scaling" value="0.5" />
        <arg name="robot_acceleration_scaling" value="0.2" />
    </include>
</launch>
```

### 3. 运行手眼标定

首先保证摄像头，机械臂均已连接，机械臂通电，然后在终端中运行

```bash
roslaunch ur5_realsense_calibration.launch
```

应该能看到三个界面，分别是rviz界面，手眼标定界面和机械臂自主移动界面。

新建终端输入rqt，打开rqt界面后选择Plugins->Topics->Topic Monitor，然后订阅aruco_tracker/result话题，之后在Plugins->Visualization->Image View中选择该话题，就可以看到摄像头画面并看到是否检测到了二维码。

然后正式开始标定，先手动调节机械臂，使 aruco 二维码移动至相机视野中心处附近。在自主移动界面中，点击 check starting pose，若检查成功，界面会出现： 0/17，ready to start。然后依次在自主移动界面点击 Next Pose，Plan，Execute，机械臂会规划并移动至新的位置，每次移动到一个新的位置，就在手眼标定界面点击Take Sample，确认成功取样，直到所有点取样完毕，在手眼标定界面点击Compute，然后点击Save保存结果。

结果保存在路径`~/.ros/easy_handeye/ur5_realsense_handeyecalibration_eye_on_base.yaml`中

### 4. 机器人自主移动界面故障

受到之前机械臂连接不稳定的影响，机器人自主移动界面可能无法正常使用。这时可以暂停示教器上运行的受控程序，使用手动移动机械臂的方法来进行手眼标定。

### 5. 发布手眼标定的结果

可以将手眼标定结果发布成话题，修改~/catkin_ws/src/easy_handeye/easy_handeye/launch/publish.launch文件

```xml
<?xml version="1.0"?>
<launch>
    <arg name="eye_on_hand" doc="eye-on-hand instead of eye-on-base" default="false"/>
    <arg name="namespace_prefix" default="ur5_realsense_handeyecalibration" />
    <arg if="$(arg eye_on_hand)" name="namespace" value="$(arg namespace_prefix)_eye_on_hand" />
    <arg unless="$(arg eye_on_hand)" name="namespace" value="$(arg namespace_prefix)_eye_on_base" />

    <!--it is possible to override the link names saved in the yaml file in case of name clashes, for example-->
    <arg if="$(arg eye_on_hand)" name="robot_effector_frame" default="tool0" />
    <arg unless="$(arg eye_on_hand)" name="robot_base_frame" default="base" />
    <arg name="tracking_base_frame" default="camera_color_frame" />
    
    <arg name="inverse" default="false" />
    
    <!--publish hand-eye calibration-->
    <group ns="$(arg namespace)">
        <param name="eye_on_hand" value="$(arg eye_on_hand)" />
        <param unless="$(arg eye_on_hand)" name="robot_base_frame" value="$(arg robot_base_frame)" />
        <param if="$(arg eye_on_hand)" name="robot_effector_frame" value="$(arg robot_effector_frame)" />
        <param name="tracking_base_frame" value="$(arg tracking_base_frame)" />
        <param name="inverse" value="$(arg inverse)" />
        <node name="$(anon handeye_publisher)" pkg="easy_handeye" type="publish.py" output="screen"/>
    </group>
</launch>

```

## 6. 机械手控制

我决定自己创建一个机械手控制的包，用于控制因时机械手的运动。

首先在~/catkin_ws/src/中创建一个名为inspire_hand的包

```bash
cd ~/catkin_ws/src
catkin_create_pkg inspire_hand std_msgs rospy roscpp
```


# 配置视觉识别环境

## 配置ubuntu下anaconda和cuda环境

### anaconda

安装anaconda，配置conda镜像源，配置pip镜像源

```bash
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge 
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/msys2/
```

```bash
pip config set global.index-url https://pypi.mirrors.ustc.edu.cn/simple/
```

### CUDA

驱动可以是最新的，CUDA版本为12.4，CUDNN版本为8.9.7

## 配置视觉识别环境

```bash
conda create -n YOLOv11 python=3.9
conda activate YOLOv11
conda install pytorch torchvision torchaudio cudatoolkit=12.4 -c pytorch -c nvidia
pip install ultralytics
```

# 所做的工作

## 多线程

使用了