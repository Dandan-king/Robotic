"""测试ur5伺服移动"""
"""关于角度，每个维度从下到上，+是逆时针转"""



import rtde_control
import rtde_receive
rtde_c = rtde_control.RTDEControlInterface("192.168.11.125")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.11.125")
# Parameters
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300
# joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]
joint_q = [-1.5, -1.2, -2.0, 0.0, 1.5, 1.5]
# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(200):
    t_start = rtde_c.initPeriod()
    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
    joint_q[5] += 0.001
    rtde_c.waitPeriod(t_start)

rtde_c.servoStop()

actual_q = rtde_r.getActualQ()
print(f"机械臂当前关节角度：{actual_q}")
actual_tcp = rtde_r.getActualTCPPose()
print(f"机械臂当前位置：{actual_tcp}")
rtde_c.stopScript()

rtde_c.disconnect()
rtde_r.disconnect()