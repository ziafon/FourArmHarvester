
import rospy
import tkinter as tk
from robot_msgs.msg import messages, hardware

rospy.init_node('robot_console')
pub = rospy.Publisher('robot_command', messages, queue_size=3)
drivePub = rospy.Publisher('robot_control', hardware, queue_size=3)


def start():
    cmd = hardware()
    cmd.motor_mode = [3 for i in range(10)] 
    cmd.motor_control = [15 for i in range(10)] 
    cmd.target_velocity = [0 for i in range(10)]
    drivePub.publish(cmd)

def stop():
    cmd = hardware()
    cmd.motor_mode = [3 for i in range(10)] 
    cmd.motor_control = [6 for i in range(10)] 
    cmd.target_velocity = [0 for i in range(10)]
    drivePub.publish(cmd)

def execute():
    cmd = messages()
    cmd.command = 'excution'
    pub.publish(cmd)
    
def observe():
    cmd = messages()
    cmd.command = 'observation'
    pub.publish(cmd)

def home():
    cmd = messages()
    cmd.command = 'origin'
    pub.publish(cmd)

def distribute():
    cmd = messages()
    cmd.command = 'useFixed'
    pub.publish(cmd)

def zeroback():
    cmd = hardware()
    cmd.motor_mode = [6 for i in range(10)] 
    cmd.motor_control = [31 for i in range(10)] 
    drivePub.publish(cmd)

def plan():
    cmd = messages()
    cmd.command = 'getDistribution'
    pub.publish(cmd)

# 创建窗口
window = tk.Tk()
window.title("按钮示例")

# 创建按钮并分配函数
start_button = tk.Button(window, text="启动", command=start)
start_button.pack()

stop_button = tk.Button(window, text="停止", command=stop)
stop_button.pack()

execute_button = tk.Button(window, text="执行", command=execute)
execute_button.pack()

observe_button = tk.Button(window, text="观测", command=observe)
observe_button.pack()

home_button = tk.Button(window, text="归位", command=home)
home_button.pack()

distribute_button = tk.Button(window, text="固定分布", command=distribute)
distribute_button.pack()

plan_button = tk.Button(window, text="检测目标", command=plan)
plan_button.pack()

plan_button = tk.Button(window, text="回零点", command=zeroback)
plan_button.pack()

# 运行窗口
window.mainloop()
