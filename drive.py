#!/usr/bin/env python3
# -*- coding: utf-8 -*-


'''
异常处理：在程序中已经有一些异常处理，例如devices_init方法中的try-except语句。但是，还可以在其他关键部分添加更多的异常处理，例如网络连接、数据读取等，以提高程序的健壮性。
代码注释：还不够完善；使用Python的docstring来描述函数的作用、输入参数和返回值等。
函数解耦：进一步将代码的功能划分成更小的函数或方法，以提高代码的可读性和可维护性。
'''


import rospy
from robot_msgs.msg import hardware
import canopen
import _thread
from CANOpenDevice import ServoMotor
from config import motorNum


class Drive():
    def __init__(self):
        rospy.init_node('robot_drive')
        self.network = canopen.Network()
        self.servomotors = [ServoMotor(i+11) for i in range(motorNum)]
        self.devices_init()
        self.rpdo_open()
        self.pub = rospy.Publisher('robot_feedback', hardware, queue_size=10)
        _thread.start_new_thread(self.feedback_publish,())
        rospy.Subscriber('robot_control', hardware, self.callback, queue_size=10)
        rospy.spin()


    """
    处理来自schedule节点和UI节点的操作
    '为每个伺服电机设置模式、控制字和目标速度
    '执行夹持器操作
    """
    def callback(self, msgs):
        if msgs.target_velocity:
            for i in range(motorNum):
                self.servomotors[i].run(msgs.target_velocity[i])
        if msgs.motor_mode:
            for i in range(motorNum):
                self.servomotors[i].mode(msgs.motor_mode[i])
        if msgs.motor_control:
            for i in range(motorNum):
                self.servomotors[i].setCommand(msgs.motor_control[i])
        if msgs.gripper_action:
            gripper[i] = grasp
        

    """
    发布机器人的反馈信息
    '每个伺服电机的位置
    '每个伺服电机的状态
    '其他...
    """
    def feedback_publish(self):
        fb = hardware()
        status = ['0000000000000000' for _ in range(motorNum)]
        limit = ['0000000000000000' for _ in range(motorNum)]
        error = ['0000000000000000' for _ in range(motorNum)]
        position = [0 for _ in range(motorNum)]
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            for i in range(motorNum):
                position[i] = self.servomotors[i].getPulse()
                status[i] = self.servomotors[i].getState()
                limit[i] = self.servomotors[i].getIO()
            fb.actual_position = position
            fb.motor_status = status
            fb.io_status = limit
            fb.error_status = error
            self.pub.publish(fb)


    def devices_init(self):
        # 添加节点
        for i in range(motorNum):
            self.network.add_node(self.servomotors[i].node)
        # 连接CAN总线
        self.network.connect(bustype="canalystii", channel=0, bitrate=500000)
        # 若连接失败，捕获异常，并返回。
        try:
            for i in range(motorNum):
                self.servomotors[i].pdo_mapping()
        except Exception as e:
            print('\033[1;31m 总线连接失败！！！\033[0m\n')
            return
        # 启动CAN总线
        self.network.sync.start(0.1)
        # 使能节点状态
        for i in range(motorNum):
            self.servomotors[i].node.nmt.state = 'OPERATIONAL'
        rospy.sleep(1)
        rospy.loginfo('所有设备已初始化 ！！')


    def rpdo_open(self):
        for i in range(motorNum):
            self.servomotors[i].rpdo_start(0.1)

    def rpdo_close(self):
        for i in range(motorNum):
            self.servomotors[i].rpdo_stop()

    def devices_down(self):
        # 断开CAN总线
        self.rpdo_close()
        self.network.sync.stop()
        self.network.disconnect()


if __name__=='__main__': 
    try:
        Drive()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')
            


















    

    
            


