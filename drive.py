#!/usr/bin/env python3
# -*- coding: utf-8 -*-


'''
异常处理：在程序中已经有一些异常处理，例如devices_init方法中的try-except语句。但是，还可以在其他关键部分添加更多的异常处理，例如网络连接、数据读取等，以提高程序的健壮性。
代码注释：还不够完善；使用Python的docstring来描述函数的作用、输入参数和返回值等。
类型提示：为函数和方法的参数和返回值添加类型提示，
变量命名：部分变量命名可能不够直观，例如msgs、wkc等
多线程库比较旧：虽然程序中使用了_thread库来实现多线程，但是这是一个较旧的库。可以考虑使用更现代的threading或者concurrent.futures库来替代_thread库。
将硬编码的数字替换为常量或配置文件：在代码中有一些硬编码的数字，类似于Gripper(2)这样，可读性太差，将这些数字替换为常量或者将它们放入配置文件中，可以提高代码的可维护性。
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
    processing the operations from robot_motion and console node
    'set mode for each servo motor
    'set controlword for each servo motor
    'set target velocity for each servo motor
    'gripper operation
    'switch on 
    'switch off
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
        

    """
    publish the feedback of the robot 
    'the position of each servo motor
    'the status of each servo motor
    'other ...
    """
    def feedback_publish(self):
        fb = hardware()
        pos = False
        status = ['0000000000000000' for i in range(motorNum)]
        limit = ['0000000000000000' for i in range(motorNum)]
        error = ['0000000000000000' for i in range(motorNum)]
        position = [0 for i in range(motorNum)]
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
            pos = True if pos == False else False
            self.pub.publish(fb)


    def devices_init(self):
        # add nodes
        for i in range(motorNum):
            self.network.add_node(self.servomotors[i].node)
        # connect from CAN bus
        self.network.connect(bustype="canalystii", channel=0, bitrate=500000)
        # 若连接失败，捕获异常，并返回。
        try:
            # pdo mapping
            for i in range(motorNum):
                self.servomotors[i].pdo_mapping()
        except Exception as e:
            self.error = 1
            print('\033[1;31m{} 总线连接失败！！！\033[0m\n'.format('xie'))
            return
        # start CAN bus
        self.network.sync.start(0.1)
        # change the work state of nodes
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
        # self.grippers.rpdo_stop()

    def devices_down(self):
        # Disconnect from CAN bus
        self.rpdo_close()
        self.network.sync.stop()
        self.network.disconnect()



if __name__=='__main__': 
    try:
        Drive()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')
            


















    

    
            


