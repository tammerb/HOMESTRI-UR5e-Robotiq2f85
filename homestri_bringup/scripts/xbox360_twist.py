#! /usr/bin/env python3

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
from threading import Lock
from copy import deepcopy

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand

class TeleopTwistJoy():
    def __init__(self):
        self.twist_pub = rospy.Publisher('/joy/twist', geometry_msgs.msg.TwistStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher('/onrobot_rg2ft/command', RG2FTCommand, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)
        
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

    def publish(self):
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()

        if msg != None:
            twist = geometry_msgs.msg.TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = "world"

            axes = list(self.joy_msg.axes)
            buttons = list(self.joy_msg.buttons)
            
            for i in range(len(axes)):
                if abs(axes[i]) < 0.15:
                    axes[i] = 0

            linear_scale = 1
            angular_scale = 1

            twist.twist.linear.x = (axes[1])  * linear_scale
            twist.twist.linear.y = (axes[0])  * linear_scale
            twist.twist.linear.z = ((1-(axes[5]+1)/2)-(1-(axes[2]+1)/2))  * linear_scale
            twist.twist.angular.x = (-axes[3])   * angular_scale
            twist.twist.angular.y = (axes[4])  * angular_scale
            twist.twist.angular.z = (buttons[2] - buttons[3])  * angular_scale


            command = RG2FTCommand()
            command.TargetForce = 30
            command.Control = 0x0001
            
            if buttons[0] == 1:
                command.TargetWidth = 0 # position
                self.gripper_pub.publish(command)
            elif buttons[1] == 1:
                command.TargetWidth = 1000 # position
                self.gripper_pub.publish(command)

            self.twist_pub.publish(twist)

    def callback(self, msg):
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()
        


if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        teleop_twist_joy.publish()
        rate.sleep()
    
    rospy.spin()