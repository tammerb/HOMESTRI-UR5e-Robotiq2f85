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

        twist_topic = rospy.get_param("~twist_topic", "/joy/twist")

        self.twist_pub = rospy.Publisher(twist_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)
        
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

    def publish(self):
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()

        if msg != None:
            twist = geometry_msgs.msg.Twist()

            axes = list(self.joy_msg.axes)
            buttons = list(self.joy_msg.buttons)
            
            for i in range(len(axes)):
                if abs(axes[i]) < 0.15:
                    axes[i] = 0

            linear_scale = 0.05
            angular_scale = 0.05

            twist.linear.x = (axes[1])  * linear_scale
            twist.linear.y = (axes[0])  * linear_scale
            twist.linear.z = ((1-(axes[5]+1)/2)-(1-(axes[2]+1)/2))  * linear_scale
            twist.angular.x = (-axes[3])   * angular_scale
            twist.angular.y = (axes[4])  * angular_scale
            twist.angular.z = (buttons[2] - buttons[3])  * angular_scale

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