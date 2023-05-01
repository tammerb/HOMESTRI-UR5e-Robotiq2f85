#! /usr/bin/env python3

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
from threading import Lock
from copy import deepcopy

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

class TeleopTwistJoy():
    def __init__(self):
        self.twist_pub = rospy.Publisher('/servo_server/cmd_vel', geometry_msgs.msg.TwistStamped, queue_size=1)
        self.wrench_pub = rospy.Publisher('/cartesian_force_controller/target_wrench', geometry_msgs.msg.WrenchStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher('/robotiq_2f_85_gripper/control', Robotiq2FGripper_robot_output, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)
        
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

    def publish(self):
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()

        if msg != None:
            twist_stamped = geometry_msgs.msg.TwistStamped()
            twist_stamped.header.stamp = rospy.Time.now()

            axes = list(self.joy_msg.axes)
            buttons = list(self.joy_msg.buttons)
            
            for i in range(len(axes)):
                if abs(axes[i]) < 0.15:
                    axes[i] = 0

            twist_stamped.twist.linear.x = axes[1]
            twist_stamped.twist.linear.y = axes[0]
            twist_stamped.twist.linear.z = (1-(axes[5]+1)/2)-(1-(axes[2]+1)/2)
            twist_stamped.twist.angular.x = -axes[3] 
            twist_stamped.twist.angular.y = axes[4]
            twist_stamped.twist.angular.z = buttons[2] - buttons[3]


            command = Robotiq2FGripper_robot_output()
            command.rACT = 0x1
            command.rGTO = 0x1 # go to position
            command.rATR = 0x0 # No emergency release
            command.rSP = 128 # speed
            command.rPR = 0x0 # position
            command.rFR = 30 # effort

            
            if buttons[0] == 1:
                command.rPR = 230 # position
                self.gripper_pub.publish(command)
            elif buttons[1] == 1:
                command.rPR = 0 # position
                self.gripper_pub.publish(command)

            wrench_stamped = geometry_msgs.msg.WrenchStamped()
            wrench_stamped.wrench.force.x = twist_stamped.twist.linear.x *20
            wrench_stamped.wrench.force.y = twist_stamped.twist.linear.y *20
            wrench_stamped.wrench.force.z = twist_stamped.twist.linear.z *20
            wrench_stamped.wrench.torque.x = twist_stamped.twist.angular.x 
            wrench_stamped.wrench.torque.y = twist_stamped.twist.angular.y 
            wrench_stamped.wrench.torque.z = twist_stamped.twist.angular.z

            self.twist_pub.publish(twist_stamped)
            self.wrench_pub.publish(wrench_stamped)

    def callback(self, msg):
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()
        


if __name__ == '__main__':
    rospy.init_node('xbox360_teleop_twist_joy')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        teleop_twist_joy.publish()
        rate.sleep()
    
    rospy.spin()