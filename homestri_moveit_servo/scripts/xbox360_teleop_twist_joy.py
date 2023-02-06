#! /usr/bin/env python3

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 
from threading import Lock
from copy import deepcopy

class TeleopTwistJoy():
    def __init__(self):
        self.twist_pub = rospy.Publisher('/servo_server/cmd_vel', geometry_msgs.msg.TwistStamped, queue_size=1)
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
            
            for i in range(len(axes)):
                if abs(axes[i]) < 0.15:
                    axes[i] = 0

            twist_stamped.twist.linear.x = axes[1]
            twist_stamped.twist.linear.y = axes[0]
            twist_stamped.twist.linear.z = (1-(axes[5]+1)/2)-(1-(axes[2]+1)/2)
            twist_stamped.twist.angular.x = 0 
            twist_stamped.twist.angular.y = 0 
            twist_stamped.twist.angular.z = 0

            self.twist_pub.publish(twist_stamped)

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