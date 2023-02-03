#! /usr/bin/env python

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 

def callback(msg):
    
    twist_stamped = geometry_msgs.msg.TwistStamped()
    twist_stamped.header.stamp = rospy.Time.now()

    axes = list(msg.axes)

    for i in range(len(axes)):
        if abs(axes[i]) < 0.15:
            axes[i] = 0

    twist_stamped.twist.linear.x = axes[1]
    twist_stamped.twist.linear.y = axes[0]
    twist_stamped.twist.linear.z = (1-(axes[5]+1)/2)-(1-(axes[2]+1)/2)
    twist_stamped.twist.angular.x = 0 
    twist_stamped.twist.angular.y = 0 
    twist_stamped.twist.angular.z = 0

    twist_pub.publish(twist_stamped)


if __name__ == '__main__':
    rospy.init_node('xbox360_teleop_twist_joy')
    twist_pub = rospy.Publisher('/servo_server/cmd_vel', geometry_msgs.msg.TwistStamped, queue_size=1)
    joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, callback)
    rospy.spin()