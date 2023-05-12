#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml

class StaticTransformPublisher():
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()

        stream = open(rospy.get_param("/static_transform"), 'r')
        self.transforms = yaml.safe_load(stream)

    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            for transform in self.transforms:
                t = TransformStamped()
                t.header.frame_id = transform['frame_id']
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = transform['child_frame_id']
                t.transform.translation.x = transform['x']
                t.transform.translation.y = transform['y']
                t.transform.translation.z = transform['z']
                t.transform.rotation.x = transform['qx']
                t.transform.rotation.y = transform['qy']
                t.transform.rotation.z = transform['qz']
                t.transform.rotation.w = transform['qw']

                self.broadcaster.sendTransform(t)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('homestri_static_transform_publisher')
    object_adder = StaticTransformPublisher()
    object_adder.run()
