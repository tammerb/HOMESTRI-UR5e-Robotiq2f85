#! /usr/bin/env python3


import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from sensor_msgs.point_cloud2 import read_points
import sys
import tty
import termios
import select
from cloud_conversions import convertCloudFromRosToOpen3d
import tf2_ros
import tf2_sensor_msgs
from datetime import datetime

point_cloud = None
tf_buffer = None

def pointcloud_callback(msg):
    # Store the PointCloud2 message for later processing
    global point_cloud
    point_cloud = msg


def process_point_cloud():
    global point_cloud, tf_buffer

    if point_cloud is None:
        rospy.loginfo("No PointCloud2 message received yet.")
        return

    # Lookup the transform between the target frame and the source frame
    transform = tf_buffer.lookup_transform("world", point_cloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

    # Transform the PointCloud2 message
    transformed_point_cloud = tf2_sensor_msgs.do_transform_cloud(point_cloud, transform)

    open3d_cloud = convertCloudFromRosToOpen3d(transformed_point_cloud)


    # Process the Open3D point cloud
    # ...

    # save_point_cloud(open3d_cloud)

    rospy.loginfo("Point cloud processed successfully.")

    return open3d_cloud


def save_point_cloud(point_cloud):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"pointcloud_{timestamp}.ply"
    o3d.io.write_point_cloud(filename, point_cloud)
    rospy.loginfo(f"Point cloud saved as {filename}.")


def keyboard_listener():
    # Configure terminal for keyboard input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        combined_pcd = o3d.geometry.PointCloud()
        while not rospy.is_shutdown():
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = ord(sys.stdin.read(1))
                if key == 32:  # Space key
                    combined_pcd += process_point_cloud()
                elif key == 27: # Escape key
                    break
        
        o3d.visualization.draw_geometries([combined_pcd])
        save_point_cloud(combined_pcd)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    global tf_buffer

    rospy.init_node('realsense_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/camera/depth/color/points',
                     PointCloud2, pointcloud_callback)
    keyboard_listener()
    rospy.spin()


if __name__ == '__main__':
    main()
