#! /usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import actionlib

from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped, Pose, Wrench, Transform, Vector3, Quaternion
from homestri_msgs.msg import ForceControlAction, ForceControlGoal

from geometry_msgs_utils import create_pose, stamp_pose, create_wrench, stamp_transform, stamp_wrench, pose_to_matrix, matrix_to_pose, rotational_error, translational_error


class ForceControlActionServer(object):
    def __init__(self, name):
        self.end_effector_link = 'gripper_tip_link'
        self.base_link = 'base_link'
        self.stall_speed_threshold = 0.0015
        self.stall_timeout = 1.0

        self.frequency = 15.0

        self.tf_timeout = rospy.Duration(3.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.wrench_pub = rospy.Publisher(
            '/target_wrench', WrenchStamped, queue_size=1)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ForceControlAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        frame_id = goal.frame_id
        wrench = goal.wrench

        # helper variables
        r = rospy.Rate(self.frequency)
        success = True
        stalled = False
        last_movement_time = rospy.Time.now()

        # start executing the action
        while not stalled and not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)

                zero_wrench = create_wrench(0, 0, 0, 0, 0, 0)
                zero_wrench_stamped = stamp_wrench(zero_wrench, frame=self.base_link)
                self.wrench_pub.publish(zero_wrench_stamped)

                self._as.set_preempted()
                success = False
                break

            lin_vel, ang_vel = self.__lookup_speed(self.base_link, self.end_effector_link)

            print(lin_vel, ang_vel)

            time = rospy.Time.now()
            if lin_vel > self.stall_speed_threshold:
                last_movement_time = time
            elif (time - last_movement_time).to_sec() > self.stall_timeout:
                stalled = True
                success = True
                break

            target_wrench_stamped = stamp_wrench(wrench, frame=frame_id, time=rospy.Time(0))
            target_wrench_stamped = self.tf_buffer.transform(target_wrench_stamped, self.end_effector_link, timeout=rospy.Duration(3.0))
            self.wrench_pub.publish(target_wrench_stamped)

            print(target_wrench_stamped)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)

            zero_wrench = create_wrench(0, 0, 0, 0, 0, 0)
            zero_wrench_stamped = stamp_wrench(zero_wrench, frame=self.base_link)
            self.wrench_pub.publish(zero_wrench_stamped)

            self._as.set_succeeded()

    def __lookup_pose(self, target_frame, source_frame):        
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), timeout=self.tf_timeout)
            
            pose = Pose()
            pose.position = trans.transform.translation
            pose.orientation = trans.transform.rotation
            return pose 
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None  

    def __lookup_speed(self, target_frame, source_frame):
        # speed sampling period
        duration = 1/self.frequency

        tfs = self.tf_buffer.lookup_transform_full(source_frame, rospy.Time(0),
                                            source_frame, rospy.Time.now() + rospy.Duration(duration),
                                            target_frame, timeout=self.tf_timeout)

        delta_t = 1.0 / duration
        tr = tfs.transform.translation
        rot = tfs.transform.rotation

        lin_err = np.linalg.norm(np.array((tr.x, tr.y, tr.z)))
        rot_err = 2 * np.arccos(-rot.w) if rot.w < 0 else 2 * np.arccos(rot.w)

        lin_speed = lin_err * delta_t
        rot_speed = rot_err * delta_t

        return lin_speed, rot_speed


if __name__ == "__main__":
    rospy.init_node('force_control_action_server')
    force_control_server = ForceControlActionServer('force_control')


    # goal = ForceControlGoal()
    # goal.frame_id = "world"
    # goal.wrench = create_wrench(-50,0,0,0,0,0)

    # force_control_server.execute_cb(goal)


    rospy.spin()