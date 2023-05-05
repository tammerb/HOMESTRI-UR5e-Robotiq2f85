#! /usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import quaternion

import actionlib

from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose, Wrench
from homestri_msgs.msg import ComplianceControlAction


def stamp_pose(pose, frame="", time=rospy.Time.now()):
    poseStamped = PoseStamped()
    poseStamped.header.frame_id = frame
    poseStamped.header.stamp = time
    poseStamped.pose = pose
    return poseStamped


def create_wrench(fx, fy, fz, tx, ty, tz):
    wrench = Wrench()
    wrench.force.x = fx
    wrench.force.y = fy
    wrench.force.z = fz
    wrench.torque.x = tx
    wrench.torque.y = ty
    wrench.torque.z = tz

    return wrench


def stamp_wrench(wrench, frame="", time=rospy.Time.now()):
    wrenchStamped = WrenchStamped()
    wrenchStamped.header.frame_id = frame
    wrenchStamped.header.stamp = time
    wrenchStamped.wrench = wrench
    return wrenchStamped


class ComplianceControlActionServer(object):
    def __init__(self, name):
        self.end_effector_link = 'gripper_tip_link'
        self.base_link = 'base_link'
        self.tf_timeout = rospy.Duration(3.0)
        self.trans_goal_tolerance = 0.01
        self.rot_goal_tolerance = np.pi/12

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pose_pub = rospy.Publisher(
            '/target_frame', PoseStamped, queue_size=1)
        self.wrench_pub = rospy.Publisher(
            '/target_wrench', WrenchStamped, queue_size=1)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ComplianceControlAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(10)
        reached_target = False
        success = True

        target_pose = None
        target_pose_stamped = None
        target_wrench = None
        target_wrench_stamped = None

        # start executing the action
        while not reached_target:

            current_frame = self.__lookup_transform()
            if current_frame == None:
                rospy.loginfo(
                    '%s: failed to read end effector frame.' % self._action_name)
                self._as.set_aborted()
                success = False
                break

            current_pose = Pose()
            current_pose.position = current_frame.translation
            current_pose.orientation = current_frame.rotation

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)

                current_pose_stamped = stamp_pose(
                    current_pose, frame=self.base_link)

                zero_wrench = Wrench(0, 0, 0, 0, 0, 0)
                zero_wrench_stamped = stamp_wrench(
                    zero_wrench, frame=self.base_link)

                self.pose_pub.publish(current_pose_stamped)
                self.wrench_pub.publish(zero_wrench_stamped)

                self._as.set_preempted()
                success = False
                break

            rot_err, axis_err = self.__rotational_error(
                current_pose.orientation, target_pose.orientation)
            trans_err = self.__translational_error(
                current_pose.position, target_pose.position)

            if rot_err < self.rot_goal_tolerance and \
               axis_err < self.rot_goal_tolerance and \
               trans_err < self.trans_goal_tolerance:
                success = True
                reached_target = True
                break

            self.pose_pub.publish(target_pose_stamped)
            self.wrench_pub.publish(target_wrench_stamped)

            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded()

    def __lookup_transform(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                self.end_effector_link, self.base_link, rospy.Time(), timeout=self.tf_timeout)
            return trans.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def __rotational_error(self, q1, q2):
        q1 = np.quaternion(q1.x, q1.y, q1.z, q1.w)
        q2 = np.quaternion(q2.x, q2.y, q2.z, q2.w)

        q_err = q1 * q2.conjugate()

        angle_of_rotation = 2 * np.arccos(q_err.w)

        q_err_norm = q_err / np.linalg.norm(quaternion.as_float_array(q_err))
        axis = np.array([q_err_norm.x, q_err_norm.y, q_err_norm.z])

        axis_of_rotation = angle_of_rotation * axis
        axis_of_rotation_magnitude = np.linalg.norm(axis_of_rotation)

        return angle_of_rotation, axis_of_rotation_magnitude

    def __translational_error(self, t1, t2):
        t1 = np.array((t1.x, t1.y, t1.z))
        t2 = np.array((t2.x, t2.y, t2.z))

        dist = np.linalg.norm(t1 - t2)

        return dist

if __name__ == "__main__":
    rospy.init_node('manipulation_action_server')
    compliance_control_server = ComplianceControlActionServer()
    rospy.spin()