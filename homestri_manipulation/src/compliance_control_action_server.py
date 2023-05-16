#! /usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import numpy as np
import quaternion

import actionlib

from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped, Pose, Wrench, Transform, Vector3, Quaternion
from homestri_msgs.msg import ComplianceControlAction, ComplianceControlGoal

import sys

def create_pose(tx, ty, tz, rx, ry, rz, rw):
    pose = Pose()
    pose.position.x = tx
    pose.position.y = ty
    pose.position.z = tz
    pose.orientation.x = rx
    pose.orientation.y = ry
    pose.orientation.z = rz
    pose.orientation.w = rw

    return pose

def stamp_pose(pose, frame="", time=None):
    if time == None: time = rospy.Time.now()

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


def stamp_wrench(wrench, frame="", time=None):
    if time == None: time = rospy.Time.now()

    wrenchStamped = WrenchStamped()
    wrenchStamped.header.frame_id = frame
    wrenchStamped.header.stamp = time
    wrenchStamped.wrench = wrench
    return wrenchStamped

def stamp_transform(transform, child_frame_id="", frame="", time=None):
    if time == None: time = rospy.Time.now()

    transformStamped = TransformStamped()
    transformStamped.child_frame_id = child_frame_id
    transformStamped.header.frame_id = frame
    transformStamped.header.stamp = time
    transformStamped.transform = transform
    return transformStamped

def pose_to_matrix(p):
    # create trans and rot vectors
    trans = [p.position.x, p.position.y, p.position.z]
    rot = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    # numpy arrays to 4x4 transform matrix 
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)

    mat = np.dot(trans_mat, rot_mat)

    return mat, trans_mat, rot_mat

def matrix_to_pose(mat):
    rot = tf.transformations.quaternion_from_matrix(mat)
    trans = tf.transformations.translation_from_matrix(mat)

    pose = create_pose(
            trans[0],
            trans[1],
            trans[2],
            rot[0],
            rot[1],
            rot[2],
            rot[3]
        )

    return pose

def rotational_error(q1, q2):
    q1 = np.quaternion(q1.x, q1.y, q1.z, q1.w)
    q2 = np.quaternion(q2.x, q2.y, q2.z, q2.w)

    q_err = q1 * q2.conjugate()

    angle_of_rotation = 2 * np.arccos(q_err.w)

    q_err_norm = q_err / np.linalg.norm(quaternion.as_float_array(q_err))
    axis = np.array([q_err_norm.x, q_err_norm.y, q_err_norm.z])

    axis_of_rotation = angle_of_rotation * axis
    axis_of_rotation_magnitude = np.linalg.norm(axis_of_rotation)

    return angle_of_rotation, axis_of_rotation_magnitude

def translational_error(t1, t2):
    t1 = np.array((t1.x, t1.y, t1.z))
    t2 = np.array((t2.x, t2.y, t2.z))

    dist = np.linalg.norm(t1 - t2)

    return dist


class ComplianceControlActionServer(object):
    def __init__(self, name):
        self.end_effector_link = 'gripper_tip_link'
        self.base_link = 'base_link'
        self.trans_goal_tolerance = 0.01
        self.rot_goal_tolerance = np.pi/12

        self.tf_timeout = rospy.Duration(3.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.pose_pub = rospy.Publisher(
            '/target_frame', PoseStamped, queue_size=1)
        self.wrench_pub = rospy.Publisher(
            '/target_wrench', WrenchStamped, queue_size=1)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ComplianceControlAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        pose = goal.pose
        offset = goal.offset
        frame_id = goal.frame_id
        wrench = goal.wrench
        mode = goal.mode

        target_wrench_stamped = stamp_wrench(wrench, frame=self.base_link)

        if mode == ComplianceControlGoal.MODE_OFFSET:
            eef_pose = self.__lookup_pose( frame_id, self.end_effector_link)
            _, eef_trans_mat, eef_rot_mat = pose_to_matrix(eef_pose)
            offset_mat, _, _= pose_to_matrix(offset)

            target_mat = np.dot(eef_trans_mat, np.dot(offset_mat, eef_rot_mat))
            target_pose = matrix_to_pose(target_mat)
            target_pose_stamped = stamp_pose(target_pose, frame=frame_id, time=rospy.Time(0))

            target_pose_stamped = self.tf_buffer.transform(target_pose_stamped, self.base_link, timeout=rospy.Duration(3.0))

        elif mode == ComplianceControlGoal.MODE_TARGET:
            pose_mat, _, _= pose_to_matrix(pose)
            offset_mat, _, _= pose_to_matrix(offset)

            target_mat = np.dot(pose_mat, offset_mat)
            target_pose = matrix_to_pose(target_mat)
            target_pose_stamped = stamp_pose(target_pose, frame=frame_id, time=rospy.Time(0))
            
            target_pose_stamped = self.tf_buffer.transform(target_pose_stamped, self.base_link, timeout=rospy.Duration(3.0))


        # helper variables
        r = rospy.Rate(10)
        reached_target = False
        success = True

        # start executing the action
        while not reached_target and not rospy.is_shutdown():
            target_pose = target_pose_stamped.pose
            current_pose = self.__lookup_pose(self.base_link, self.end_effector_link)
            if current_pose == None:
                rospy.loginfo(
                    '%s: failed to read end effector frame.' % self._action_name)
                self._as.set_aborted()
                success = False
                break

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

            rot_err, axis_err = rotational_error(
                current_pose.orientation, target_pose.orientation)
            trans_err = translational_error(
                current_pose.position, target_pose.position)
            
            print(f"trans_err {trans_err}, rot_err {rot_err}, axis_err {axis_err}")

            if rot_err < self.rot_goal_tolerance and \
               axis_err < self.rot_goal_tolerance and \
               trans_err < self.trans_goal_tolerance:
                success = True
                reached_target = True
                break
            
            t = Transform()
            t.translation = target_pose_stamped.pose.position
            t.rotation = target_pose_stamped.pose.orientation
            t = stamp_transform(t, child_frame_id="target_frame", frame=self.base_link)
            self.broadcaster.sendTransform(t)

            self.pose_pub.publish(target_pose_stamped)
            self.wrench_pub.publish(target_wrench_stamped)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
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

if __name__ == "__main__":
    rospy.init_node('compliance_control_action_server')
    compliance_control_server = ComplianceControlActionServer('compliance_control')


    goal = ComplianceControlGoal()
    goal.pose = create_pose(0.3, 0, 0.3, 0,0,0,1)
    goal.frame_id = "base_link"
    goal.mode = ComplianceControlGoal.MODE_TARGET
    goal.offset = create_pose(-.1,0,1,0, 0, 0.7071068, 0.7071068 )

    compliance_control_server.execute_cb(goal)


    rospy.spin()