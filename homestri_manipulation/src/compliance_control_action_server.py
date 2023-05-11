#! /usr/bin/env python3

import rospy
import tf
import tf.transformations
import numpy as np
import quaternion

import actionlib

from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose, Wrench
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
    if time == None: time = rospy.Time(0)

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
    if time == None: time = rospy.Time(0)

    wrenchStamped = WrenchStamped()
    wrenchStamped.header.frame_id = frame
    wrenchStamped.header.stamp = time
    wrenchStamped.wrench = wrench
    return wrenchStamped


class ComplianceControlActionServer(object):
    def __init__(self, name):
        self.end_effector_link = 'gripper_tip_link'
        self.base_link = 'base_link'
        self.trans_goal_tolerance = 0.01
        self.rot_goal_tolerance = np.pi/12

        self.tf_listener = tf.TransformListener()

        self.pose_pub = rospy.Publisher(
            '/target_frame', PoseStamped, queue_size=1)
        self.wrench_pub = rospy.Publisher(
            '/target_wrench', WrenchStamped, queue_size=1)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ComplianceControlAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):



        eef_pose = self.__lookup_transform(goal.frame_id, self.end_effector_link)

        print(eef_pose)

        target_pose = self.__transform_pose_with_pose(goal.pose, eef_pose)

        print(target_pose)

        target_pose = self.__transform_pose(target_pose, goal.frame_id, self.base_link)


        print(target_pose)

        sys.exit()





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

    def __lookup_transform(self, target_frame, source_frame):        
        try:
            (trans,rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))

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

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        
    def __transform_pose(self, pose, current_frame, target_frame):


        transform = self.__lookup_transform(current_frame, target_frame)

        new_pose = self.__transform_pose_with_pose(transform, pose)


        return new_pose


        
    def __transform_pose_with_pose(self, p1, p2):
        mat1 = self.__pose_to_matrix(p1)
        mat2 = self.__pose_to_matrix(p2)

        new_mat = np.matmul(mat1, mat2)

        new_p = self.__matrix_to_pose(new_mat)

        return new_p

    def __pose_to_matrix(self, p):
        # create trans and rot vectors
        trans = [p.position.x, p.position.y, p.position.z]
        rot = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        # numpy arrays to 4x4 transform matrix 
        trans_mat = tf.transformations.translation_matrix(trans)
        rot_mat = tf.transformations.quaternion_matrix(rot)
        # create a 4x4 matrix
        mat = np.dot(trans_mat, rot_mat)

        return mat
    
    def __matrix_to_pose(self, mat):
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
    rospy.init_node('compliance_control_action_server')
    compliance_control_server = ComplianceControlActionServer('compliance_control')


    goal = ComplianceControlGoal()
    goal.pose = create_pose(1,0,0,0,0,0,1)
    goal.frame_id = "gripper_tip_link"

    compliance_control_server.execute_cb(goal)


    rospy.spin()