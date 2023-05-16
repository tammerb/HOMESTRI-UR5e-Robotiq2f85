#! /usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import actionlib

from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped, Pose, Wrench, Transform, Vector3, Quaternion
from homestri_msgs.msg import ComplianceControlAction, ComplianceControlGoal

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

import sys

from geometry_msgs_utils import create_pose, stamp_pose, create_wrench, stamp_transform, stamp_wrench, pose_to_matrix, matrix_to_pose, rotational_error, translational_error


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
    rospy.init_node('cartesian_control_action_server')
    # compliance_control_server = ComplianceControlActionServer('compliance_control')


    # goal = ComplianceControlGoal()
    # goal.pose = create_pose(0.3, 0, 0.3, 0,0,0,1)
    # goal.frame_id = "base_link"
    # goal.mode = ComplianceControlGoal.MODE_TARGET
    # goal.offset = create_pose(-.1,0,1,0, 0, 0.7071068, 0.7071068 )

    # compliance_control_server.execute_cb(goal)


    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()


    try:
        trans = tf_buffer.lookup_transform(
            'base', 'tool0_controller', rospy.Time(0), timeout=rospy.Duration(3.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        sys.exit()

    pose = Pose()
    pose.position = trans.transform.translation
    pose.orientation = trans.transform.rotation

    

    pose.position.z += 0.05

    print(pose)

    # make sure the correct controller is loaded and activated
    goal = FollowCartesianTrajectoryGoal()
    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_cartesian_trajectory".format("forward_cartesian_traj_controller"),
        FollowCartesianTrajectoryAction,
    )

    # Wait for action server to be ready
    timeout = rospy.Duration(5)
    if not trajectory_client.wait_for_server(timeout):
        rospy.logerr("Could not reach controller action server.")
        sys.exit(-1)

    # The following list are arbitrary positions
    # Change to your own needs if desired
    pose_list = [
        pose,
    ]
    duration_list = [10.0]
    for i, pose in enumerate(pose_list):
        point = CartesianTrajectoryPoint()
        point.pose = pose
        point.time_from_start = rospy.Duration(duration_list[i])
        goal.trajectory.points.append(point)

    trajectory_client.send_goal(goal)
    trajectory_client.wait_for_result()

    result = trajectory_client.get_result()

    rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))


    rospy.spin()