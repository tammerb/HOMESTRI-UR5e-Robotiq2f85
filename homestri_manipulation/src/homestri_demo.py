#! /usr/bin/env python3

import rospy
from move_groups import Arm, Gripper
from tf.transformations import quaternion_from_euler
import tf
import geometry_msgs.msg 
import moveit_msgs.msg
import controller_manager_msgs.srv
import math

def create_pose(x, y, z, roll, pitch, yaw, frame = ""):
    pose = geometry_msgs.msg.Pose()
    quat_tf = quaternion_from_euler(roll, pitch, yaw)
    orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = orientation

    if frame == "":
        return pose
    else:
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id = frame
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = pose
        return poseStamped

def switch_controllers(start_controllers, stop_controllers):
    rospy.wait_for_service('/controller_manager/switch_controller', timeout=3)
    
    try:
        switch_controller = rospy.ServiceProxy(
            '/controller_manager/switch_controller', 
            controller_manager_msgs.srv.SwitchController
        )

        req = controller_manager_msgs.srv.SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = 2

        res = switch_controller(req)

        print(res)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def zero_ft_sensor():
    pass

if __name__ == "__main__":

    rospy.init_node('homestri_demo')
    frame_pub = rospy.Publisher('/target_frame', geometry_msgs.msg.PoseStamped, queue_size=1)

    arm = Arm('arm')
    gripper = Gripper('gripper')

    # set vel/acc scaling
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    # open gripper slightly
    gripper.move_to_position(0.471239)

    # pregrasp position
    constraints = moveit_msgs.msg.Constraints()
    joint_constraint = moveit_msgs.msg.JointConstraint()
    joint_constraint.joint_name = 'wrist_1_joint'
    joint_constraint.position = -math.pi/2
    joint_constraint.tolerance_above = math.pi/2
    joint_constraint.tolerance_below = math.pi/2
    joint_constraint.weight = 10
    constraints.joint_constraints = [joint_constraint]
    arm.set_path_constraints(constraints)
    pose = create_pose(0.855, .105, .916, 1.5707, 0, 0)
    arm.move_to_pose(pose, 0.15)
    arm.clear_path_constraints()

    # grasp approach
    arm.move_to_offset(0.15, 0, 0)

    # close gripper
    gripper.move_to_target('close')

    # switch to compliance controller
    switch_controllers(
        ['cartesian_compliance_controller'], 
        ['scaled_pos_joint_traj_controller']
    )

    # peel build plate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cur_pose = arm.get_current_pose()
        if cur_pose.pose.position.z >= 0.97:
            break

        pose = create_pose(0.865, .105, 1, 1.5707, .25, 0, 'world')
        frame_pub.publish(pose)
        rate.sleep()

    # switch back to trajectory controller
    switch_controllers(
        ['scaled_pos_joint_traj_controller'],
        ['cartesian_compliance_controller']        
    )

    # raise plate
    arm.move_to_offset(0, .1, 0)

    input("PRESS ENTER TO LOWER.")

    # lower plate
    arm.move_to_offset(0, -.1, 0)

    # switch to compliance controller
    switch_controllers(
        ['cartesian_compliance_controller'], 
        ['scaled_pos_joint_traj_controller']
    )

    # attach build plate
    frame_pub = rospy.Publisher('/target_frame', geometry_msgs.msg.PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cur_pose = arm.get_current_pose()
        if cur_pose.pose.position.z < .917:
            break

        pose = create_pose(0.855, .105, .916, 1.5707, 0, 0, 'world')
        frame_pub.publish(pose)
        rate.sleep()

    # switch back to trajectory controller
    switch_controllers(
        ['scaled_pos_joint_traj_controller'],
        ['cartesian_compliance_controller']        
    )

    # open gripper slightly
    gripper.move_to_position(0.471239)

    # grasp retreat
    arm.move_to_offset(-0.15, 0, 0)

    # go to home
    arm.move_to_target('home')

    print('FINISHED!')

    # rospy.spin()


