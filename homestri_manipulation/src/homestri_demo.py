#! /usr/bin/env python3

import rospy
from move_groups import Arm, Gripper
from tf.transformations import quaternion_from_euler
import tf
import geometry_msgs.msg 
import moveit_msgs.msg
import controller_manager_msgs.srv
import std_srvs.srv
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
    
def create_wrench(fx, fy, fz, tx, ty, tz, frame = ""):
    wrench = geometry_msgs.msg.Wrench()
    wrench.force.x = fx
    wrench.force.y = fy
    wrench.force.z = fz
    wrench.torque.x = tx
    wrench.torque.y = ty
    wrench.torque.z = tz

    if frame == "":
        return wrench
    else:
        wrenchStamped = geometry_msgs.msg.WrenchStamped()
        wrenchStamped.header.frame_id = frame
        wrenchStamped.header.stamp = rospy.Time.now()
        wrenchStamped.wrench = wrench
        return wrenchStamped

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
    rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor', timeout=3)
    
    try:
        zero_ft_sensor_srv = rospy.ServiceProxy(
            '/ur_hardware_interface/zero_ftsensor', 
            std_srvs.srv.Trigger
        )

        req = std_srvs.srv.TriggerRequest()

        res = zero_ft_sensor_srv(req)

        print(res)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    x = 0.85
    y = 0.11
    z = 0.917

    x_offset = 0.005
    z_offset = 0.064



    rospy.init_node('homestri_demo')
    frame_pub = rospy.Publisher('/target_frame', geometry_msgs.msg.PoseStamped, queue_size=1)
    wrench_pub = rospy.Publisher('/target_wrench', geometry_msgs.msg.WrenchStamped, queue_size=1)

    arm = Arm('arm')
    gripper = Gripper('gripper')

    # set vel/acc scaling
    arm.set_max_velocity_scaling_factor(0.05)
    arm.set_max_acceleration_scaling_factor(0.05)

    input("PRESS ENTER TO GO TO PREGRASP POSITION.")

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
    pose = create_pose(x, y, z, 1.5707, 0, 0)
    if not arm.move_to_pose(pose, 0.15): raise Exception("failed move to pose") 
    arm.clear_path_constraints()

    input("PRESS ENTER TO OPEN GRIPPER.")

    # open gripper slightly
    if not gripper.move_to_position(0.471239): raise Exception("failed open gripper")

    input("PRESS ENTER TO APPROACH PLATE.")

    # grasp approach
    if not arm.move_to_offset(0.15, 0, 0): raise Exception("failed move to offset")

    input("PRESS ENTER TO ZERO FORCE TORQUE SENSOR.")

    zero_ft_sensor()

    input("PRESS ENTER TO CLOSE GRIPPER.")

    # close gripper
    if not gripper.move_to_target('close'): raise Exception("failed close gripper")

    input("PRESS ENTER TO PEEL PLATE.")

    # switch to compliance controller
    switch_controllers(
        ['cartesian_compliance_controller'], 
        ['scaled_pos_joint_traj_controller']
    )

    # rospy.sleep(2)

    # peel build plate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cur_pose = arm.get_current_pose()
        print(cur_pose.pose.position.z)
        if cur_pose.pose.position.z >= z + z_offset - 0.008:
            break

        pose = create_pose(x + x_offset, y, z + z_offset, 1.5707, .3, 0, 'world')
        wrench = create_wrench(0, 20, 0, 0, 0, 0, "world")
        frame_pub.publish(pose)
        wrench_pub.publish(wrench)
        rate.sleep()

    # switch back to trajectory controller
    switch_controllers(
        ['scaled_pos_joint_traj_controller'],
        ['cartesian_compliance_controller']        
    )

    # rospy.sleep(2)
    # pose = create_pose(x + x_offset, y, z + z_offset, 1.5707, .3, 0)
    # if not arm.move_cartesian_path(pose): raise Exception("failed move to pose") 

    input("PRESS ENTER TO RAISE.")

    # raise plate
    pose = create_pose(x + x_offset, y, z+z_offset+.1, 1.5707, .3, 0)
    if not arm.move_cartesian_path(pose): raise Exception("failed move cartesian path")

    input("PRESS ENTER TO LOWER.")

    # lower plate
    pose = create_pose(x + x_offset, y, z + z_offset, 1.5707, .3, 0)
    if not arm.move_cartesian_path(pose): raise Exception("failed move cartesian path")

    input("PRESS ENTER TO PLACE PLATE.")

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
        print(cur_pose.pose.position.z)
        if cur_pose.pose.position.z < z + 0.005:
            break
        
        wrench = create_wrench(0, 0, 0, 0, 0, 0, "world")
        pose = create_pose(x, y, z, 1.5707, 0, 0, 'world')
        frame_pub.publish(pose)
        wrench_pub.publish(wrench)
        rate.sleep()

    # switch back to trajectory controller
    switch_controllers(
        ['scaled_pos_joint_traj_controller'],
        ['cartesian_compliance_controller']        
    )

    input("PRESS ENTER TO OPEN GRIPPER.")

    # open gripper slightly
    while ((not gripper.move_to_position(0.471239)) and (not rospy.is_shutdown())):
        pass
    
    input("PRESS ENTER TO RETREAT.")

    # grasp retreat
    if not arm.move_to_offset(-0.15, 0, 0): raise Exception("failed move to offset")

    input("PRESS ENTER TO GO TO HOME.")

    # go to home
    if not arm.move_to_target('home'): raise Exception("failed move to home")

    print('FINISHED!')

    # rospy.spin()


