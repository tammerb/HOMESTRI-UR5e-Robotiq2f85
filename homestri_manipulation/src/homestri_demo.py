#! /usr/bin/env python3

import rospy
from move_groups import Arm, Gripper
from tf.transformations import quaternion_from_euler
import tf
import geometry_msgs.msg 
import controller_manager_msgs.srv

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
        poseStamped.pose = pose
        return poseStamped

def switch_controllers(start_controllers, stop_controllers):
    rospy.wait_for_service('/controller_manager/switch_controller')
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

    arm = Arm('arm')
    gripper = Gripper('gripper')

    gripper.move_to_position(0.471239)

    pose = create_pose(0.855, .105, .916, 1.5707, 0, 0)

    arm.move_to_pose(pose, 0.1)

    arm.move_to_offset(0.1, 0, 0)

    gripper.move_to_position(1.18)

    
    # switch_controllers(
    #     ['cartesian_compliance_controller'], 
    #     ['scaled_pos_joint_traj_controller']
    # )

    # pub = rospy.Publisher('/target_frame', geometry_msgs.msg.PoseStamped, queue_size=1)
    # pose = create_pose(0.10518, -.87766, 0.17319, 0, 0, 0)#, frame='base_link')
    # arm.set_pose_reference_frame('base_link')
    # arm.move_to_pose(pose, 0)



    # listener = tf.TransformListener()

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     pub.publish(pose)
    #     rate.sleep()


    rospy.spin()


