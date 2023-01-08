#! /usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]    

class Arm(moveit_commander.MoveGroupCommander):
    def __init__(self, name):
        super().__init__(name)
        self.set_max_velocity_scaling_factor(1.0)
        self.set_max_acceleration_scaling_factor(1.0)

    # Moves to a Pose
    def move_to_pose(self, pose):
        self.set_pose_target(pose)

        error_code_val, plan, planning_time, error_code = self.plan()
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        error_code_val = self.execute(plan)
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        return True

    # Moves to a target joint position
    def move_to_target(self, target):
        self.set_named_target(target)

        error_code_val, plan, planning_time, error_code = self.plan()
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        error_code_val = self.execute(plan)
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        return True

    def pregrasp_approach(self, distance):
        pose = self.get_current_pose().pose

        q = pose.orientation
        offset = qv_mult(
            [q.x, q.y, q.z, q.w], 
            [distance, 0, 0]
        )

        pose.position.x += offset[0]
        pose.position.y += offset[1]
        pose.position.z += offset[2]

        success = self.move_cartesian_path(pose)
        return success

    def move_cartesian_path(self, pose):
        waypoints = []
        waypoints.append(pose)

        (plan, fraction) = self.compute_cartesian_path(
            waypoints,
            eef_step=0.01,
            jump_threshold=0, # 0?
            avoid_collisions=True
        )

        if fraction < 1:
            return False
        
        error_code_val = self.execute(plan)
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        return True

class Gripper(moveit_commander.MoveGroupCommander):
    def __init__(self, name):
        super().__init__(name)

    # Moves gripper to joint position
    def move_to_position(self, pos): 
        joint_goal = self.get_current_joint_values()
        joint_goal = [pos] * len(joint_goal)

        self.set_joint_value_target(joint_goal)

        error_code_val, plan, planning_time, error_code = self.plan()
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        error_code_val = self.execute(plan)
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        return True

    # Moves gripper to joint position
    def move_to_target(self, target): 
        self.set_named_target(target)

        error_code_val, plan, planning_time, error_code = self.plan()
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        error_code_val = self.execute(plan)
        if error_code_val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return False

        return True

    

