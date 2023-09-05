import rospy
import moveit_commander
from homestri_planning_interface.srv import SimplePlan, SimplePlanRequest, SimplePlanResponse

class PlanningInterface:
    def __init__(self):
        # Retrieve the arm and gripper move group names from the parameter server
        name = rospy.get_name()
        move_group_name = rospy.get_param(name + '/move_group')
        self.default_vel_scaling = rospy.get_param(name + '/default_vel_scaling')
        self.default_acc_scaling = rospy.get_param(name + '/default_acc_scaling')

        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)

        self.plan_srv = rospy.Service(name + '/simple_plan', SimplePlan, self.simple_plan_cb)

    def simple_plan_cb(self, req):
        mode = req.mode
        target = req.target
        pose = req.pose
        vel_scaling = req.vel_scaling
        acc_scaling = req.acc_scaling

        # set start state to current state
        self.move_group.set_start_state_to_current_state()

        # set velocity and acceleration scaling factors
        if vel_scaling == 0:
            vel_scaling = self.default_vel_scaling
        if acc_scaling == 0:
            acc_scaling = self.default_acc_scaling
        self.move_group.set_max_velocity_scaling_factor(vel_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acc_scaling)

        if mode == SimplePlanRequest.POSE:
            self.move_group.set_planning_pipeline_id("chomp")
            self.move_group.set_planner_id("CHOMP")
            self.move_group.set_pose_target(pose)
        elif mode == SimplePlanRequest.POSE_LINE:
            self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            self.move_group.set_planner_id("LIN")
            self.move_group.set_joint_value_target(target)
        elif mode == SimplePlanRequest.TARGET:
            self.move_group.set_planning_pipeline_id("chomp")
            self.move_group.set_planner_id("CHOMP")
            self.move_group.set_named_target(target)

        success, robot_traj, time, error_code = self.move_group.plan()

        res = SimplePlanResponse()

        if not success:
            rospy.logerr("Planning failed with error code: " + str(error_code))
            res.success = False
            return res
        
        res.joint_trajectory = robot_traj.joint_trajectory
        res.success = True
        
        return res
    