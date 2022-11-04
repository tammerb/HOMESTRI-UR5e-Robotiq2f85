#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_test");
    ros::NodeHandle nh;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //static const std::string PLANNING_GROUP = "arm";
    //moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//
    //const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ros::shutdown();
    return 0;
}


