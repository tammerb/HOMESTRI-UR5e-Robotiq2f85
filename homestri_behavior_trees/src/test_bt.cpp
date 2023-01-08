#include <ros/ros.h>
#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>

#include <homestri_behavior_trees/action_nodes.h>
#include <homestri_behavior_trees/condition_nodes.h>

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;

  std::string xml_filename;
  nh.getParam("/bt_xml", xml_filename);
  std::cout << xml_filename << "********" << std::endl;
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  BehaviorTreeFactory factory;
  RegisterRosAction<ManipulationAction>(factory, "ManipulationAction", nh);
  RegisterRosSubscriber<GraspedCondition>(factory, "GraspedCondition", nh);

  auto tree = factory.createTreeFromFile(xml_filename);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING /*|| status == NodeStatus::FAILURE*/))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    // std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}