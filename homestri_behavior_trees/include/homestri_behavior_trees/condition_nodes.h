#ifndef CONDITION_NODE_H
#define CONDITION_NODE_H

#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_subscriber_node.h>
#include <homestri_behavior_trees/node_input_conversions.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>

class GraspedCondition: public RosSubscriberNode<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>
{

public:
  GraspedCondition( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosSubscriberNode<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return { };
  }

  NodeStatus onFailedReceive() override 
  {

    return NodeStatus::SUCCESS;
  }

  NodeStatus onReceive(const MessageType& msg) override {
    int gOBJ = msg.gOBJ;
    int gGTO = msg.gGTO;

    if (gOBJ == 2 && gGTO == 1) {
      return NodeStatus::SUCCESS; 
    }

    return NodeStatus::FAILURE;
  }
};

#endif