#ifndef CONDITION_NODE_H
#define CONDITION_NODE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_ros/bt_subscriber_node.h>

#include "geometry_msgs/Twist.h"

#include <homestri_behavior_trees/node_input_conversions.h>

class GraspedCondition: public RosSubscriberNode<geometry_msgs::Twist>
{

public:
  GraspedCondition( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosSubscriberNode<geometry_msgs::Twist>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return { };
  }

  NodeStatus onFailedReceive() override 
  {

    return NodeStatus::SUCCESS;
  }

  NodeStatus onReceive(const MessageType& msg) override {
    double x = msg.linear.x;
    if (x > 0) {
      return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
  }
};

#endif