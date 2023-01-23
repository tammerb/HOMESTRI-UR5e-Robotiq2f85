#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>

#include <homestri_msgs/ManipulationAction.h>
#include <homestri_msgs/ManipulationGoal.h>
#include <homestri_msgs/ManipulationResult.h>
#include <homestri_msgs/ManipulationFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>

#include <homestri_behavior_trees/node_input_conversions.h>

using namespace BT;

class ManipulationAction: public RosActionNode<homestri_msgs::ManipulationAction>
{

public:
  ManipulationAction( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<homestri_msgs::ManipulationAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return { 
      InputPort<std::string>("id"),
      InputPort<geometry_msgs::Pose>("pose"),
      InputPort<std::string>("target"),
      InputPort<double>("position")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    if (!getInput<std::string>("id", goal.id)) {
      ROS_ERROR("missing required input [id]");
      return false;
    }
    getInput<geometry_msgs::Pose>("pose", goal.pose);
    getInput<std::string>("target", goal.target);
    getInput<double>("position", goal.position);

    ROS_INFO("SENDING GOAL: %s", goal.id.c_str());

    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("SUCCEEDED: %s", res.id.c_str());
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("ManipulationAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("ManipulationAction halted");
      BaseClass::halt();
    }
  }
};


class DetectObject : public SyncActionNode
{
public:
  DetectObject(const std::string& name, const NodeConfiguration & conf)
    : SyncActionNode(name, conf)
  { }

  static PortsList providedPorts()
  {
    return { 
      InputPort<geometry_msgs::Pose>("fake_pose"),
      OutputPort<geometry_msgs::Pose>("output") 
    };
  }

  // This Action writes a value into the port "text"
  NodeStatus tick() override
  {
    geometry_msgs::Pose pose;

    if (!getInput<geometry_msgs::Pose>("fake_pose", pose)) {
      ROS_ERROR("missing required input [fake_pose]");
      return NodeStatus::FAILURE;
    }

    setOutput("output", pose);

    return NodeStatus::SUCCESS;
  }
};

#endif