#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>

#include <algorithm>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <tf2_ros/transform_listener.h>

#include <homestri_msgs/ManipulationAction.h>
#include <homestri_msgs/ManipulationGoal.h>
#include <homestri_msgs/ManipulationResult.h>
#include <homestri_msgs/ManipulationFeedback.h>

#include <homestri_msgs/ComplianceControlAction.h>
#include <homestri_msgs/ComplianceControlGoal.h>
#include <homestri_msgs/ComplianceControlResult.h>
#include <homestri_msgs/ComplianceControlFeedback.h>

#include <homestri_msgs/ForceControlAction.h>
#include <homestri_msgs/ForceControlGoal.h>
#include <homestri_msgs/ForceControlResult.h>
#include <homestri_msgs/ForceControlFeedback.h>

#include <homestri_msgs/CartesianControlAction.h>
#include <homestri_msgs/CartesianControlGoal.h>
#include <homestri_msgs/CartesianControlResult.h>
#include <homestri_msgs/CartesianControlFeedback.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <control_msgs/GripperCommandResult.h>
#include <control_msgs/GripperCommandFeedback.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/ApplyPlanningSceneRequest.h>
#include <moveit_msgs/ApplyPlanningSceneResponse.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

#include "behaviortree_cpp/behavior_tree.h"
#include <behaviortree_ros/bt_action_node.h>
#include <behaviortree_ros/bt_service_node.h>

#include <homestri_behavior_trees/node_input_conversions.h>

using namespace BT;

class ManipulationAction : public RosActionNode<homestri_msgs::ManipulationAction>
{

public:
  ManipulationAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<homestri_msgs::ManipulationAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<std::string>("id"),
        InputPort<std::string>("target"),
        InputPort<double>("position"),
        InputPort<double>("offset"),
        InputPort<geometry_msgs::Vector3>("translational_offset"),
        InputPort<geometry_msgs::Pose>("pose")};
  }

  bool sendGoal(GoalType &goal) override
  {
    // initialize defaults
    goal.id = "";
    goal.target = "";
    goal.position = 0;
    goal.offset = 0;
    goal.pose = geometry_msgs::Pose();

    if (!getInput<std::string>("id", goal.id))
    {
      ROS_ERROR("missing required input [id]");
      return false;
    }
    getInput<std::string>("target", goal.target);
    getInput<double>("position", goal.position);
    getInput<double>("offset", goal.offset);
    getInput<geometry_msgs::Vector3>("translational_offset", goal.translational_offset);
    getInput<geometry_msgs::Pose>("pose", goal.pose);

    ROS_INFO("SENDING GOAL: %s", goal.id.c_str());

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
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
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("ManipulationAction halted");
      BaseClass::halt();
    }
  }
};

class DetectFrame : public SyncActionNode
{
public:
  DetectFrame(const std::string &name, const NodeConfiguration &conf)
      : SyncActionNode(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return {
        InputPort<std::string>("target_frame"),
        InputPort<std::string>("source_frame"),
        InputPort<std::string>("t_x"),
        InputPort<std::string>("t_y"),
        InputPort<std::string>("t_z"),
        InputPort<std::string>("r_x"),
        InputPort<std::string>("r_y"),
        InputPort<std::string>("r_z"),
        InputPort<std::string>("r_w"),
        OutputPort<geometry_msgs::Pose>("output_pose"),
        InputPort<double>("timeout")};
  }

  // This Action writes a value into the port "text"
  NodeStatus
  tick() override
  {
    std::string target_frame;
    if (!getInput<std::string>("target_frame", target_frame))
    {
      throw RuntimeError("missing port [target_frame]");
    }

    std::string source_frame;
    if (!getInput<std::string>("source_frame", source_frame))
    {
      throw RuntimeError("missing port [source_frame]");
    }

    double timeout;
    if (!getInput<double>("timeout", timeout))
    {
      throw RuntimeError("missing port [timeout]");
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(timeout));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return NodeStatus::FAILURE;
    }

    geometry_msgs::Pose pose;
    pose.position.x = transformStamped.transform.translation.x;
    pose.position.y = transformStamped.transform.translation.y;
    pose.position.z = transformStamped.transform.translation.z;
    pose.orientation.x = transformStamped.transform.rotation.x;
    pose.orientation.y = transformStamped.transform.rotation.y;
    pose.orientation.z = transformStamped.transform.rotation.z;
    pose.orientation.w = transformStamped.transform.rotation.w;

    double t_x, t_y, t_z; 
    double r_x, r_y, r_z, r_w;
    if (getInput<double>("t_x", t_x)) pose.position.x = t_x;
    if (getInput<double>("t_y", t_y)) pose.position.y = t_y;
    if (getInput<double>("t_z", t_z)) pose.position.z = t_z;
    if (getInput<double>("r_x", r_x)) pose.orientation.x = r_x;
    if (getInput<double>("r_y", r_y)) pose.orientation.y = r_y;
    if (getInput<double>("r_z", r_z)) pose.orientation.z = r_z;
    if (getInput<double>("r_w", r_w)) pose.orientation.w = r_w;

    setOutput("output_pose", pose);

    return NodeStatus::SUCCESS;
  }
};

class DetectObject : public SyncActionNode
{
public:
  DetectObject(const std::string &name, const NodeConfiguration &conf)
      : SyncActionNode(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return {
        InputPort<bool>("detected"),
        InputPort<std::string>("target_object"),
        InputPort<std::string>("actual_object"),
        InputPort<std::string>("predicted_object")};
  }

  // This Action writes a value into the port "text"
  NodeStatus tick() override
  {
    auto status = NodeStatus::SUCCESS;

    ros::Duration(5).sleep(); // wait 1 sec, pretend to process image

    return status;
  }
};

class UpdatePose : public SyncActionNode
{
public:
  UpdatePose(const std::string &name, const NodeConfiguration &conf)
      : SyncActionNode(name, conf)
  {
  }

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::Pose>("input_pose"),
        OutputPort<geometry_msgs::Pose>("output_pose")};
  }

  // This Action writes a value into the port "text"
  NodeStatus tick() override
  {
    geometry_msgs::Pose input_pose;
    if (!getInput("input_pose", input_pose))
    {
      throw RuntimeError("missing port [input_pose]");
    }
    setOutput("output_pose", input_pose);
    return NodeStatus::SUCCESS;
  }
};

class TriggerService: public RosServiceNode<std_srvs::Trigger>
{

public:
  TriggerService( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<std_srvs::Trigger>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {};
  }

  void sendRequest(RequestType& request) override
  {
    ROS_INFO("TriggerService: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("TriggerService: response received");
    if( rep.success == true)
    {
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("TriggerService failed");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("TriggerService request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};

class AddCollisionMeshService: public RosServiceNode<moveit_msgs::ApplyPlanningScene>
{

public:
  AddCollisionMeshService( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<moveit_msgs::ApplyPlanningScene>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("mesh_path"),
      InputPort<geometry_msgs::Pose>("pose"),
      InputPort<geometry_msgs::Pose>("mesh_pose"),
      InputPort<std::string>("id"),
      InputPort<std::string>("frame_id"),
    };
  }

  void sendRequest(RequestType& request) override
  {
    ROS_INFO("AddCollisionMesh: sending request");

    std::string mesh_path;
    if (!getInput<std::string>("mesh_path", mesh_path))
    {
      ROS_ERROR("missing required input [mesh_path]");
    }

    std::string frame_id;
    if (!getInput<std::string>("frame_id", frame_id))
    {
      ROS_ERROR("missing required input [frame_id]");
    }

    std::string id;
    if (!getInput<std::string>("id", id))
    {
      ROS_ERROR("missing required input [id]");
    }

    geometry_msgs::Pose pose;
    if (!getInput<geometry_msgs::Pose>("pose", pose))
    {
      ROS_ERROR("missing required input [pose]");
    }

    geometry_msgs::Pose mesh_pose;
    if (!getInput<geometry_msgs::Pose>("mesh_pose", mesh_pose))
    {
      ROS_ERROR("missing required input [mesh_pose]");
    }

    shapes::Mesh* m = shapes::createMeshFromResource(mesh_path, Eigen::Vector3d(0.001, 0.001, 0.001));

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::CollisionObject co;
    co.header.frame_id = frame_id;
    co.pose = pose;
    co.id = id;
    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(mesh_pose);
    co.operation = moveit_msgs::CollisionObject::ADD;

    request.scene.is_diff = true;
    request.scene.world.collision_objects.push_back(co);
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("AddCollisionMesh: response received");
    if( rep.success == true)
    {
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("AddCollisionMesh failed");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("AddCollisionMesh request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};

class SwitchControllerService: public RosServiceNode<controller_manager_msgs::SwitchController>
{
private:
  
  const static inline std::vector<std::string> ConflictingControllers = { 
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
    "joint_group_vel_controller", 
    "twist_controller",
    "cartesian_motion_controller",
    "cartesian_force_controller",
    "cartesian_compliance_controller"
  };

public:
  SwitchControllerService( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<controller_manager_msgs::SwitchController>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("controller")
    };
  }

  void sendRequest(RequestType& request) override
  {
    ROS_INFO("AddCollisionMesh: sending request");

    std::string controller;
    if (!getInput<std::string>("controller", controller))
    {
      ROS_ERROR("missing required input [controller]");
    }

    std::vector<std::string> other_controllers = SwitchControllerService::ConflictingControllers;
    other_controllers.erase(std::remove(other_controllers.begin(), other_controllers.end(), controller), other_controllers.end());

    request.start_controllers.resize(0);
    request.start_controllers.push_back(controller);

    request.stop_controllers = other_controllers;

    request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("SwitchControllerService: response received");
    if( rep.ok == true)
    {
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("SwitchControllerService failed");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("SwitchControllerService request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }
};

class ComplianceControlAction : public RosActionNode<homestri_msgs::ComplianceControlAction>
{

public:
  ComplianceControlAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<homestri_msgs::ComplianceControlAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::Pose>("pose"),
        InputPort<geometry_msgs::Pose>("offset"),
        InputPort<geometry_msgs::Wrench>("wrench"),
        InputPort<std::string>("frame_id"),
        InputPort<std::string>("mode")
      };
  }

  bool sendGoal(GoalType &goal) override
  {
    getInput<geometry_msgs::Pose>("pose", goal.pose);

    getInput<geometry_msgs::Pose>("offset", goal.offset);

    if (!getInput<geometry_msgs::Wrench>("wrench", goal.wrench))
    {
      ROS_ERROR("missing required input [wrench]");
      return false;
    }
    if (!getInput<std::string>("frame_id", goal.frame_id))
    {
      ROS_ERROR("missing required input [frame_id]");
      return false;
    }

    std::string mode;
    if (!getInput<std::string>("mode", mode))
    {
      ROS_ERROR("missing required input [mode]");
      return false;
    }

    if (mode == "offset") {
      goal.mode = homestri_msgs::CartesianControlGoal::MODE_OFFSET;
    }
    else if (mode == "target") {
      goal.mode = homestri_msgs::CartesianControlGoal::MODE_TARGET;
    }
    else {
      ROS_ERROR("mode should be offset or target");
      return false;
    }


    ROS_INFO("ComplianceControlAction: sending request");

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
  {
    ROS_INFO("ComplianceControlAction: succeeded");
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("ComplianceControlAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("ComplianceControlAction halted");
      BaseClass::halt();
    }
  }
};

class GripperAction : public RosActionNode<control_msgs::GripperCommandAction>
{
public:
  GripperAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<control_msgs::GripperCommandAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<double>("position"),
        InputPort<double>("max_effort")};
  }

  bool sendGoal(GoalType &goal) override
  {
    if (!getInput<double>("position", goal.command.position))
    {
      ROS_ERROR("missing required input [position]");
      return false;
    }

    if (!getInput<double>("max_effort", goal.command.max_effort))
    {
      ROS_ERROR("missing required input [max_effort]");
      return false;
    }

    ROS_INFO("SENDING GOAL: GripperAction");

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
  {
    ROS_INFO("SUCCEEDED: GripperAction");
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("ManipulationAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("ManipulationAction halted");
      BaseClass::halt();
    }
  }
};

class CartesianControlAction : public RosActionNode<homestri_msgs::CartesianControlAction>
{

public:
  CartesianControlAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<homestri_msgs::CartesianControlAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::Pose>("pose"),
        InputPort<geometry_msgs::Pose>("offset"),
        InputPort<std::string>("frame_id"),
        InputPort<double>("duration"),
        InputPort<std::string>("mode")
      };
  }

  bool sendGoal(GoalType &goal) override
  {
    getInput<geometry_msgs::Pose>("pose", goal.pose);

    getInput<geometry_msgs::Pose>("offset", goal.offset);
  
    if (!getInput<std::string>("frame_id", goal.frame_id))
    {
      ROS_ERROR("missing required input [frame_id]");
      return false;
    }
    if (!getInput<double>("duration", goal.duration))
    {
      ROS_ERROR("missing required input [duration]");
      return false;
    }

    std::string mode;
    if (!getInput<std::string>("mode", mode))
    {
      ROS_ERROR("missing required input [mode]");
      return false;
    }

    if (mode == "offset") {
      goal.mode = homestri_msgs::CartesianControlGoal::MODE_OFFSET;
    }
    else if (mode == "target") {
      goal.mode = homestri_msgs::CartesianControlGoal::MODE_TARGET;
    }
    else {
      ROS_ERROR("mode should be offset or target");
      return false;
    }

    ROS_INFO("CartesianControlAction: sending request");

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
  {
    ROS_INFO("CartesianControlAction: succeeded");
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("CartesianControlAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("CartesianControlAction halted");
      BaseClass::halt();
    }
  }
};

class ForceControlAction : public RosActionNode<homestri_msgs::ForceControlAction>
{

public:
  ForceControlAction(ros::NodeHandle &handle, const std::string &name, const NodeConfiguration &conf) : RosActionNode<homestri_msgs::ForceControlAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return {
        InputPort<geometry_msgs::Wrench>("wrench"),
        InputPort<std::string>("frame_id")
      };
  }

  bool sendGoal(GoalType &goal) override
  {
    if (!getInput<geometry_msgs::Wrench>("wrench", goal.wrench))
    {
      ROS_ERROR("missing required input [wrench]");
      return false;
    }
    if (!getInput<std::string>("frame_id", goal.frame_id))
    {
      ROS_ERROR("missing required input [frame_id]");
      return false;
    }

    ROS_INFO("ForceControlAction: sending request");

    return true;
  }

  NodeStatus onResult(const ResultType &res) override
  {
    ROS_INFO("ForceControlAction: succeeded");
    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("ForceControlAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (status() == NodeStatus::RUNNING)
    {
      ROS_WARN("ForceControlAction halted");
      BaseClass::halt();
    }
  }
};

#endif