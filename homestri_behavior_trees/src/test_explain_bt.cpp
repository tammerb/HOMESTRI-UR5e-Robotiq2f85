#include <ros/ros.h>
#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <homestri_behavior_trees/action_nodes.h>
#include <homestri_behavior_trees/condition_nodes.h>

#include <explain_bt/ExplainableBT.h>

using namespace BT;

std::string get_xml_filename(ros::NodeHandle& nh, std::string param) {
  std::string xml_filename;
  nh.getParam(param, xml_filename);
  std::cout << xml_filename << "********" << std::endl;
  ROS_INFO("Loading XML : %s", xml_filename.c_str());
  return xml_filename;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;

  std::string xml_filename = get_xml_filename(nh, "/bt_xml");

  BehaviorTreeFactory factory;
  RegisterRosAction<ManipulationAction>(factory, "ManipulationAction", nh);
  RegisterRosSubscriber<GraspedCondition>(factory, "GraspedCondition", nh);
  factory.registerNodeType<DetectObject>("DetectObject");
  auto tree = factory.createTreeFromFile(xml_filename);
  ExplainableBT explainable_tree(tree);
  
  ros::ServiceServer service = nh.advertiseService("explainable_bt", &ExplainableBT::explain_callback, &explainable_tree);

  NodeStatus status = NodeStatus::IDLE;
  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING /*|| status == NodeStatus::FAILURE*/))
  {
    ros::spinOnce();
    status = explainable_tree.execute();
    // std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  ros::spin();

  return 0;
}