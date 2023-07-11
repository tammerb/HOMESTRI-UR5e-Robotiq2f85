#ifndef NODE_INPUT_CONVERSIONS_H
#define NODE_INPUT_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "behaviortree_cpp/behavior_tree.h"

namespace BT
{
  template <> inline geometry_msgs::Pose convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() == 7) {
      geometry_msgs::Pose pose;
      pose.position.x = convertFromString<double>(parts[0]);
      pose.position.y = convertFromString<double>(parts[1]);
      pose.position.z = convertFromString<double>(parts[2]);
      pose.orientation.x = convertFromString<double>(parts[3]);
      pose.orientation.y = convertFromString<double>(parts[4]);
      pose.orientation.z = convertFromString<double>(parts[5]);
      pose.orientation.w = convertFromString<double>(parts[6]);

      return pose;
    }
    else if (parts.size() == 6) {
      geometry_msgs::Pose pose;
      pose.position.x = convertFromString<double>(parts[0]);
      pose.position.y = convertFromString<double>(parts[1]);
      pose.position.z = convertFromString<double>(parts[2]);

      tf2::Quaternion quat;
      double r = convertFromString<double>(parts[3]);
      double p = convertFromString<double>(parts[4]);
      double y = convertFromString<double>(parts[5]);
      quat.setRPY( r, p, y );
      pose.orientation.x = quat.getX();
      pose.orientation.y = quat.getY();
      pose.orientation.z = quat.getZ();
      pose.orientation.w = quat.getW();

      return pose;
    }
    else {
      throw RuntimeError("invalid input");
    }
  }

  template <> inline geometry_msgs::Wrench convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() == 6) {
      geometry_msgs::Wrench wrench;
      wrench.force.x = convertFromString<double>(parts[0]);
      wrench.force.y = convertFromString<double>(parts[1]);
      wrench.force.z = convertFromString<double>(parts[2]);
      wrench.torque.x = convertFromString<double>(parts[3]);
      wrench.torque.y = convertFromString<double>(parts[4]);
      wrench.torque.z = convertFromString<double>(parts[5]);

      return wrench;
    }
    else {
      throw RuntimeError("invalid input");
    }
  }

  template <> inline geometry_msgs::Transform convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() == 7) {
      geometry_msgs::Transform transform;
      transform.translation.x = convertFromString<double>(parts[0]);
      transform.translation.y = convertFromString<double>(parts[1]);
      transform.translation.z = convertFromString<double>(parts[2]);
      transform.rotation.x = convertFromString<double>(parts[3]);
      transform.rotation.y = convertFromString<double>(parts[4]);
      transform.rotation.z = convertFromString<double>(parts[5]);
      transform.rotation.w = convertFromString<double>(parts[6]);

      return transform;
    }
    else if (parts.size() == 6) {
      geometry_msgs::Transform transform;
      transform.translation.x = convertFromString<double>(parts[0]);
      transform.translation.y = convertFromString<double>(parts[1]);
      transform.translation.z = convertFromString<double>(parts[2]);

      tf2::Quaternion quat;
      double r = convertFromString<double>(parts[3]);
      double p = convertFromString<double>(parts[4]);
      double y = convertFromString<double>(parts[5]);
      quat.setRPY( r, p, y );
      transform.rotation.x = quat.getX();
      transform.rotation.y = quat.getY();
      transform.rotation.z = quat.getZ();
      transform.rotation.w = quat.getW();

      return transform;
    }
    else {
      throw RuntimeError("invalid input");
    }
  }


  template <> inline std::vector<std::string> convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    std::vector<std::string> strvec(parts.size());
    for (unsigned int i = 0; i < parts.size(); i++) {
      strvec[i] = convertFromString<std::string>(parts[i]);
    }
    return strvec;
  }

  template <> inline geometry_msgs::Vector3 convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() == 3) {
      geometry_msgs::Vector3 vector;
      vector.x = convertFromString<double>(parts[0]);
      vector.y = convertFromString<double>(parts[1]);
      vector.z = convertFromString<double>(parts[2]);
      return vector;
    }
    else {
      throw RuntimeError("invalid input");
    }
  }
}

#endif