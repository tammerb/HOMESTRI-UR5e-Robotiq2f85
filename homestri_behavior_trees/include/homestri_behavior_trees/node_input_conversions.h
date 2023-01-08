#ifndef NODE_INPUT_CONVERSIONS_H
#define NODE_INPUT_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

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
}

#endif