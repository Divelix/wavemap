#pragma once

// wavemap
#include <wavemap/tmp_check_3rdparty.hh>

// ROS
#include <string>

#include <ros/ros.h>

namespace wavemap_ros {

class MyCoreClass {
 public:
  MyCoreClass(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};

}  // namespace wavemap_ros