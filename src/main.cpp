#include <ros/ros.h>
#include "vrep_swing_mechanism.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrep_swing_mechanism_node");
  VrepSwingMechanism vrep_swing_mechanism;
  ROS_INFO("VrepSwingMechanism was initialized!");

  ros::spin();
  return 0;
}
