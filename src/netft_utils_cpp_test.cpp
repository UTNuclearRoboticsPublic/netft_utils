
#ifndef KEY_COMPLIANCE_H_
#define KEY_COMPLIANCE_H_
#endif

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/console.h"
#include "netft_utils_lean.h"
#include <string.h>

// Author: Andy Zelenak, 2016
// Test the C++ interface for the ATI force/torque sensor with a Netbox.

// Callback for the ft sensor
void netftCallback(const geometry_msgs::WrenchStamped& wrench);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "netft_utils_cpp_test");
  ros::NodeHandle n;
  ros::AsyncSpinner* spinner;
  spinner = new ros::AsyncSpinner(3);
  spinner->start();
  double ftSleep; // controls the data acquisition rate
  
  // Subscribe to the F/T topic
  ros::Subscriber ftSub = n.subscribe("/netft/netft_data", 1, netftCallback);
  
  // Connect and bias the ft sensor
  NetftUtilsLean* fti = new NetftUtilsLean(&n);
  fti->setFTAddress("192.168.1.85");
  // Rate to match motion commands to robot. This controls the loop rate.
  fti->initialize(1/ftSleep, "right_ur5_ee_link", "right_ur5_ee_link");
  fti->setMax(80.0, 8.0, 60.0, 6.0);
  std::future<bool> ftThread;
  ftThread = std::async(std::launch::async, &NetftUtilsLean::run, fti);
  fti->biasSensor(1);
  
  ros::waitForShutdown();
 
  return 0;
}

// Callback for the ft sensor.
void netftCallback(const geometry_msgs::WrenchStamped& wrench)
{
  ROS_INFO_STREAM("x force: " << wrench.wrench.force.x);
}