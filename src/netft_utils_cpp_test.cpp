
/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Andy Zelenak
*/

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
  // Adjust the data acquisition rate and set the World and Sensor frames, respectively
  fti->initialize(1/ftSleep, "base_link", "ee_link");
  
  // Set max and threshold force/torque readings
  // Vaules below the thresholds will register as zero
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
