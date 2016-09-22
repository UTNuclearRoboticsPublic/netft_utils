
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

// Test the C++ interface for the ATI force/torque sensor with a Netbox.

// Most of the action happens in this class.
// The underscores denote members of the class.
class netftExample
{
  public:

    ros::NodeHandle n_;
    double ftSleep_; // Controls the data acquisition rate
    NetftUtilsLean* fti_;
    geometry_msgs::WrenchStamped wrench_; // For data from the sensor

    // Constructor
    netftExample();

    // Most of the action happens here
    void getFTData();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "netft_utils_cpp_test");
  ros::AsyncSpinner* spinner;
  spinner = new ros::AsyncSpinner(3);
  spinner->start();

  // Create a new netftExample object.
  // Most of the action happens here
  netftExample *myNetftExample = new netftExample();
  
  ros::waitForShutdown();
 
  return 0;
}

// Constructor for the netftExample class
netftExample::netftExample()
{
  // Connect and bias the ft sensor
  fti_ = new NetftUtilsLean(&n_);
  fti_->setFTAddress("192.168.1.84");
  // Adjust the data acquisition rate and set the World and Sensor frames, respectively
  ftSleep_ = 0.05; // Will yield a 20 Hz data acquisition rate
  fti_->initialize(1/ftSleep_, "right_ur5_base_link", "right_ur5_ee_link");
  
  // Set max and threshold force/torque readings
  // Vaules below the thresholds will register as zero
  fti_->setMax(80.0, 8.0, 60.0, 6.0);
  std::future<bool> ftThread;
  ftThread = std::async(std::launch::async, &NetftUtilsLean::run, fti_);
  fti_->biasSensor(1);

  getFTData();
}

void netftExample::getFTData()
{
  while (ros::ok())
  {
    fti_->getToolData(wrench_);
    ROS_INFO_STREAM("X force in the tool frame: " << wrench_.wrench.force.x);

    fti_->getWorldData(wrench_);
    ROS_INFO_STREAM("X force in the world frame: " << wrench_.wrench.force.x);

    sleep(ftSleep_);
  }
}
