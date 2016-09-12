
/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Alex von Sternberg
*/

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "netft_utils/StartSim.h"
#include "netft_utils/StopSim.h"
#include <math.h>

/**
 * This program simulates the force torque data coming from a FT sensor
 */

// SIM vars
enum SimType
{
  TYPE_ZEROS    = 0,
  TYPE_LINEAR   = 1,
  TYPE_EXP      = 2,
  TYPE_TRIANGLE = 3
};
enum Dim   {DIM_X      = 0,
            DIM_Y      = 1,
            DIM_Z      = 2,
            DIM_RX     = 3,
            DIM_RY     = 4,
            DIM_RZ     = 5};

geometry_msgs::WrenchStamped simWrench;        // Wrench containing the simulated data
bool toSim = false;                            // True if we should output simulated data
SimType simType;                               // Defines the simulation profile type
Dim simDim;                                    // Dimension to simulate FT in
double simStart;                               // Time that the simulation started
double simSlope;                               // Parameter to tweak how fast force increases
double maxForce = 0.0;                         // Max force to be simulated
double upSlope = true;                         // True if the triangle wave is on up slope

void setWrench(double x, double y, double z, double rx, double ry, double rz)
{
  simWrench.wrench.force.x = x;
  simWrench.wrench.force.y = y;
  simWrench.wrench.force.z = z;
  simWrench.wrench.torque.x = rx;
  simWrench.wrench.torque.y = ry;
  simWrench.wrench.torque.z = rz;
}

void simData()
{
  if(toSim)
  {
    double ft = 0.0;
    double deltaT = ros::Time::now().toSec() - simStart;
    switch (simType)
    {
      case TYPE_ZEROS:
        break;      
      case TYPE_LINEAR:
      {
        if((deltaT * simSlope) < maxForce)
          ft = deltaT * simSlope;
        else
          ft = maxForce;
      }
        break;
      case TYPE_EXP:
      {
        if((pow(deltaT,2) * simSlope) < maxForce)
          ft = pow(deltaT,2) * simSlope;
        else
          ft = maxForce;
      }
        break;
      case TYPE_TRIANGLE:
      {
        if((deltaT * simSlope) < maxForce && upSlope)
        {
          ft = deltaT * simSlope;
        }
        else if((deltaT * simSlope) >= maxForce && upSlope)
        {
          ft = maxForce;
          upSlope = false;
        }
        else if(2*maxForce - deltaT*simSlope > 0.0)
        {
          ft = 2*maxForce - deltaT*simSlope;
        }
        else
        {
          ft = 0.0;
        }
      }
        break;
      default:
        ROS_ERROR("Sim type invalid. Stopping sim.");
        toSim = false;
        break;
    }
    switch (simDim)
    {
      case DIM_X:
        setWrench(ft, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      case DIM_Y:
        setWrench(0.0, ft, 0.0, 0.0, 0.0, 0.0);
        break;
      case DIM_Z:
        setWrench(0.0, 0.0, ft, 0.0, 0.0, 0.0);
        break;
      case DIM_RX:
        setWrench(0.0, 0.0, 0.0, ft, 0.0, 0.0);
        break;
      case DIM_RY:
        setWrench(0.0, 0.0, 0.0, 0.0, ft, 0.0);
        break;
      case DIM_RZ:
        setWrench(0.0, 0.0, 0.0, 0.0, 0.0, ft);
        break;
      default:
        ROS_ERROR("Dimension invalid. Stopping sim.");
        toSim = false;
        break;
    }
  }
  else
  {
    upSlope = true;
    setWrench(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
}                 
                  
bool startSim(netft_utils::StartSim::Request &req, netft_utils::StartSim::Response &res)
{  
  simStart = ros::Time::now().toSec();
  simDim = (Dim)req.simDim;
  simType = (SimType)req.simType;
  simSlope = req.simSlope;              
  maxForce = req.maxForce;
  toSim = true;
  return true;
}  

bool stopSim(netft_utils::StopSim::Request &req, netft_utils::StopSim::Response &res)
{ 
  toSim = false;                
  return true;    
}  
    
int main(int argc, char **argv)
{                 
  //Node name     
  ros::init(argc, argv, "netft_utils_sim");
  
  //Access main ROS system
  ros::NodeHandle n;
                  
  //Publish on the /netft_transformed_data topic. Queue up to 100000 data points
  ros::Publisher netft_data_pub = n.advertise<geometry_msgs::WrenchStamped>("netft_data", 100000);
  
  //Advertise bias and threshold services
  ros::ServiceServer start_service = n.advertiseService("start_sim", startSim);
  ros::ServiceServer stop_service = n.advertiseService("stop_sim", stopSim);
  
  ros::Rate loop_rate(400);

  //Initialize variables
  upSlope = true;
  
  while ( ros::ok() )
  {
    simData();
    
    // Publish transformed dat
    netft_data_pub.publish( simWrench );
  
    loop_rate.sleep();		
    ros::spinOnce();
  }
  
  return 0;
}
