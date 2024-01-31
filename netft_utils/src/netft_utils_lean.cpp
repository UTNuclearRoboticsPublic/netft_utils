
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

#include "netft_utils_lean.h"
namespace netft_utils_lean
{

  NetftUtilsLean::NetftUtilsLean() : Node("netft_utils_lean"),
    ftAddress(""),
    ftTopic(""),
    isInit(false),
    hasData(false),
    cycleRate(0.0),
    isFilterOn(false),
    deltaTFilter(0.0),
    cutoffFrequency(0.0),
    newFilter(false),
    lpExists(false),
    isBiased(false),
    isNewBias(false),
    waitingForTransform(true),
    isActive(false),
    cancel_count(MAX_CANCEL),
    cancel_wait(MAX_WAIT),
    forceMaxB(10.0),
    torqueMaxB(0.8),
    forceMaxU(50.0),
    torqueMaxU(5.0),
    toUpdate(false),
    toMonitor(false)
  {
  }

  NetftUtilsLean::~NetftUtilsLean()
  {
    delete listener;
    if(lpExists)
      delete lp;
  }

  bool NetftUtilsLean::initialize(double rate, std::string world, std::string ft, double force, double torque)
  {
    // Set the user input
    if(!setUserInput(world, ft, force, torque))
    {
      return false;
    }
    // Set the cycle rate
    if(rate>0.001)
    {
      cycleRate = rate;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Cycle rate should be positive");
      return false;
    }

    // Set up access to netft data
    if(!ftTopic.empty())
    {
      ft_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(ftTopic, 1, &NetftUtilsLean::dataCallback);
      RCLCPP_INFO_STREAM(get_logger(), "Using NETFT topic instead of IP because ftTopic is:" << ftTopic);
    }
    else if(!ftAddress.empty())
    {
      try
      {
        netft = std::unique_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(ftAddress));
      }
      catch(std::runtime_error)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not access data from netft_rdt_driver");
        return false;
      }
    }
    else
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Can not initialize FT, must set topic or address first.");
        return false;
    }

    //Zero out the zero wrench
    zero_wrench.wrench.force.x = 0.0;
    zero_wrench.wrench.force.y = 0.0;
    zero_wrench.wrench.force.z = 0.0;
    zero_wrench.wrench.torque.x = 0.0;
    zero_wrench.wrench.torque.y = 0.0;
    zero_wrench.wrench.torque.z = 0.0;

    //Initialize cancel message
    cancel_msg.cancel = false;

    //Listen to the transfomation from the ft sensor to world frame.
    listener = new tf2_ros::TransformListener(bufferCore);

    //Publish on the /cancel topic. Queue up to 100000 data points
    netft_cancel_pub = this->create_publisher<netft_interfaces::msg::Cancel>("/netft/cancel", 100000);

    if(DEBUG_DATA)
      data_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/netft/netft_data", 100000);

    isInit = true;
    return true;
  }

  bool NetftUtilsLean::run()
  {
    if(!isInit)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Cannot run before initialization is successful.");
      return false;
    }

    isActive = true;

    toUpdate = true;
    toMonitor = true;

    //Spin off update thread
    updateThread = std::async(std::launch::async, &NetftUtilsLean::update, this);

    //Spin off thread to monitor netft data
    if(!ftAddress.empty())
    {
      monitorThread = std::async(std::launch::async, &NetftUtilsLean::monitorData, this);
    }

    //Join threads
    if(!ftAddress.empty())
    {
      monitorThread.get();
    }
    updateThread.get();

    isActive = false;
    return true;
  }

  void NetftUtilsLean::stop()
  {
    toUpdate = false;
    toMonitor = false;
  }

  bool NetftUtilsLean::monitorData()
  {
    while(waitingForTransform)
    {
      std::chrono::nanoseconds duration(5000);
      rclcpp::sleep_for(duration);
    }
    rclcpp::Rate r(cycleRate);
    while (rclcpp::ok() && toMonitor)
    {
      //ros::Time tempTime = ros::Time::now();
      if (netft->waitForNewData())
      {
        //std::cout << "Time waiting for new data: " <<ros::Time::now().toSec()-tempTime.toSec() << std::endl;
        geometry_msgs::msg::WrenchStamped data;
        //tempTime = ros::Time::now();
        netft->getData(data);
        //std::cout << "Time to get data: " <<ros::Time::now().toSec()-tempTime.toSec() << std::endl;
        //tempTime = ros::Time::now();
        netftCallback(data);
        //std::cout << "Time in netftCallback: " <<ros::Time::now().toSec()-tempTime.toSec() << std::endl;
        hasData = true;
      }
      //std::cout << "Monitoring!!!!!!" << std::endl;
      //tempTime = ros::Time::now();
      r.sleep();
      //std::cout << "Time sleeping: " <<ros::Time::now().toSec()-tempTime.toSec() << std::endl;
    }
    hasData = false;
    return true;
  }

  void NetftUtilsLean::dataCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr& msg)
  {
    if(!waitingForTransform && toMonitor)
    {
      raw_topic_data;
      raw_topic_data.header = msg->header;
      raw_topic_data.wrench = msg->wrench;
      netftCallback(raw_topic_data);
      hasData = true;
    }
    else
      hasData = false;
  }

  bool NetftUtilsLean::isReady()
  {
    return hasData;
  }

  bool NetftUtilsLean::waitForData(double timeout)
  {
    rclcpp::Clock clock;
    rclcpp::Time startWait = clock.now();
    while((clock.now().seconds() - startWait.seconds()) < timeout && !hasData)
    {
      std::chrono::nanoseconds duration(1000);
      rclcpp::sleep_for(duration);
    }
    return hasData;
  }

  bool NetftUtilsLean::setUserInput(std::string world, std::string ft, double force, double torque)
  {
    if(world.compare("") == 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "World frame string cannot be empty");
      return false;
    }
    world_frame = world;
    RCLCPP_INFO_STREAM(get_logger(), "World frame: " << world_frame);
    if(ft.compare("") == 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "FT frame string cannot be empty");
      return false;
    }
    ft_frame = ft;
    RCLCPP_INFO_STREAM(get_logger(), "FT frame: " << ft_frame);
    if(force >= 0.001)
    {
      forceMaxU = force;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "FT max force must be positive");
      return false;
    }
    if(torque >= 0.001)
    {
      torqueMaxU = torque;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "FT max torque must be positive");
      return false;
    }
    return true;
  }

  bool NetftUtilsLean::update()
  {
    if(!isInit)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Cannot update until NetftUtilsLean is initialized properly.");
      return false;
    }
    rclcpp::Rate r(cycleRate);
    while (rclcpp::ok() && toUpdate)
    {
      // Check for a filter
      if(newFilter)
      {
        if(lpExists)
          delete lp;
        lp = new LPFilter(deltaTFilter,cutoffFrequency,6);
        lpExists = true;
        newFilter = false;
      }

      // Look up transform from ft to world frame
      geometry_msgs::msg::TransformStamped tempTransform;
      try
      {
        tf2::TimePoint time;
        tempTransform = bufferCore.lookupTransform(world_frame, ft_frame, time);
      }
      catch (tf2::TransformException ex)
      {
        RCLCPP_ERROR(get_logger(), "%s",ex.what());
      }
      
      // Set translation to zero before updating value
      tf2::convert(tempTransform, ft_to_world);
      ft_to_world.setOrigin(tf2::Vector3(0.0,0.0,0.0));
      waitingForTransform = false;

      checkMaxForce();

      // Publish cancel_msg
      netft_cancel_pub->publish( cancel_msg );
      r.sleep();
    }
    return true;
  }

  void NetftUtilsLean::copyWrench(geometry_msgs::msg::WrenchStamped &in, geometry_msgs::msg::WrenchStamped &out, geometry_msgs::msg::WrenchStamped &diff)
  {
    out.header.stamp = in.header.stamp;
    out.header.frame_id = in.header.frame_id;
    out.wrench.force.x = in.wrench.force.x - diff.wrench.force.x;
    out.wrench.force.y = in.wrench.force.y - diff.wrench.force.y;
    out.wrench.force.z = in.wrench.force.z - diff.wrench.force.z;
    out.wrench.torque.x = in.wrench.torque.x - diff.wrench.torque.x;
    out.wrench.torque.y = in.wrench.torque.y - diff.wrench.torque.y;
    out.wrench.torque.z = in.wrench.torque.z - diff.wrench.torque.z;
  }

  void NetftUtilsLean::applyThreshold(double &value, double thresh)
  {
    if(value <= thresh && value >= -thresh)
    {
      value = 0.0;
    }
  }

  void NetftUtilsLean::transformFrame(geometry_msgs::msg::WrenchStamped in_data, geometry_msgs::msg::WrenchStamped &out_data, char target_frame)
  {
    tf2::Vector3 tempF;
    tf2::Vector3 tempT;
    tempF.setX(in_data.wrench.force.x);
    tempF.setY(in_data.wrench.force.y);
    tempF.setZ(in_data.wrench.force.z);
    tempT.setX(in_data.wrench.torque.x);
    tempT.setY(in_data.wrench.torque.y);
    tempT.setZ(in_data.wrench.torque.z);
    if(target_frame == 'w')
    {
        out_data.header.frame_id = world_frame;
        tempF = ft_to_world * tempF;
        tempT = ft_to_world * tempT;
    }
    else if(target_frame == 't')
    {
        out_data.header.frame_id = ft_frame;
        tempF = ft_to_world.inverse() * tempF;
        tempT = ft_to_world.inverse() * tempT;
    }
    out_data.header.stamp = in_data.header.stamp;
    out_data.wrench.force.x = tempF.getX();
    out_data.wrench.force.y = tempF.getY();
    out_data.wrench.force.z = tempF.getZ();
    out_data.wrench.torque.x = tempT.getX();
    out_data.wrench.torque.y = tempT.getY();
    out_data.wrench.torque.z = tempT.getZ();
  }

  void NetftUtilsLean::netftCallback(const geometry_msgs::msg::WrenchStamped& data)
  {
    // Filter data. apply negative to x data to follow right hand rule convention (ft raw data does not)
    std::vector<double> tempData;
    tempData.resize(6);
    if(!ftAddress.empty())
    {
      tempData.at(0) = -data.wrench.force.x;
      tempData.at(1) = data.wrench.force.y;
      tempData.at(2) = data.wrench.force.z;
      tempData.at(3) = -data.wrench.torque.x;
      tempData.at(4) = data.wrench.torque.y;
      tempData.at(5) = data.wrench.torque.z;
    }
    else
    {
      tempData.at(0) = data.wrench.force.x;
      tempData.at(1) = data.wrench.force.y;
      tempData.at(2) = data.wrench.force.z;
      tempData.at(3) = data.wrench.torque.x;
      tempData.at(4) = data.wrench.torque.y;
      tempData.at(5) = data.wrench.torque.z;
    }

    if(isFilterOn && !newFilter)
      lp->update(tempData,tempData);

    // Copy tool frame data.
    raw_data_tool.header.stamp = data.header.stamp;
    raw_data_tool.header.frame_id = ft_frame;
    raw_data_tool.wrench.force.x = tempData.at(0);
    raw_data_tool.wrench.force.y = tempData.at(1);
    raw_data_tool.wrench.force.z = tempData.at(2);
    raw_data_tool.wrench.torque.x = tempData.at(3);
    raw_data_tool.wrench.torque.y = tempData.at(4);
    raw_data_tool.wrench.torque.z = tempData.at(5);

    // Apply bias
    copyWrench(raw_data_tool, tf_data_tool, bias);

    // Copy in new netft data in tool frame and transform to world frame
    transformFrame(tf_data_tool, tf_data_world, 'w');

    // Apply thresholds
    applyThreshold(tf_data_world.wrench.force.x, threshold.wrench.force.x);
    applyThreshold(tf_data_world.wrench.force.y, threshold.wrench.force.y);
    applyThreshold(tf_data_world.wrench.force.z, threshold.wrench.force.z);
    applyThreshold(tf_data_world.wrench.torque.x, threshold.wrench.torque.x);
    applyThreshold(tf_data_world.wrench.torque.y, threshold.wrench.torque.y);
    applyThreshold(tf_data_world.wrench.torque.z, threshold.wrench.torque.z);
    applyThreshold(tf_data_tool.wrench.force.x, threshold.wrench.force.x);
    applyThreshold(tf_data_tool.wrench.force.y, threshold.wrench.force.y);
    applyThreshold(tf_data_tool.wrench.force.z, threshold.wrench.force.z);
    applyThreshold(tf_data_tool.wrench.torque.x, threshold.wrench.torque.x);
    applyThreshold(tf_data_tool.wrench.torque.y, threshold.wrench.torque.y);
    applyThreshold(tf_data_tool.wrench.torque.z, threshold.wrench.torque.z);

    // Publish data for debugging
    if(DEBUG_DATA)
      data_pub->publish(tf_data_tool);
    //RCLCPP_INFO_STREAM(get_logger(), "Callback time: " << tf_data_tool.header.stamp.toSec()-ros::Time::now().toSec());
  }

  bool NetftUtilsLean::biasSensor(bool toBias)
  {
    if(toBias)
    {
      if(!hasData)
      {
        geometry_msgs::msg::WrenchStamped data;
        if(!ftTopic.empty())
        {
          rclcpp::Clock clock;
          rclcpp::Time startTime = clock.now();
          while(clock.now().seconds()-startTime.seconds() < 0.1 && !hasData)
          {
            std::chrono::nanoseconds duration(1000);
            rclcpp::sleep_for(duration);
          }
          if(hasData)
          {
            data = raw_topic_data;
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Bias sensor failed.");
            return false;
          }
        }
        else
        {
          if (netft->waitForNewData())
          {
            netft->getData(data);
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Bias sensor failed.");
            return false;
          }
        }
        // Copy tool frame data.
        raw_data_tool.header.stamp = data.header.stamp;
        raw_data_tool.header.frame_id = ft_frame;
        raw_data_tool.wrench.force.x = -data.wrench.force.x;
        raw_data_tool.wrench.force.y = data.wrench.force.y;
        raw_data_tool.wrench.force.z = data.wrench.force.z;
        raw_data_tool.wrench.torque.x = -data.wrench.torque.x;
        raw_data_tool.wrench.torque.y = data.wrench.torque.y;
        raw_data_tool.wrench.torque.z = data.wrench.torque.z;
      }
      copyWrench(raw_data_tool, bias, zero_wrench);
      isNewBias = true;
    }
    else
    {
      copyWrench(zero_wrench, bias, zero_wrench);
    }
    isBiased = toBias;
    return true;
  }

  bool NetftUtilsLean::setFilter(bool toFilter, double deltaT, double cutoffFreq)
  {
    if(toFilter)
    {
      newFilter = true;
      isFilterOn = true;
      deltaTFilter = deltaT;
      cutoffFrequency = cutoffFreq;
    }
    else
    {
      isFilterOn = false;
    }

    return true;
  }

  bool NetftUtilsLean::setMax(double fMaxU, double tMaxU, double fMaxB, double tMaxB)
  {
    if(fMaxU >= 0.0001)
    {
      forceMaxU = fMaxU;
      torqueMaxU = tMaxU;
      forceMaxB = fMaxB;
      torqueMaxB = tMaxB;
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "All maximum FT values must be positive.");
      return false;
    }
  }

  bool NetftUtilsLean::setThreshold(double fThresh, double tThresh)
  {
    threshold.wrench.force.x = fThresh;
    threshold.wrench.force.y = fThresh;
    threshold.wrench.force.z = fThresh;
    threshold.wrench.torque.x = tThresh;
    threshold.wrench.torque.y = tThresh;
    threshold.wrench.torque.z = tThresh;

    return true;
  }

  void NetftUtilsLean::checkMaxForce()
  {
    double fMag = pow((pow(tf_data_tool.wrench.force.x, 2.0) + pow(tf_data_tool.wrench.force.y, 2.0) + pow(tf_data_tool.wrench.force.z, 2.0)), 0.5);
    double tMag = pow((pow(tf_data_tool.wrench.torque.x, 2.0) + pow(tf_data_tool.wrench.torque.y, 2.0) + pow(tf_data_tool.wrench.torque.z, 2.0)), 0.5);
    double fMax;
    double tMax;
    if(isBiased && !isNewBias)
    {
      if(forceMaxB > 0.001 && torqueMaxB > 0.001)
      {
        fMax = forceMaxB;
        tMax = torqueMaxB;
      }
      else
      {
        fMax = forceMaxU;
        tMax = torqueMaxU;
      }
    }
    else
    {
      if(isBiased && isNewBias)
      {
        isNewBias = false;
      }
      fMax = forceMaxU;
      tMax = torqueMaxU;
    }

    // If max FT exceeded, send cancel unless we have just sent it MAX_CANCEL times
    //RCLCPP_INFO("FMAG: %f TMAG: %f", fMag, tMag);
    if((fabs(fMag) > fMax || fabs(tMag) > tMax) && cancel_count > 0)
    {
      cancel_msg.cancel = true;
      //RCLCPP_INFO("Force torque violation. Canceling move.");
      RCLCPP_INFO(get_logger(), "FMAG: %f FMAX: %f TMAG:%f TMAX: %f count: %d wait: %d", fMag, fMax, tMag, tMax, cancel_count, cancel_wait);
      cancel_count-=1;
    }
    // If we have sent cancel MAX_CANCEL times, don't send cancel again for MAX_WAIT cycles
    else if(cancel_count == 0 && cancel_wait > 0 && cancel_wait <= MAX_WAIT)
    {
      cancel_msg.cancel = false;
      cancel_wait-=1;
    }
    // If we have just finished waiting MAX_WAIT times, or the max force is no longer exceeded, reset cancel_count and cancel_wait
    else if(cancel_wait == 0 || !(fabs(fMag) > fMax || fabs(tMag) > tMax))
    {
      cancel_msg.cancel = false;
      cancel_count = MAX_CANCEL;
      cancel_wait = MAX_WAIT;
    }
  }

  void NetftUtilsLean::setFTAddress(std::string ftAd)
  {
    ftAddress = ftAd;
  }

  void NetftUtilsLean::setFTTopic(std::string ftTop)
  {
    ftTopic = ftTop;
  }

  void NetftUtilsLean::getRawData(geometry_msgs::msg::WrenchStamped& data)
  {
    data = raw_data_tool;
  }

  void NetftUtilsLean::getToolData(geometry_msgs::msg::WrenchStamped& data)
  {
    data = tf_data_tool;
  }

  void NetftUtilsLean::getWorldData(geometry_msgs::msg::WrenchStamped& data)
  {
    data = tf_data_world;
  }

  bool NetftUtilsLean::isRunning()
  {
    return isActive;
  }
}

