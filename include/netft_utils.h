#ifndef NET_FT_UTILS_H
#define NET_FT_UTILS_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/WrenchStamped.h"
#include "netft_utils/SetBias.h"
#include "netft_utils/SetMax.h"
#include "netft_utils/SetThreshold.h"
#include "netft_utils/SetToolData.h"
#include "netft_utils/SetFilter.h"
#include "netft_utils/GetDouble.h"
#include "netft_utils/Cancel.h"
#include "lpfilter.h"
#include <math.h>

/**
 * This program takes force/torque data and applies transforms to usable data
 */

class NetftUtils
{
public:
  NetftUtils(ros::NodeHandle nh);
  ~NetftUtils();

  void initialize();
  void setUserInput(std::string world, std::string ft, double force, double torque);
  void update();

private:
  //Node handle
  ros::NodeHandle n;                               // ROS node handle
  
  //LPFilter
  LPFilter* lp;                                    // Filter
  bool isFilterOn;
  double deltaTFilter;
  double cutoffFrequency;
  bool newFilter;

  // Transform listener
  tf::TransformListener* listener;
  tf::StampedTransform ft_to_world;                // Transform from ft frame to world frame
  std::string world_frame;
  std::string ft_frame;
 
  // Wrenches used to hold force/torque and bias data
  geometry_msgs::WrenchStamped bias;               // Wrench containing the current bias data in tool frame
  geometry_msgs::WrenchStamped weight_bias;        // Wrench containing the bias at a measurement pose (to measure the weight)
  geometry_msgs::WrenchStamped raw_data_world;     // Wrench containing the current raw data from the netft sensor transformed into the world frame
  geometry_msgs::WrenchStamped raw_data_tool;      // Wrench containing the current raw data from the netft sensor in the tool frame
  geometry_msgs::WrenchStamped tf_data_world;      // Wrench containing the transformed (world frame) data with bias and threshold applied
  geometry_msgs::WrenchStamped tf_data_tool;       // Wrench containing the transformed (tool frame) data with bias and threshold applied
  geometry_msgs::WrenchStamped zero_wrench;        // Wrench of all zeros for convenience
  geometry_msgs::WrenchStamped threshold;          // Wrench containing thresholds
  
  double payloadWeight;				   // Used in gravity compensation
  double payloadLeverArm;			   // Used in gravity compensation. The z-coordinate to payload CoM (in sensor's raw frame)
  
  bool isBiased;                                   // True if sensor is biased
  bool isNewBias;                                  // True if sensor was biased this pass
  bool isNewGravityBias;			   // True if gravity compensation was applied this pass
  bool isGravityBiased;				   // True if gravity is compensated
  
  // Variables used to monitor FT violation and send a cancel move message
  netft_utils::Cancel cancel_msg;
  static const int MAX_CANCEL = 5;                 // Number of times to send cancel message when max force is exceeded
  static const int MAX_WAIT = 100;                 // Number of cycles to wait after cancel message before sending again
  int cancel_count;                                // Counter for times to send cancel message when max force is exceeded
  int cancel_wait;                                 // Counter of cycles to wait after cancel message before sending again
  double forceMaxB;                                // Default max force limit to send cancel when FT is biased
  double torqueMaxB;                               // Default max torque limit to send cancel when FT is biased
  double forceMaxU;                                // Default max force limit to send cancel when FT is unbiased
  double torqueMaxU;                               // Default max torque limit to send cancel when FT is unbiased
  
  // ROS subscribers
  ros::Subscriber raw_data_sub;
  
  // ROS publishers
  ros::Publisher netft_raw_world_data_pub;
  ros::Publisher netft_world_data_pub;
  ros::Publisher netft_tool_data_pub;
  ros::Publisher netft_cancel_pub;
  
  ////////////////
  // ROS services
  ////////////////
  ros::ServiceServer bias_service;
  ros::ServiceServer gravity_comp_service;
  ros::ServiceServer set_max_service;
  ros::ServiceServer theshold_service;
  ros::ServiceServer weight_bias_service;
  ros::ServiceServer get_weight_service;
  ros::ServiceServer filter_service;

  ////////////////////
  // Callback methods
  ////////////////////
  
  // Runs when a new datapoint comes in
  void netftCallback(const geometry_msgs::WrenchStamped::ConstPtr& data);
  
  // Set the readings from the sensor to zero at this instant and continue to apply the bias on future readings.
  // This doesn't account for gravity i.e. it will not change if the sensor's orientation changes.
  // Run this method when the sensor is stationary to avoid inertial effects.
  bool fixedOrientationBias(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res);
  
  // Set the readings from the sensor to zero at this instant.
  // Calculate the payload's mass and center of mass so gravity can be compensated for, even as the sensor changes orientation.
  // It's assumed that the payload's center of mass is located on the sensor's central access.
  // Run this method when the sensor is stationary to avoid inertial effects.
  // It assumes the Z-axis of the World tf frame is up.
  bool compensateForGravity(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res);
  
  bool setMax(netft_utils::SetMax::Request &req, netft_utils::SetMax::Response &res);
  bool setWeightBias(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res);
  bool getWeight(netft_utils::GetDouble::Request &req, netft_utils::GetDouble::Response &res);
  bool setThreshold(netft_utils::SetThreshold::Request &req, netft_utils::SetThreshold::Response &res);
  bool setFilter(netft_utils::SetFilter::Request &req, netft_utils::SetFilter::Response &res);

  // Convenience methods
  void copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out, geometry_msgs::WrenchStamped &bias);
  void applyThreshold(double &value, double thresh);
  void transformFrame(geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped &out_data, char target_frame);
  void checkMaxForce();
};

#endif