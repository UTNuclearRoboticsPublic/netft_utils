#ifndef NET_FT_UTILS_LEAN_H
#define NET_FT_UTILS_LEAN_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/WrenchStamped.h"
#include "netft_utils/Cancel.h"
#include "lpfilter.h"
#include <math.h>
#include "netft_rdt_driver.h"
#include <memory>
#include <future>

/**
 * This program takes force/torque data and applies transforms to usable data
 */

class NetftUtilsLean
{
public:
  NetftUtilsLean(ros::NodeHandle* nh);
  ~NetftUtilsLean();

  bool initialize(double rate, std::string world, std::string ft, double force = 60.0, double torque = 6.0);
  bool setUserInput(std::string world, std::string ft, double force, double torque);
  bool run();
  void stop();

  // Access methods
  bool biasSensor(bool toBias);
  bool setMax(double fMaxU, double tMaxU, double fMaxB, double tMaxB);
  bool setThreshold(double fThresh, double tThresh);
  bool setFilter(bool toFilter, double deltaT, double cutoffFreq);
  bool isReady();
  bool waitForData(double timeout);
  void getWorldData(geometry_msgs::WrenchStamped& data);
  void getToolData(geometry_msgs::WrenchStamped& data);
  void getRawData(geometry_msgs::WrenchStamped& data);
  bool isRunning();
  void setFTAddress(std::string ftAddress);
  void setFTTopic(std::string ftTopic);

private:
  //Node handle
  ros::NodeHandle* n;                               // ROS node handle

  // Initialization
  std::string ftAddress;
  std::string ftTopic;
  bool isInit;
  bool hasData;

  //netft class
  std::unique_ptr<netft_rdt_driver::NetFTRDTDriver> netft;

  // cycle rate
  double cycleRate;

  //LPFilter
  LPFilter* lp;                                    // Filter
  bool isFilterOn;
  double deltaTFilter;
  double cutoffFrequency;
  bool newFilter;
  bool lpExists;

  // Transform listener
  tf::TransformListener* listener;
  tf::StampedTransform ft_to_world;                // Transform from ft frame to world frame
  std::string world_frame;
  std::string ft_frame;

  // Wrenches used to hold force/torque and bias data
  geometry_msgs::WrenchStamped bias;               // Wrench containing the current bias data in tool frame
  geometry_msgs::WrenchStamped raw_data_tool;      // Wrench containing the current raw data from the netft sensor in the tool frame
  geometry_msgs::WrenchStamped tf_data_world;      // Wrench containing the transformed (world frame) data with bias and threshold applied
  geometry_msgs::WrenchStamped tf_data_tool;       // Wrench containing the transformed (tool frame) data with bias and threshold applied
  geometry_msgs::WrenchStamped zero_wrench;        // Wrench of all zeros for convenience
  geometry_msgs::WrenchStamped threshold;          // Wrench containing thresholds
  geometry_msgs::WrenchStamped raw_topic_data;     // Wrench containing raw topic data

  bool isBiased;                                   // True if sensor is biased
  bool isNewBias;                                  // True if sensor was biased this pass
  bool waitingForTransform;                        // False after initial transform is supplied
  bool isActive;                                   // True if run function has been called

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
  ros::Subscriber ft_sub;

  // ROS publishers
  ros::Publisher netft_cancel_pub;
  ros::Publisher data_pub;

  // Callback methods
  void netftCallback(const geometry_msgs::WrenchStamped& data);
  void dataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Threads
  std::future<bool> monitorThread;
  std::future<bool> updateThread;
  bool toUpdate;
  bool toMonitor;

  bool update();

  // Convenience methods
  void copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out, geometry_msgs::WrenchStamped &bias);
  void applyThreshold(double &value, double thresh);
  void transformFrame(geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped &out_data, char target_frame);
  void checkMaxForce();
  bool monitorData();

  static const bool DEBUG_DATA = true;
};

#endif
