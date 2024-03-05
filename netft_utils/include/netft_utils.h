#ifndef NET_FT_UTILS_H
#define NET_FT_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <netft_interfaces/srv/set_bias.hpp>
#include <netft_interfaces/srv/set_max.hpp>
#include <netft_interfaces/srv/set_threshold.hpp>
#include <netft_interfaces/srv/set_tool_data.hpp>
#include <netft_interfaces/srv/set_filter.hpp>
#include <netft_interfaces/srv/get_double.hpp>
#include <netft_interfaces/msg/cancel.hpp>
#include <lpfilter.h>
#include <math.h>

/**
 * This program takes force/torque data and applies transforms to usable data
 */

namespace netft_utils
{

  class NetftUtils : rclcpp::Node
  {
  public:
    NetftUtils();
    ~NetftUtils();

    void initialize();
    void setUserInput(std::string world, std::string ft, double force, double torque);
    void update();
    rclcpp::Logger getLog();

  private:
    
    //LPFilter
    LPFilter* lp;                                    // Filter
    bool isFilterOn;
    double deltaTFilter; 
    double cutoffFrequency;
    bool newFilter;

    // Transform listener
    tf2_ros::TransformListener* listener;
    tf2::BufferCore bufferCore;
    tf2::Stamped<tf2::Transform> ft_to_world;                // Transform from ft frame to world frame
    std::string world_frame;
    std::string ft_frame;
  
    // Wrenches used to hold force/torque and bias data
    geometry_msgs::msg::WrenchStamped bias;               // Wrench containing the current bias data in tool frame
    geometry_msgs::msg::WrenchStamped weight_bias;        // Wrench containing the bias at a measurement pose (to measure the weight)
    geometry_msgs::msg::WrenchStamped raw_data_world;     // Wrench containing the current raw data from the netft sensor transformed into the world frame
    geometry_msgs::msg::WrenchStamped raw_data_tool;      // Wrench containing the current raw data from the netft sensor in the tool frame
    geometry_msgs::msg::WrenchStamped tf_data_world;      // Wrench containing the transformed (world frame) data with bias and threshold applied
    geometry_msgs::msg::WrenchStamped tf_data_tool;       // Wrench containing the transformed (tool frame) data with bias and threshold applied
    geometry_msgs::msg::WrenchStamped zero_wrench;        // Wrench of all zeros for convenience
    geometry_msgs::msg::WrenchStamped threshold;          // Wrench containing thresholds
    
    double payloadWeight;				   // Used in gravity compensation
    double payloadLeverArm;			   // Used in gravity compensation. The z-coordinate to payload CoM (in sensor's raw frame)
    
    bool isBiased;                                   // True if sensor is biased
    bool isNewBias;                                  // True if sensor was biased this pass
    bool isNewGravityBias;			   // True if gravity compensation was applied this pass
    bool isGravityBiased;				   // True if gravity is compensated
    
    // Variables used to monitor FT violation and send a cancel move message
    netft_interfaces::msg::Cancel cancel_msg;
    static const int MAX_CANCEL = 5;                 // Number of times to send cancel message when max force is exceeded
    static const int MAX_WAIT = 100;                 // Number of cycles to wait after cancel message before sending again
    int cancel_count;                                // Counter for times to send cancel message when max force is exceeded
    int cancel_wait;                                 // Counter of cycles to wait after cancel message before sending again
    double forceMaxB;                                // Default max force limit to send cancel when FT is biased
    double torqueMaxB;                               // Default max torque limit to send cancel when FT is biased
    double forceMaxU;                                // Default max force limit to send cancel when FT is unbiased
    double torqueMaxU;                               // Default max torque limit to send cancel when FT is unbiased
    
    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_data_sub;
    
    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_raw_world_data_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_world_data_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr netft_tool_data_pub;
    rclcpp::Publisher<netft_interfaces::msg::Cancel>::SharedPtr netft_cancel_pub;
    
    ////////////////
    // ROS services
    ////////////////
    rclcpp::Service<netft_interfaces::srv::SetBias>::SharedPtr bias_service;
    rclcpp::Service<netft_interfaces::srv::SetBias>::SharedPtr gravity_comp_service;
    rclcpp::Service<netft_interfaces::srv::SetMax>::SharedPtr set_max_service;
    rclcpp::Service<netft_interfaces::srv::SetThreshold>::SharedPtr threshold_service;
    rclcpp::Service<netft_interfaces::srv::SetBias>::SharedPtr weight_bias_service;
    rclcpp::Service<netft_interfaces::srv::GetDouble>::SharedPtr get_weight_service;
    rclcpp::Service<netft_interfaces::srv::SetFilter>::SharedPtr filter_service;

    ////////////////////
    // Callback methods
    ////////////////////
    
    // Runs when a new datapoint comes in
    void netftCallback(const geometry_msgs::msg::WrenchStamped::ConstPtr& data);
    
    // Set the readings from the sensor to zero at this instant and continue to apply the bias on future readings.
    // This doesn't account for gravity i.e. it will not change if the sensor's orientation changes.
    // Run this method when the sensor is stationary to avoid inertial effects.
    bool fixedOrientationBias(netft_interfaces::srv::SetBias::Request &req, netft_interfaces::srv::SetBias::Response &res);
    
    // Set the readings from the sensor to zero at this instant.
    // Calculate the payload's mass and center of mass so gravity can be compensated for, even as the sensor changes orientation.
    // It's assumed that the payload's center of mass is located on the sensor's central access.
    // Run this method when the sensor is stationary to avoid inertial effects.
    // It assumes the Z-axis of the World tf frame is up.
    bool compensateForGravity(netft_interfaces::srv::SetBias::Request &req, netft_interfaces::srv::SetBias::Response &res);

    bool setMax(netft_interfaces::srv::SetMax::Request &req, netft_interfaces::srv::SetMax::Response &res);
    bool setWeightBias(netft_interfaces::srv::SetBias::Request &req, netft_interfaces::srv::SetBias::Response &res);
    bool getWeight(netft_interfaces::srv::GetDouble::Request &req, netft_interfaces::srv::GetDouble::Response &res);
    bool setThreshold(netft_interfaces::srv::SetThreshold::Request &req, netft_interfaces::srv::SetThreshold::Response &res);
    bool setFilter(netft_interfaces::srv::SetFilter::Request &req, netft_interfaces::srv::SetFilter::Response &res);

    // Convenience methods
    void copyWrench(geometry_msgs::msg::WrenchStamped &in, geometry_msgs::msg::WrenchStamped &out, geometry_msgs::msg::WrenchStamped &bias);
    void applyThreshold(double &value, double thresh);
    void transformFrame(geometry_msgs::msg::WrenchStamped in_data, geometry_msgs::msg::WrenchStamped &out_data, char target_frame);
    void checkMaxForce();
  };
}
#endif