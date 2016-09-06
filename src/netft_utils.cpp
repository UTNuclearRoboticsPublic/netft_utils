#include "netft_utils.h"

int main(int argc, char **argv)
{
  // Initialize the ros netft_utils_node
  ros::init(argc, argv, "netft_utils_node");

  // Instantiate utils class
  ros::NodeHandle n;
  NetftUtils utils(n);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize utils
  utils.initialize();

  // Set up user input
  std::string world_frame;
  std::string ft_frame;
  double forceMaxU = 0.0;
  double torqueMaxU = 0.0;
  if(argc<3)
  {
    ROS_FATAL("You must pass in at least the world and ft frame as command line arguments. Argument options are [world frame, ft frame, max force, max torque]");
    return 1;
  }
  else if(argc>=6)
  {
    ROS_FATAL("Too many arguments for netft_utils");
  }
  else
  {
    world_frame = argv[1];
    ft_frame = argv[2];
    if(argc>=4)
      forceMaxU = atof(argv[3]);
    if(5==argc)
      torqueMaxU = atof(argv[4]);
  }
  utils.setUserInput(world_frame, ft_frame, forceMaxU, torqueMaxU);

  // Main ros loop
  ros::Rate loop_rate(500);
  //ros::Time last;
  while(ros::ok())
  {
    utils.update();
    loop_rate.sleep();
    //ros::Time curr = ros::Time::now();
    //ROS_INFO_STREAM("Loop time: " <<  curr.toSec()-last.toSec());
    //last = curr;
  }

  return 0;
}

NetftUtils::NetftUtils(ros::NodeHandle nh) :
  n(nh),
  isFilterOn(false),
  deltaTFilter(0.0),
  cutoffFrequency(0.0),
  newFilter(false),
  sensor_mass(0.0),
  sensor_COM_z(0.0),
  tool_mass(0.0),
  tool_COM_x(0.0),
  tool_COM_y(0.0),
  tool_COM_z(0.0),
  isBiased(false),
  isNewBias(false),
  cancel_count(MAX_CANCEL),
  cancel_wait(MAX_WAIT),
  forceMaxB(10.0),
  torqueMaxB(0.8),
  forceMaxU(50.0),
  torqueMaxU(5.0)
{
}

NetftUtils::~NetftUtils()
{
  delete listener;
  delete lp;
}

void NetftUtils::initialize()
{
  //lp = new LPFilter(0.002,200,6);
  
  //Zero out the zero wrench
  zero_wrench.wrench.force.x = 0.0;
  zero_wrench.wrench.force.y = 0.0;
  zero_wrench.wrench.force.z = 0.0;
  zero_wrench.wrench.torque.x = 0.0;
  zero_wrench.wrench.torque.y = 0.0;
  zero_wrench.wrench.torque.z = 0.0;

  //Initialize cancel message
  cancel_msg.toCancel = false;

  //Listen to the transfomation from the ft sensor to world frame.
  listener = new tf::TransformListener(ros::Duration(300));

  //Subscribe to the NetFT topic.
  raw_data_sub = n.subscribe("netft_data",100, &NetftUtils::netftCallback, this);

  //Publish on the /netft_transformed_data topic. Queue up to 100000 data points
  netft_raw_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("raw_world", 100000);
  netft_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("transformed_world", 100000);
  netft_tool_data_pub = n.advertise<geometry_msgs::WrenchStamped>("transformed_tool", 100000);
  netft_cancel_pub = n.advertise<netft_utils::Cancel>("cancel", 100000);

  //Advertise bias and threshold services
  bias_service = n.advertiseService("bias", &NetftUtils::biasSensor, this);
  set_max_service = n.advertiseService("set_max_values", &NetftUtils::setMax, this);
  theshold_service = n.advertiseService("set_threshold", &NetftUtils::setThreshold, this);
  weight_bias_service = n.advertiseService("set_weight_bias", &NetftUtils::setWeightBias, this);
  get_weight_service = n.advertiseService("get_weight", &NetftUtils::getWeight, this);
  filter_service = n.advertiseService("filter", &NetftUtils::setFilter, this);
  //set_tool_service = n.advertiseService("set_netft_tool_data", set_tool_data);
  //world_data_service = n.advertiseService("get_netft_raw_world_data", get_world_data);
}

void NetftUtils::setUserInput(std::string world, std::string ft, double force, double torque)
{
  world_frame = world;
  ft_frame = ft;
  if(force != 0.0)
  {
    forceMaxU = force;
  }
  if(torque != 0.0)
  {
    torqueMaxU = torque;
  }
}

void NetftUtils::update()
{
  // Check for a filter
  if(newFilter)
  {
    delete lp;
    lp = new LPFilter(deltaTFilter,cutoffFrequency,6);
    newFilter = false;
  }
  // Look up transform from ft to world frame
  tf::StampedTransform tempTransform;
  try
  {
    listener->waitForTransform(world_frame, ft_frame, ros::Time(0), ros::Duration(1.0));
    listener->lookupTransform(world_frame, ft_frame, ros::Time(0), tempTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // Set translation to zero before updating value
  tempTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  ft_to_world = tempTransform;

  checkMaxForce();

  // Publish transformed dat
  netft_raw_world_data_pub.publish( raw_data_world );
  netft_world_data_pub.publish( tf_data_world );
  netft_tool_data_pub.publish( tf_data_tool );
  netft_cancel_pub.publish( cancel_msg );

  ros::spinOnce();
}

void NetftUtils::copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out, geometry_msgs::WrenchStamped &bias)
{
  out.header.stamp = in.header.stamp;
  out.header.frame_id = in.header.frame_id;
  out.wrench.force.x = in.wrench.force.x - bias.wrench.force.x;
  out.wrench.force.y = in.wrench.force.y - bias.wrench.force.y;
  out.wrench.force.z = in.wrench.force.z - bias.wrench.force.z;
  out.wrench.torque.x = in.wrench.torque.x - bias.wrench.torque.x;
  out.wrench.torque.y = in.wrench.torque.y - bias.wrench.torque.y;
  out.wrench.torque.z = in.wrench.torque.z - bias.wrench.torque.z;
}

void NetftUtils::applyThreshold(double &value, double thresh)
{
  if(value <= thresh && value >= -thresh)
  {
    value = 0.0;
  }
}

void NetftUtils::transformFrame(geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped &out_data, char target_frame)
{
  tf::Vector3 tempF;
  tf::Vector3 tempT;
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

void NetftUtils::netftCallback(const geometry_msgs::WrenchStamped::ConstPtr& data)
{
  // Filter data
  std::vector<double> tempData;
  tempData.resize(6);
  tempData.at(0) = -data->wrench.force.x;
  tempData.at(1) = data->wrench.force.y;
  tempData.at(2) = data->wrench.force.z;
  tempData.at(3) = -data->wrench.torque.x;
  tempData.at(4) = data->wrench.torque.y;
  tempData.at(5) = data->wrench.torque.z;
  
  if(isFilterOn && !newFilter)
    lp->update(tempData,tempData);
  
  // Copy tool frame data. apply negative to x data to follow right hand rule convention (ft raw data does not)
  raw_data_tool.header.stamp = data->header.stamp;
  raw_data_tool.header.frame_id = ft_frame;
  raw_data_tool.wrench.force.x = tempData.at(0);
  raw_data_tool.wrench.force.y = tempData.at(1);
  raw_data_tool.wrench.force.z = tempData.at(2);
  raw_data_tool.wrench.torque.x = tempData.at(3);
  raw_data_tool.wrench.torque.y = tempData.at(4);
  raw_data_tool.wrench.torque.z = tempData.at(5);
  
  // Calculate current gravity bias
  tf::Vector3 sensor_COM;
  sensor_COM.setX(0.0);
  sensor_COM.setY(0.0);
  sensor_COM.setZ(sensor_COM_z);
  tf::Vector3 tool_COM;
  sensor_COM.setZ(tool_COM_x);
  sensor_COM.setZ(tool_COM_y);
  sensor_COM.setZ(tool_COM_z);
  tf::Vector3 arm;
  double total_mass = tool_mass + sensor_mass;
  arm = (sensor_COM * sensor_mass + tool_COM * tool_mass)/total_mass;
  tf::Vector3 arm_world = ft_to_world * arm;
  tf::Vector3 gravity;
  gravity.setX(0.0);
  gravity.setY(0.0);
  gravity.setZ(-9.81);
  tf::Vector3 moments;
  if(total_mass!=0.0)
  {
    moments = arm_world.cross(total_mass * gravity);
  }
  else
  {
    moments.setX(0.0);
    moments.setY(0.0);
    moments.setZ(0.0);
  }  

  gravity_bias.wrench.force.x = 0.0;
  gravity_bias.wrench.force.y = 0.0;
  gravity_bias.wrench.force.z = total_mass * -9.81;
  gravity_bias.wrench.torque.x = moments.getX();
  gravity_bias.wrench.torque.y = moments.getY();
  gravity_bias.wrench.torque.z = moments.getZ();
  
  // Copy in new netft data in tool frame and transform to world frame
  transformFrame(raw_data_tool, raw_data_world, 'w');
  
  // Get tool bias in world frame
  geometry_msgs::WrenchStamped world_bias;
  transformFrame(bias, world_bias, 'w');
  
  // Add bias and apply threshold to get transformed data
  copyWrench(raw_data_world, tf_data_world, world_bias);
  copyWrench(tf_data_world, tf_data_world, gravity_bias);
  
  // Transform to tool frame
  transformFrame(tf_data_world, tf_data_tool, 't');
                  
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
  //ROS_INFO_STREAM("Callback time: " << tf_data_tool.header.stamp.toSec()-ros::Time::now().toSec());
}                 
                  
bool NetftUtils::biasSensor(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res)
{                 
  if(req.toBias)  
  {               
    copyWrench(raw_data_tool, bias, zero_wrench);
    geometry_msgs::WrenchStamped gravity_bias_tool;
    transformFrame(gravity_bias, gravity_bias_tool, 't');
    copyWrench(bias, bias, gravity_bias_tool);
    if(req.forceMax >= 0.0001)
      forceMaxB = req.forceMax;
    if(req.torqueMax >= 0.0001)
      torqueMaxB = req.torqueMax;
    isNewBias = true;
  }               
  else            
  {               
    copyWrench(zero_wrench, bias, zero_wrench);
  }               
  isBiased = req.toBias;
  res.success = true;
                  
  return true;    
}   

bool NetftUtils::setFilter(netft_utils::SetFilter::Request &req, netft_utils::SetFilter::Response &res)
{                 
  if(req.toFilter)  
  {
    newFilter = true;
    isFilterOn = true;
    deltaTFilter = req.deltaT;
    cutoffFrequency = req.cutoffFrequency;
  }               
  else            
  {               
    isFilterOn = false;
  }               
  
  return true;    
}  

bool NetftUtils::setMax(netft_utils::SetMax::Request &req, netft_utils::SetMax::Response &res)
{                 
  if(req.forceMax >= 0.0001)
    forceMaxU = req.forceMax;
  if(req.torqueMax >= 0.0001)
    torqueMaxU = req.torqueMax;
  
  res.success = true;             
  return true;    
}  

bool NetftUtils::setWeightBias(netft_utils::SetBias::Request &req, netft_utils::SetBias::Response &res)
{                 
  if(req.toBias)  
  {               
    copyWrench(raw_data_tool, weight_bias, zero_wrench);
  }               
  else            
  {               
    copyWrench(zero_wrench, weight_bias, zero_wrench);
  }               
  res.success = true;
                  
  return true;    
}  
    
bool NetftUtils::getWeight(netft_utils::GetDouble::Request &req, netft_utils::GetDouble::Response &res)
{                 
  geometry_msgs::WrenchStamped carried_weight;
  copyWrench(raw_data_tool, carried_weight, weight_bias);
  res.weight = pow((pow(carried_weight.wrench.force.x, 2.0) + pow(carried_weight.wrench.force.y, 2.0) + pow(carried_weight.wrench.force.z, 2.0)), 0.5)/9.81*1000;
                  
  return true;    
}      
            
bool NetftUtils::setThreshold(netft_utils::SetThreshold::Request &req, netft_utils::SetThreshold::Response &res)
{                 
  threshold.wrench.force.x = req.data.wrench.force.x;
  threshold.wrench.force.y = req.data.wrench.force.y;
  threshold.wrench.force.z = req.data.wrench.force.z;
  threshold.wrench.torque.x = req.data.wrench.torque.x;
  threshold.wrench.torque.y = req.data.wrench.torque.y;
  threshold.wrench.torque.z = req.data.wrench.torque.z;
                  
  res.success = true;
                  
  return true;    
}

void NetftUtils::checkMaxForce()
{
  double fMag = pow((pow(tf_data_tool.wrench.force.x, 2.0) + pow(tf_data_tool.wrench.force.y, 2.0) + pow(tf_data_tool.wrench.force.z, 2.0)), 0.5);
  double tMag = pow((pow(tf_data_tool.wrench.torque.x, 2.0) + pow(tf_data_tool.wrench.torque.y, 2.0) + pow(tf_data_tool.wrench.torque.z, 2.0)), 0.5);
  double fMax;
  double tMax;
  if(isBiased && !isNewBias)
  {
    fMax = forceMaxB;
    tMax = torqueMaxB;
  }
  else
  {
    if(isBiased && isNewBias)
    {
      isNewBias = false;
    }

    fMax = forceMaxU; //50.0;
    tMax = torqueMaxU; //5.0;
  }

  // If max FT exceeded, send cancel unless we have just sent it MAX_CANCEL times
  //ROS_INFO("FMAG: %f TMAG: %f", fMag, tMag);
  if((fabs(fMag) > fMax || fabs(tMag) > tMax) && cancel_count > 0)
  {
    cancel_msg.toCancel = true;
    //ROS_INFO("Force torque violation. Canceling move.");
    ROS_INFO("FMAG: %f FMAX: %f TMAG:%f TMAX: %f count: %d wait: %d", fMag, fMax, tMag, tMax, cancel_count, cancel_wait);
    cancel_count-=1;
  }
  // If we have sent cancel MAX_CANCEL times, don't send cancel again for MAX_WAIT cycles
  else if(cancel_count == 0 && cancel_wait > 0 && cancel_wait <= MAX_WAIT)
  {
    cancel_msg.toCancel = false;
    cancel_wait-=1;
  }
  // If we have just finished waiting MAX_WAIT times, or the max force is no longer exceeded, reset cancel_count and cancel_wait
  else if(cancel_wait == 0 || !(fabs(fMag) > fMax || fabs(tMag) > tMax))
  {
    cancel_msg.toCancel = false;
    cancel_count = MAX_CANCEL;
    cancel_wait = MAX_WAIT;
  }
}
//bool set_tool_data(netft_utils::SetToolData::Request &req, netft_utils::SetToolData::Response &res)
//{               
//    tool_mass = req.mass;
//    tool_COM = req.COM;
//                
//    res.success = true;
//                
//    return true;
//}               
                  
//bool get_world_data(netft_utils::GetWorldData::Request &req, netft_utils::GetWorldData::Response &res)
//{                 
//  res.data = raw_data_world;
//  return true;    
//}                 
                  
