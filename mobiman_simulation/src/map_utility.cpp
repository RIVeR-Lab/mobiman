// LAST UPDATE: 2024.01.23
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
// 
// TODO:

// --CUSTOM LIBRARIES--
#include "mobiman_simulation/map_utility.h"

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::MapUtility()
{
  tflistener = new tf::TransformListener;
  world_frame_name_ = "";
  map_name = "";
  map_frame_name = "";
  sensor_pc2_msg_name = "";
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;
  x_range.clear();
  y_range.clear();
  z_range.clear();
  map_resolution_ = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 0;
  map_server_dt = 0;
  skip_cnt = 0;
  oct_msg.header.frame_id = world_frame_name_;
  pc_msg.header.frame_id = world_frame_name_;
  pc2_msg.header.frame_id = world_frame_name_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::MapUtility(ros::NodeHandle& nh,
                       std::string oct_msg_name,
                       std::string pub_name_oct_msg)
{
  cout << "[MapUtility::MapUtility(3)] START" << std::endl;

  nh_ = nh;
  ns_ = nh_.getNamespace();

  cout << "[MapUtility::MapUtility(3)] BEFORE subscribe" << std::endl;
  sub_oct_msg_ = nh.subscribe(oct_msg_name, 100, &MapUtility::octMsgCallback, this);
  //sub_map = nh.subscribe(robot_param.local_map_msg, 1000, &Tentabot::mapCallback, this);

  cout << "[MapUtility::MapUtility(3)] BEFORE advertise" << std::endl;
  nh.advertise<octomap_msgs::Octomap>(pub_name_oct_msg, 10);

  cout << "[MapUtility::MapUtility(3)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::MapUtility(NodeHandle& nh,
                       NodeHandle& pnh,
                       std::string& world_frame_name,
                       std::string& gz_model_msg,
                       vector<std::string>& frame_name_pkgs_ign,
                       vector<sensor_msgs::PointCloud2>& pc2_msg_gz_pkgs_ign,
                       vector<geometry_msgs::Point>& vec_obj_bbx_min_ign,
                       vector<geometry_msgs::Point>& vec_obj_bbx_max_ign,
                       vector<geometry_msgs::Point>& vec_obj_dim_ign,
                       vector<std::string>& frame_name_pkgs_man,
                       vector<sensor_msgs::PointCloud2>& pc2_msg_gz_pkgs_man,
                       vector<geometry_msgs::Point>& vec_obj_bbx_min_man,
                       vector<geometry_msgs::Point>& vec_obj_bbx_max_man,
                       vector<geometry_msgs::Point>& vec_obj_dim_man,
                       double map_resolution)
{
  //cout << "[MapUtility::MapUtility(15)] START" << std::endl;

  nh_ = nh;
  ns_ = nh_.getNamespace();
  tflistener = new tf::TransformListener;

  world_frame_name_ = world_frame_name;

  vec_frame_name_ign_ = frame_name_pkgs_ign;
  vec_pc2_msg_gz_ign_ = pc2_msg_gz_pkgs_ign;
  vec_obj_bbx_min_ign_ = vec_obj_bbx_min_ign;
  vec_obj_bbx_max_ign_ = vec_obj_bbx_max_ign;
  vec_obj_dim_ign_ = vec_obj_dim_ign;

  vec_frame_name_man_ = frame_name_pkgs_man;
  vec_pc2_msg_gz_man_ = pc2_msg_gz_pkgs_man;
  vec_obj_bbx_min_man_ = vec_obj_bbx_min_man;
  vec_obj_bbx_max_man_ = vec_obj_bbx_max_man;
  vec_obj_dim_man_ = vec_obj_dim_man;

  map_resolution_ = map_resolution;
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  // Subscribers
  sub_gz_model_ = nh_.subscribe(gz_model_msg, 10, &MapUtility::gazeboModelCallback, this);

  // Publishers
  pub_pc2_msg_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan", 5);
  pub_oct_msg_ = nh_.advertise<octomap_msgs::Octomap>("octomap_scan", 5);
  pub_occ_grid_msg_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 5);

  /// NUA TODO: MAKE IT GENERALIZABLE!
  oct_conveyor_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_pkg_normal_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_pkg_long_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_pkg_longwide_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_red_cube_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_green_cube_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_blue_cube_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_actor0_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_actor1_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);
  oct_bin_ = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  //// NUA TODO: LEFT HEREEE DEFINE MSG NAMES AND ADD NAMESPACES IF REQUIRED!!! 
  if (ns_ != "/")
  {
    obj_name_conveyor_ = ns_ + "/" + obj_name_conveyor_;
    obj_name_normal_pkg_ = ns_ + "/" + obj_name_normal_pkg_;
    obj_name_long_pkg_ = ns_ + "/" + obj_name_long_pkg_;
    obj_name_longwide_pkg_ = ns_ + "/" + obj_name_longwide_pkg_;
    obj_name_red_cube_ = ns_ + "/" + obj_name_red_cube_;
    obj_name_green_cube_ = ns_ + "/" + obj_name_green_cube_;
    obj_name_blue_cube_ = ns_ + "/" + obj_name_blue_cube_;
    obj_name_actor0_ = ns_ + "/" + obj_name_actor0_;
    obj_name_actor1_ = ns_ + "/" + obj_name_actor1_;
    obj_name_bin_ = ns_ + "/" + obj_name_bin_;


    pc2_msg_name_conveyor_ = ns_ + "/" + pc2_msg_name_conveyor_;
    pc2_msg_name_normal_pkg_ = ns_ + "/" + pc2_msg_name_normal_pkg_;
    pc2_msg_name_long_pkg_ = ns_ + "/" + pc2_msg_name_long_pkg_;
    pc2_msg_name_longwide_pkg_ = ns_ + "/" + pc2_msg_name_longwide_pkg_;
    pc2_msg_name_red_cube_ = ns_ + "/" + pc2_msg_name_red_cube_;
    pc2_msg_name_green_cube_ = ns_ + "/" + pc2_msg_name_green_cube_;
    pc2_msg_name_blue_cube_ = ns_ + "/" + pc2_msg_name_blue_cube_;
    pc2_msg_name_actor0_ = ns_ + "/" + pc2_msg_name_actor0_;
    pc2_msg_name_actor1_ = ns_ + "/" + pc2_msg_name_actor1_;
    pc2_msg_name_bin_ = ns_ + "/" + pc2_msg_name_bin_;

    oct_msg_name_conveyor_ = ns_ + "/" + oct_msg_name_conveyor_;
    oct_msg_name_normal_pkg_ = ns_ + "/" + oct_msg_name_normal_pkg_;
    oct_msg_name_long_pkg_ = ns_ + "/" + oct_msg_name_long_pkg_;
    oct_msg_name_longwide_pkg_ = ns_ + "/" + oct_msg_name_longwide_pkg_;
    oct_msg_name_red_cube_ = ns_ + "/" + oct_msg_name_red_cube_;
    oct_msg_name_green_cube_ = ns_ + "/" + oct_msg_name_green_cube_;
    oct_msg_name_blue_cube_ = ns_ + "/" + oct_msg_name_blue_cube_;
    oct_msg_name_actor0_ = ns_ + "/" + oct_msg_name_actor0_;
    oct_msg_name_actor1_ = ns_ + "/" + oct_msg_name_actor1_;
    oct_msg_name_bin_ = ns_ + "/" + oct_msg_name_bin_;

    oct_msg_name_ = ns_ + "/" + oct_msg_name_;
  }

  pub_pc2_msg_conveyor_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_conveyor_, 5);
  pub_pc2_msg_pkg_normal_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_normal_pkg_, 5);
  pub_pc2_msg_pkg_long_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_long_pkg_, 5);
  pub_pc2_msg_pkg_longwide_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_longwide_pkg_, 5);
  pub_pc2_msg_red_cube_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_red_cube_, 5);
  pub_pc2_msg_green_cube_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_green_cube_, 5);
  pub_pc2_msg_blue_cube_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_blue_cube_, 5);
  pub_pc2_msg_actor0_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_actor0_, 5);
  pub_pc2_msg_actor1_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_actor1_, 5);
  pub_pc2_msg_bin_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_msg_name_bin_, 5);

  pub_oct_msg_conveyor_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_conveyor_, 5);
  pub_oct_msg_pkg_normal_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_normal_pkg_, 5);
  pub_oct_msg_pkg_long_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_long_pkg_, 5);
  pub_oct_msg_pkg_longwide_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_longwide_pkg_, 5);
  pub_oct_msg_red_cube_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_red_cube_, 5);
  pub_oct_msg_green_cube_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_green_cube_, 5);
  pub_oct_msg_blue_cube_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_blue_cube_, 5);
  pub_oct_msg_actor0_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_actor0_, 5);
  pub_oct_msg_actor1_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_actor1_, 5);
  pub_oct_msg_bin_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_bin_, 5);

  pub_oct_msg_ = nh_.advertise<octomap_msgs::Octomap>(oct_msg_name_, 5);

  //pub_pc2_msg_gz_man_pkg_normal_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_normal_pkg", 5);
  //pub_pc2_msg_gz_man_pkg_long_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_long_pkg", 5);
  //pub_pc2_msg_gz_man_pkg_longwide_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_longwide_pkg", 5);

  //pub_moveit_collision_object_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 5);

  pub_visu_occ_distance_ = nh_.advertise<visualization_msgs::Marker>("occupancy_distance", 5);
  pub_visu_array_occ_distance_ = nh_.advertise<visualization_msgs::MarkerArray>("occupancy_distance_array", 5);

  //cout << "[MapUtility::MapUtility(15)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::MapUtility(NodeHandle& nh, 
                       std::string new_map_name,
                       std::string new_sensor_pc2_msg_name, 
                       std::string new_sensor_laser_msg_name)
{
  nh_ = nh;
  ns_ = nh_.getNamespace();
  tflistener = new tf::TransformListener;

  map_name = new_map_name;

  world_frame_name_ = "";
  map_frame_name = "";
  sensor_pc2_msg_name = new_sensor_pc2_msg_name;
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;

  sensor_laser_msg_name = new_sensor_laser_msg_name;
  
  x_range.clear();
  y_range.clear();
  z_range.clear();
  
  map_resolution_ = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 100;
  map_server_dt = 0;
  skip_cnt = 0;
  
  oct_msg.header.frame_id = world_frame_name_;
  pc_msg.header.frame_id = world_frame_name_;
  pc2_msg.header.frame_id = world_frame_name_;

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (PointCloud2)
  sub_pc2 = nh.subscribe(sensor_pc2_msg_name, 100, &MapUtility::pc2Callback, this);

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (LaserScan)
  sub_laser = nh.subscribe(sensor_laser_msg_name, 100, &MapUtility::laserCallback, this);

  // NUA TODO: MAP SERVICE
  //ros::ServiceServer service_reset_map_utility = nh.advertiseService("reset_map_utility", &MapUtility::reset_map_utility, this);

  pub_oct_msg_ = nh.advertise<octomap_msgs::Octomap>("octomap_" + map_name, 10);
  pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("PC_" + map_name, 10);
  pc2_msg_pub = nh.advertise<sensor_msgs::PointCloud2>("PC2_" + map_name, 10);
  debug_array_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("debug_array_" + map_name, 10);
  debug_visu_pub = nh.advertise<visualization_msgs::Marker>("debug_" + map_name, 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::MapUtility(const MapUtility& mu)
{
  nh_ = mu.nh_;
  ns_ = mu.ns_;
  tflistener = mu.tflistener;
  world_frame_name_ = mu.world_frame_name_;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution_ = mu.map_resolution_;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  pub_oct_msg_ = mu.pub_oct_msg_;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility::~MapUtility()
{
  //ROS_INFO( "Calling Destructor for MapUtility..." );
  delete[] tflistener;
  //delete oct;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MapUtility& MapUtility::operator=(const MapUtility& mu) 
{
  nh_ = mu.nh_;
  ns_ = mu.ns_;
  tflistener = mu.tflistener;
  world_frame_name_ = mu.world_frame_name_;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution_ = mu.map_resolution_;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  pub_oct_msg_ = mu.pub_oct_msg_;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
  return *this;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getWorldFrameName()
{
  return world_frame_name_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getMapName()
{
  return map_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getMapFrameName()
{
  return map_frame_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getSensorPC2MsgName()
{
  return sensor_pc2_msg_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getSensorPC2Direction()
{
  return sensor_pc2_direction;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getSensorPC2FrameName()
{
  return sensor_pc2_frame_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getSensorPC2MinRange()
{
  return sensor_pc2_min_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getSensorPC2MaxRange()
{
  return sensor_pc2_max_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getSensorPC2MaxYaw()
{
  return sensor_pc2_max_yaw;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getSensorPC2MaxPitch()
{
  return sensor_pc2_max_pitch;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string MapUtility::getSensorLaserMsgName()
{
  return sensor_laser_msg_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
float MapUtility::getSensorLaserMaxRange()
{
  return sensor_laser_max_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
geometry_msgs::Pose MapUtility::getMeasuredSensorPC2Pose()
{
  return measured_sensor_pc2_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
geometry_msgs::Pose MapUtility::getSensorPC2Pose()
{
  return sensor_pc2_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
geometry_msgs::Pose MapUtility::getMeasuredMapPose()
{
  return measured_map_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
geometry_msgs::Pose MapUtility::getMapPose()
{
  return map_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<double> MapUtility::getXRange()
{
  return x_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<double> MapUtility::getYRange()
{
  return y_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<double> MapUtility::getZRange()
{
  return z_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxXMax()
{
  return bbx_x_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxXMin()
{
  return bbx_x_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxYMax()
{
  return bbx_y_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxYMin()
{
  return bbx_y_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxZMax()
{
  return bbx_z_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getBBxZMin()
{
  return bbx_z_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropXMax()
{
  return crop_x_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropXMin()
{
  return crop_x_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropYMax()
{
  return crop_y_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropYMin()
{
  return crop_y_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropZMax()
{
  return crop_z_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getCropZMin()
{
  return crop_z_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::getFilterGround()
{
  return filter_ground;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getFilterGroundThreshold()
{
  return filter_ground_threshold;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getMapResolution()
{
  return map_resolution_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getPCResolutionScale()
{
  return pc_resolution_scale;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getMaxOccupancyBeliefValue()
{
  return max_occupancy_belief_value;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getMapServerDt()
{
  return map_server_dt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::getLocalMapFlag()
{
  return local_map_flag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::getDynamicFlag()
{
  return dynamic_flag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int MapUtility::getSkipCntResetSensorRange()
{
  return skip_cnt_reset_sensor_range;
}

shared_ptr<octomap::ColorOcTree> MapUtility::getOct()
{
  return oct;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
octomap_msgs::Octomap& MapUtility::getOctMsg()
{
  return oct_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
octomap::Pointcloud& MapUtility::getOctPC()
{
  return oct_pc;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
sensor_msgs::PointCloud& MapUtility::getMeasuredPCMsg()
{
  return measured_pc_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
sensor_msgs::PointCloud2& MapUtility::getMeasuredPC2Msg()
{
  return measured_pc2_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
sensor_msgs::PointCloud& MapUtility::getPCMsg()
{
  return pc_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
sensor_msgs::PointCloud2& MapUtility::getPC2Msg()
{
  return pc2_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher MapUtility::getOctMsgPub()
{
  return pub_oct_msg_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher MapUtility::getPCMsgPub()
{
  return pc_msg_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher MapUtility::getPC2MsgPub()
{
  return pc2_msg_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setNodeHandle(ros::NodeHandle nh)
{
  nh_ = nh;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setWorldFrameName(std::string world_frame_name)
{
  world_frame_name_ = world_frame_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMapName(std::string new_map_name)
{
  map_name = new_map_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMapFrameName(std::string new_map_frame_name)
{
  map_frame_name = new_map_frame_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2MsgName(std::string new_sensor_pc2_msg_name)
{
  sensor_pc2_msg_name = new_sensor_pc2_msg_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2Direction(std::string new_sensor_pc2_direction)
{
  sensor_pc2_direction = new_sensor_pc2_direction;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2FrameName(std::string new_sensor_pc2_frame_name)
{
  sensor_pc2_frame_name = new_sensor_pc2_frame_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2MinRange(double new_sensor_pc2_min_range)
{
  sensor_pc2_min_range = new_sensor_pc2_min_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2MaxRange(double new_sensor_pc2_max_range)
{
  sensor_pc2_max_range = new_sensor_pc2_max_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2MaxYaw(double new_sensor_pc2_max_yaw)
{
  sensor_pc2_max_yaw = new_sensor_pc2_max_yaw;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2MaxPitch(double new_sensor_pc2_max_pitch)
{
  sensor_pc2_max_pitch = new_sensor_pc2_max_pitch;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorLaserMsgName(std::string new_sensor_laser_msg_name)
{
  sensor_laser_msg_name = new_sensor_laser_msg_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorLaserMaxRange(float new_sensor_laser_max_range)
{
  sensor_laser_max_range = new_sensor_laser_max_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMeasuredSensorPC2Pose(geometry_msgs::Pose new_measured_sensor_pc2_pose)
{
  measured_sensor_pc2_pose = new_measured_sensor_pc2_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorPC2Pose(geometry_msgs::Pose new_sensor_pc2_pose)
{
  sensor_pc2_pose = new_sensor_pc2_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMeasuredMapPose(geometry_msgs::Pose new_measured_map_pose)
{
  measured_map_pose = new_measured_map_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMapPose(geometry_msgs::Pose new_map_pose)
{
  map_pose = new_map_pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setXRange(double x0, double x1)
{
  x_range[0] = x0;
  x_range[1] = x1;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setYRange(double y0, double y1)
{
  y_range[0] = y0;
  y_range[1] = y1;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setZRange(double z0, double z1)
{
  z_range[0] = z0;
  z_range[1] = z1;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxXMax(double new_bbx_x_max)
{
  bbx_x_max = new_bbx_x_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxXMin(double new_bbx_x_min)
{
  bbx_x_min = new_bbx_x_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxYMax(double new_bbx_y_max)
{
  bbx_y_max = new_bbx_y_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxYMin(double new_bbx_y_min)
{
  bbx_y_min = new_bbx_y_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxZMax(double new_bbx_z_max)
{
  bbx_z_max = new_bbx_z_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setBBxZMin(double new_bbx_z_min)
{
  bbx_z_min = new_bbx_z_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropXMax(double new_crop_x_max)
{
  crop_x_max = new_crop_x_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropXMin(double new_crop_x_min)
{
  crop_x_min = new_crop_x_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropYMax(double new_crop_y_max)
{
  crop_y_max = new_crop_y_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropYMin(double new_crop_y_min)
{
  crop_y_min = new_crop_y_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropZMax(double new_crop_z_max)
{
  crop_z_max = new_crop_z_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setCropZMin(double new_crop_z_min)
{
  crop_z_min = new_crop_z_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setFilterGround(bool new_filter_ground)
{
  filter_ground =  new_filter_ground;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setFilterGroundThreshold(double new_filter_ground_threshold)
{
  filter_ground_threshold =  new_filter_ground_threshold;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMapResolution(double map_resolution)
{
  map_resolution_ = map_resolution;

  //oct = new ColorOcTree(map_resolution_);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setPCResolutionScale(double new_pc_resolution_scale)
{
  pc_resolution_scale = new_pc_resolution_scale;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value)
{
  max_occupancy_belief_value = new_max_occupancy_belief_value;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMapServerDt(double new_map_server_dt)
{
  map_server_dt = new_map_server_dt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setLocalMapFlag(bool new_local_map_flag)
{
  local_map_flag = new_local_map_flag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setDynamicFlag(bool new_dynamic_flag)
{
  dynamic_flag = new_dynamic_flag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSkipCntResetSensorRange(double new_skip_cnt_reset_sensor_range)
{
  skip_cnt_reset_sensor_range = new_skip_cnt_reset_sensor_range;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setOctPC(octomap::Pointcloud& new_oct_pc)
{
  oct_pc = new_oct_pc;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMeasuredPCMsg(sensor_msgs::PointCloud& new_measured_pc_msg)
{
  measured_pc_msg = new_measured_pc_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setMeasuredPC2Msg(sensor_msgs::PointCloud2& new_measured_pc2_msg)
{
  measured_pc2_msg = new_measured_pc2_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg = new_pc_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setPC2Msg(sensor_msgs::PointCloud2& new_pc2_msg)
{
  pc2_msg = new_pc2_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setFrameNamePkgsIgn(vector<std::string> frame_name_pkgs_ign)
{
  vec_frame_name_ign_ = frame_name_pkgs_ign;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setFrameNamePkgsMan(vector<std::string> frame_name_pkgs_man)
{
  vec_frame_name_man_ = frame_name_pkgs_man;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setPubOctMsg(std::string pub_name_oct_msg)
{
  pub_oct_msg_ = nh_.advertise<octomap_msgs::Octomap>(pub_name_oct_msg, 100);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setPubOctDistVisu(std::string pub_name_occ_dist_visu)
{
  pub_visu_occ_distance_ = nh_.advertise<visualization_msgs::Marker>(pub_name_occ_dist_visu, 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::resetMap()
{
  //oct = new ColorOcTree(map_resolution_);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  fillOctMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::initializeGazeboModelCallback(ros::NodeHandle& nh, std::string gz_model_msg)
{
  nh_ = nh;
  sub_gz_model_ = nh_.subscribe(gz_model_msg, 100, &MapUtility::gazeboModelCallback, this);
}

void MapUtility::initializeMoveitCollisionObjects()
{
  pub_moveit_collision_object_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10, true);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::transformPoint(std::string& frame_from,
                                std::string& frame_to,
                                geometry_msgs::Point& p_from_to)
{
  tf::Point p_from_tf;
  geometry_msgs::Point p_from_msg = p_from_to;
  tf::pointMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Point> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Point> p_to_stamped_tf;
  geometry_msgs::PointStamped p_to_stamped_msg;

  try
  {
    tflistener->transformPoint(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::transformPoint] Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::pointStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_from_to = p_to_stamped_msg.point;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::transformPose(std::string& frame_from,
                               std::string& frame_to,
                               geometry_msgs::Pose& p_from_to)
{
  tf::Pose p_from_tf;
  geometry_msgs::Pose p_from_msg = p_from_to;
  tf::poseMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Pose> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Pose> p_to_stamped_tf;
  geometry_msgs::PoseStamped p_to_stamped_msg;

  try
  {
    ros::Time t = ros::Time(0);
    tflistener->transformPose(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::transformPose] Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::poseStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_from_to = p_to_stamped_msg.pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::createColorOcTree(double map_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB)
{
  map_resolution_ = map_resolution;
  //oct = new ColorOcTree(map_resolution);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  int pcd_size = new_pc.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    oct->updateNode(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z, true);
    oct->setNodeColor(oct->coordToKey(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::toEgridPoint(int ind, geometry_msgs::Point& po)
{
  int egrid_vnum_x_min = abs(egrid_bbx_min_.x) / egrid_resolution_;
  int egrid_vnum_y_min = abs(egrid_bbx_min_.y) / egrid_resolution_;
  int egrid_vnum_z_min = abs(egrid_bbx_min_.z) / egrid_resolution_;

  int egrid_vnumx = abs(egrid_bbx_max_.x - egrid_bbx_min_.x) / egrid_resolution_;
  int egrid_vnumy = abs(egrid_bbx_max_.y - egrid_bbx_min_.y) / egrid_resolution_;

  po.x = egrid_resolution_ * ((ind % (egrid_vnumx * egrid_vnumy)) % egrid_vnumx - egrid_vnum_x_min) + 0.5 * egrid_resolution_;
  po.y = egrid_resolution_ * ((ind % (egrid_vnumx * egrid_vnumy)) / egrid_vnumx - egrid_vnum_y_min) + 0.5 * egrid_resolution_;
  po.z = egrid_resolution_ * (ind / (egrid_vnumx * egrid_vnumy) - egrid_vnum_z_min) + 0.5 * egrid_resolution_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int MapUtility::toEgridIndex(double pos, int egrid_vnum)
{
  return egrid_vnum + floor(pos / egrid_resolution_);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int MapUtility::toEgridLinIndex(tf::Vector3 po)
{
  int egrid_vnum_x_min = abs(egrid_bbx_min_.x) / egrid_resolution_;
  int egrid_vnum_y_min = abs(egrid_bbx_min_.y) / egrid_resolution_;
  int egrid_vnum_z_min = abs(egrid_bbx_min_.z) / egrid_resolution_;

  int egrid_vnumx = abs(egrid_bbx_max_.x - egrid_bbx_min_.x) / egrid_resolution_;
  int egrid_vnumy = abs(egrid_bbx_max_.y - egrid_bbx_min_.y) / egrid_resolution_;

  int ind_x = toEgridIndex(po.x(), egrid_vnum_x_min);
  int ind_y = toEgridIndex(po.y(), egrid_vnum_y_min);
  int ind_z = toEgridIndex(po.z(), egrid_vnum_z_min);

  return (ind_x + ind_y * egrid_vnumx + ind_z * egrid_vnumx * egrid_vnumy);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::initializeEgoGrid(std::string egrid_frame_name, 
                                   double egrid_resolution, 
                                   geometry_msgs::Point egrid_bbx_min, 
                                   geometry_msgs::Point egrid_bbx_max,
                                   double egrid_occ_threshold)
{
  //std::cout << "[MapUtility::initializeEgoGrid] START" << std::endl;

  egrid_frame_name_ = egrid_frame_name;
  egrid_resolution_ = egrid_resolution;
  egrid_bbx_min_ = egrid_bbx_min;
  egrid_bbx_max_ = egrid_bbx_max;
  egrid_occ_threshold_ = egrid_occ_threshold;

  double egrid_vnumx = abs(egrid_bbx_max_.x - egrid_bbx_min_.x) / egrid_resolution_;
  double egrid_vnumy = abs(egrid_bbx_max_.y - egrid_bbx_min_.y) / egrid_resolution_;
  double egrid_vnumz = abs(egrid_bbx_max_.z - egrid_bbx_min_.z) / egrid_resolution_;
  int total_voxel_cnt = egrid_vnumx * egrid_vnumy * egrid_vnumz;

  egrid_pos_.resize(total_voxel_cnt);
  egrid_pc_msg_.points.clear();

  for(int v = 0; v < total_voxel_cnt; v++)
  {
    geometry_msgs::Point po;
    toEgridPoint(v, po);
    egrid_pos_[v] = po;

    geometry_msgs::Point32 epo;
    epo.x = po.x;
    epo.y = po.y;
    epo.z = po.z;
    egrid_pc_msg_.points.push_back(epo);
  }

  pub_egrid_pc_msg_ = nh_.advertise<sensor_msgs::PointCloud>("ego_grid_pos", 10);
  pub_egrid_target_pc_msg_ = nh_.advertise<sensor_msgs::PointCloud>("ego_grid_target_pos", 10);
  pub_egrid_occ_pc_msg_ = nh_.advertise<sensor_msgs::PointCloud>("ego_grid_occupancy", 10);

  //std::cout << "[MapUtility::initializeEgoGrid] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
template <typename T>
vector<T> MapUtility::extractPointsAroundCenter(T center, double vdim)
{
  double half_vdim = 0.5 * vdim;

  vector<geometry_msgs::Point32> opc;
  geometry_msgs::Point minp;
  geometry_msgs::Point maxp;

  // (-x, -y, -z)
  minp.x = center.x - half_vdim;                    
  minp.y = center.y - half_vdim;
  minp.z = center.z - half_vdim;

  // (+x, +y, +z)
  maxp.x = center.x + half_vdim;                    
  maxp.y = center.y + half_vdim;
  maxp.z = center.z + half_vdim;

  for(double i = minp.x; i < maxp.x; i += vdim)
  {
    for(double j = minp.y; j < maxp.y; j += vdim)
    {
      for(double k = minp.z; k < maxp.z; k += vdim)
      {
        geometry_msgs::Point32 newp;
        newp.x = i;
        newp.y = j;
        newp.z = k;
        opc.push_back(newp);

        if(i == minp.x)
        {
          geometry_msgs::Point32 newpx;
          newpx.x = maxp.x;
          newpx.y = j;
          newpx.z = k;
          opc.push_back(newpx);
        }
        
        if(j == minp.y)
        {
          geometry_msgs::Point32 newpy;
          newpy.x = i;
          newpy.y = maxp.y;
          newpy.z = k;
          opc.push_back(newpy);
        }

        if(k == minp.z)
        {
          geometry_msgs::Point32 newpz;
          newpz.x = i;
          newpz.y = j;
          newpz.z = maxp.z;
          opc.push_back(newpz);
        }
        
        if(i == minp.x && j == minp.y)
        {
          geometry_msgs::Point32 newpxy;
          newpxy.x = maxp.x;
          newpxy.y = maxp.y;
          newpxy.z = k;
          opc.push_back(newpxy);
        }      

        if(i == minp.x && k == minp.z)
        {
          geometry_msgs::Point32 newpxz;
          newpxz.x = maxp.x;
          newpxz.y = j;
          newpxz.z = maxp.z;
          opc.push_back(newpxz);
        }
        
        if(j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpyz;
          newpyz.x = i;
          newpyz.y = maxp.y;
          newpyz.z = maxp.z;
          opc.push_back(newpyz);
        }
        
        if(i == minp.x && j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpxyz;
          newpxyz.x = maxp.x;
          newpxyz.y = maxp.y;
          newpxyz.z = maxp.z;
          opc.push_back(newpxyz);
        }
      }
    }
  }
  return opc;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillEgoTargetPC()
{
  std::cout << "[MapUtility::fillEgoTargetPC] START" << std::endl;

  std::vector<double> dist_vec = {1, 2};
  geometry_msgs::Point32 center;
  center.x = 0;
  center.y = 0;
  center.z = 0;

  egrid_target_pc_msg_.points.clear();

  std::vector<geometry_msgs::Point32> tmp_tps;
  for (auto d : dist_vec)
  {
    tmp_tps = extractPointsAroundCenter(center, d);
    egrid_target_pc_msg_.points.insert(egrid_target_pc_msg_.points.end(),
                                       std::make_move_iterator(tmp_tps.begin()),
                                       std::make_move_iterator(tmp_tps.end()));
  }

  std::cout << "[MapUtility::fillEgoTargetPC] egrid_target_pc_msg_.points size: " << egrid_target_pc_msg_.points.size() << std::endl;

  std::cout << "[MapUtility::fillEgoTargetPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillOct(sensor_msgs::PointCloud& pc_msg)
{
  oct->clear();

  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct->updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillOctFromMeasuredPCMsg()
{
  oct->clear();

  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct->updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillOctMsgFromOct()
{
  //cout << "[MapUtility::fillOctMsgFromOct] START" << endl;
  oct_msg.data.clear();
  oct_msg.header.frame_id = world_frame_name_;
  oct_msg.binary = false;
  oct_msg.id = map_name;
  oct_msg.resolution = map_resolution_;
  octomap_msgs::fullMapToMsg(*oct, oct_msg);
  //cout << "[MapUtility::fillOctMsgFromOct] END" << endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillOctMsgFromOct(std::string octMsgName)
{
  //cout << "[MapUtility::fillOctMsgFromOct(1)] START" << endl;
  //cout << "[MapUtility::fillOctMsgFromOct(1)] octMsgName: " << octMsgName << endl;

  if (octMsgName == obj_name_conveyor_)
  {
    oct_msg_conveyor_.data.clear();
    oct_msg_conveyor_.header.frame_id = world_frame_name_;
    oct_msg_conveyor_.binary = false;
    oct_msg_conveyor_.id = map_name;
    oct_msg_conveyor_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_conveyor_, oct_msg_conveyor_);
  }
  else if (octMsgName == obj_name_normal_pkg_)
  {
    oct_msg_pkg_normal_.data.clear();
    oct_msg_pkg_normal_.header.frame_id = world_frame_name_;
    oct_msg_pkg_normal_.binary = false;
    oct_msg_pkg_normal_.id = map_name;
    oct_msg_pkg_normal_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_pkg_normal_, oct_msg_pkg_normal_);
  }
  else if (octMsgName == obj_name_long_pkg_)
  {
    oct_msg_pkg_long_.data.clear();
    oct_msg_pkg_long_.header.frame_id = world_frame_name_;
    oct_msg_pkg_long_.binary = false;
    oct_msg_pkg_long_.id = map_name;
    oct_msg_pkg_long_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_pkg_long_, oct_msg_pkg_long_);
  }
  else if (octMsgName == obj_name_longwide_pkg_)
  {
    oct_msg_pkg_longwide_.data.clear();
    oct_msg_pkg_longwide_.header.frame_id = world_frame_name_;
    oct_msg_pkg_longwide_.binary = false;
    oct_msg_pkg_longwide_.id = map_name;
    oct_msg_pkg_longwide_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_pkg_longwide_, oct_msg_pkg_longwide_);
  }
  else if (octMsgName == obj_name_red_cube_)
  {
    oct_msg_red_cube_.data.clear();
    oct_msg_red_cube_.header.frame_id = world_frame_name_;
    oct_msg_red_cube_.binary = false;
    oct_msg_red_cube_.id = map_name;
    oct_msg_red_cube_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_red_cube_, oct_msg_red_cube_);
  }
  else if (octMsgName == obj_name_green_cube_)
  {
    oct_msg_green_cube_.data.clear();
    oct_msg_green_cube_.header.frame_id = world_frame_name_;
    oct_msg_green_cube_.binary = false;
    oct_msg_green_cube_.id = map_name;
    oct_msg_green_cube_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_green_cube_, oct_msg_green_cube_);
  }
  else if (octMsgName == obj_name_blue_cube_)
  {
    oct_msg_blue_cube_.data.clear();
    oct_msg_blue_cube_.header.frame_id = world_frame_name_;
    oct_msg_blue_cube_.binary = false;
    oct_msg_blue_cube_.id = map_name;
    oct_msg_blue_cube_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_blue_cube_, oct_msg_blue_cube_);
  }
  else if (octMsgName == obj_name_actor0_)
  {
    oct_msg_actor0_.data.clear();
    oct_msg_actor0_.header.frame_id = world_frame_name_;
    oct_msg_actor0_.binary = false;
    oct_msg_actor0_.id = map_name;
    oct_msg_actor0_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_actor0_, oct_msg_actor0_);
  }
  else if (octMsgName == obj_name_actor1_)
  {
    oct_msg_actor1_.data.clear();
    oct_msg_actor1_.header.frame_id = world_frame_name_;
    oct_msg_actor1_.binary = false;
    oct_msg_actor1_.id = map_name;
    oct_msg_actor1_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_actor1_, oct_msg_actor1_);
  }
  else if (octMsgName == obj_name_bin_)
  {
    oct_msg_bin_.data.clear();
    oct_msg_bin_.header.frame_id = world_frame_name_;
    oct_msg_bin_.binary = false;
    oct_msg_bin_.id = map_name;
    oct_msg_bin_.resolution = map_resolution_;
    octomap_msgs::fullMapToMsg(*oct_bin_, oct_msg_bin_);

    //cout << "[MapUtility::fillOctMsgFromOct(1)] oct_msg_bin_.data size: " << oct_msg_bin_.data.size() << endl;
  }

  //cout << "[MapUtility::fillOctMsgFromOct(1)] END" << endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillPCMsgFromOct()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct->begin(); it != oct->end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    pc_msg.points.push_back(op);
  }
  pc_msg.header.frame_id = world_frame_name_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillPCMsgFromOctByResolutionScale()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct->begin(); it != oct->end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    vector<geometry_msgs::Point32> opc = extractPointsAroundCenter(op, map_resolution_);
    for(int i = 0; i < opc.size(); i++)
    {
      pc_msg.points.push_back(opc[i]);
    }
  }
  pc_msg.header.frame_id = world_frame_name_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillDebugArrayVisu(vector<tf::Vector3> v)
{
  debug_array_visu.markers.clear();

  for(int i = 0; i < v.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.ns = "point" + to_string(i);
    marker.id = i;
    marker.header.frame_id = world_frame_name_;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = v[i].x();
    marker.pose.position.y = v[i].y();
    marker.pose.position.z = v[i].z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    debug_array_visu.markers.push_back(marker); 
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::fillDebugVisu(vector<tf::Vector3> v)
{
  debug_visu.points.clear();

  debug_visu.ns = "points";
  debug_visu.id = 1;
  debug_visu.header.frame_id = world_frame_name_;
  debug_visu.type = visualization_msgs::Marker::POINTS;
  debug_visu.action = visualization_msgs::Marker::ADD;
  debug_visu.pose.orientation.w = 1.0;
  debug_visu.scale.x = 0.04;
  debug_visu.scale.y = 0.04;
  debug_visu.scale.z = 0.04;
  debug_visu.color.r = 1.0;
  debug_visu.color.g = 1.0;
  debug_visu.color.b = 0.0;
  debug_visu.color.a = 1.0;

  debug_visu.points.clear();
  for(int i = 0; i < v.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = v[i].x();
    p.y = v[i].y();
    p.z = v[i].z();

    debug_visu.points.push_back(p); 
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::insertToOctFromOctPC(octomap::point3d sensor_pc2_origin)
{
  oct -> insertPointCloud(oct_pc, sensor_pc2_origin, sensor_pc2_max_range, false, true);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addToOct(sensor_msgs::PointCloud& pc_msg)
{
  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct->updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addToOct(sensor_msgs::PointCloud2& pc2_msg)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      oct->updateNode(*iter_x, *iter_y, *iter_z, true);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateObjectOct(sensor_msgs::PointCloud2& pc2_msg, std::string objName)
{
  //std::cout << "[MapUtility::updateObjectOct] START" << std::endl;
  //std::cout << "[MapUtility::updateObjectOct] objName: " << objName << std::endl;

  oct_conveyor_->clear();
  oct_pkg_normal_->clear();
  oct_pkg_long_->clear();
  oct_pkg_longwide_->clear();
  oct_red_cube_->clear();
  oct_green_cube_->clear();
  oct_blue_cube_->clear();
  oct_actor0_->clear();
  oct_actor1_->clear();
  oct_bin_->clear();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

  /// NUA TODO: GENERALIZE!
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      if (objName == obj_name_conveyor_) 
      {
        oct_conveyor_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_normal_pkg_) 
      {
        oct_pkg_normal_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_long_pkg_) 
      {
        oct_pkg_long_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_longwide_pkg_) 
      { 
        oct_pkg_longwide_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_red_cube_) 
      { 
        oct_red_cube_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_green_cube_) 
      {
        oct_green_cube_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_blue_cube_) 
      {
        oct_blue_cube_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_actor0_) 
      {
        oct_actor0_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_actor1_) 
      {
        oct_actor1_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
      else if (objName == obj_name_bin_) 
      {
        oct_bin_->updateNode(*iter_x, *iter_y, *iter_z, true);
      }
    }
  }

  //std::cout << "[MapUtility::updateObjectOct] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addToOctFromMeasuredPCMsg()
{
  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct -> updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
  fillOctMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addToPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg.header = new_pc_msg.header;
  pc_msg.channels = new_pc_msg.channels;
  pc_msg.points.insert(end(pc_msg.points), begin(new_pc_msg.points), end(new_pc_msg.points));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addToPCMsg(geometry_msgs::Point32 new_point)
{
  pc_msg.points.push_back(new_point);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::clearMeasuredPCMsg()
{
  measured_pc_msg.points.clear();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::clearPCMsg()
{
  pc_msg.points.clear();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = oct->search(x, y, z);
  if(node)
  {
    return oct->isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupied(geometry_msgs::Point po)
{
  OcTreeNode* node = oct->search(po.x, po.y, po.z);
  if(node)
  {
    return oct->isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupied(geometry_msgs::Point32 po)
{
  OcTreeNode* node = oct->search(po.x, po.y, po.z);
  if(node)
  {
    return oct->isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getOctOccupancy(tf::Vector3 po)
{
  OcTreeNode* node = oct->search(po.x(), po.y(), po.z());
  if(node)
  {
    return node->getOccupancy();
  }
  else
  {
    return -1.0;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::setSensorRangeQuery(float query_point_resolution)
{
  // SET THE QUERY POINT (BOX)
  fcl::Box<float> query(query_point_resolution, query_point_resolution, query_point_resolution);
  query_sharedPtr = std::make_shared< fcl::Box<float> > (query);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::constructCameraSensorRange()
{
  tf::Transform aligner;
  aligner.setIdentity();
  tf::Quaternion aligner_q(0, 0, 0, 1);
  if (sensor_pc2_direction == "y")
  {
    //cout << "MapUtility::constructCameraSensorRange -> Sensor is in y direction!" << endl;
    aligner_q.setRPY(0, 0, 0.5*M_PI);
  }
  else if (sensor_pc2_direction == "z")
  {
    //cout << "MapUtility::constructCameraSensorRange -> Sensor is in z direction!" << endl;
    aligner_q.setRPY(0, -0.5*M_PI, 0);
  }

  aligner.setRotation(aligner_q);
  
  // SET THE CAMERA SENSOR RANGE AS POLYTOPE
  tf::Vector3 vert;

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v0( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v1( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v2( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v3( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v4( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v5( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v6( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v7( vert.x(), vert.y(), vert.z() );

  std::vector< fcl::Vector3<float> > vertices;
  auto vertices_sharedPtr = std::make_shared< std::vector< fcl::Vector3<float> > > (vertices);

  vertices_sharedPtr.get()->push_back(v0);
  vertices_sharedPtr.get()->push_back(v1);
  vertices_sharedPtr.get()->push_back(v2);
  vertices_sharedPtr.get()->push_back(v3);
  vertices_sharedPtr.get()->push_back(v4);
  vertices_sharedPtr.get()->push_back(v5);
  vertices_sharedPtr.get()->push_back(v6);
  vertices_sharedPtr.get()->push_back(v7);

  vector<int> f0 = {4,0,1,2,3};
  vector<int> f1 = {4,4,5,1,0};
  vector<int> f2 = {4,7,6,5,4};
  vector<int> f3 = {4,3,2,6,7};
  vector<int> f4 = {4,4,0,3,7};
  vector<int> f5 = {4,1,5,6,2};

  vector<int> faces;
  auto faces_sharedPtr = std::make_shared< std::vector<int> > (faces);

  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f0.begin(), f0.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f1.begin(), f1.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f2.begin(), f2.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f3.begin(), f3.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f4.begin(), f4.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f5.begin(), f5.end());

  int num_faces = 6;
  bool throw_if_invalid = true;

  fcl::Convex<float> sensor_pc2_range_hull(vertices_sharedPtr, num_faces, faces_sharedPtr, throw_if_invalid);

  sensor_pc2_range_sharedPtr = std::make_shared< fcl::Convex<float> > (sensor_pc2_range_hull);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::constructLaserSensorRange(float sensor_laser_z_range)
{
  // SET THE LASER SENSOR COVERAGE AS CYLINDER
  fcl::Cylinder<float> sensor_laser_hull(sensor_laser_max_range, sensor_laser_z_range);
  sensor_laser_range_sharedPtr = std::make_shared< fcl::Cylinder<float> > (sensor_laser_hull);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isInSensorCameraRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the camera sensor range collision obj
    fcl::CollisionObjectf* sensor_pc2_range_collision_obj = new fcl::CollisionObjectf(sensor_pc2_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_pc2_range_result.clear();

    fcl::collide(sensor_pc2_range_collision_obj, query_collision_obj, sensor_pc2_range_request, sensor_pc2_range_result);

    delete query_collision_obj;
    delete sensor_pc2_range_collision_obj;
    
    return sensor_pc2_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isInSensorLaserRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the laser sensor range collision obj
    fcl::CollisionObjectf* laser_sensor_collision_obj = new fcl::CollisionObjectf(sensor_laser_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_laser_range_result.clear();

    fcl::collide(laser_sensor_collision_obj, query_collision_obj, sensor_laser_range_request, sensor_laser_range_result);

    delete query_collision_obj;
    delete laser_sensor_collision_obj;
    
    return sensor_laser_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad)
{
  return (po.x >= center.x - rad) && (po.x <= center.x + rad) && (po.y >= center.y - rad) && (po.y <= center.y + rad) && (po.z >= center.z - rad) && (po.z <= center.z + rad);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isInCube(geometry_msgs::Point32 po, geometry_msgs::Point center, double rad)
{
  return (po.x >= center.x - rad) && (po.x <= center.x + rad) && (po.y >= center.y - rad) && (po.y <= center.y + rad) && (po.z >= center.z - rad) && (po.z <= center.z + rad);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  double free_rad = 2 * map_resolution_;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal)
{
  double free_rad = 2 * map_resolution_;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::isOccupiedByGoal(geometry_msgs::Point32 po, vector<geometry_msgs::Pose> goal)
{
  double free_rad = 2 * map_resolution_;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addStaticObstacleByResolutionScale2PCMsg(geometry_msgs::Point32 po)
{
  vector<geometry_msgs::Point32> opc = extractPointsAroundCenter(po, map_resolution_);
  for(int i = 0; i < opc.size(); i++)
  {
    pc_msg.points.push_back(opc[i]);
  }
  pc_msg.header.frame_id = world_frame_name_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::addStaticObstacle(double x, 
                                   double y, 
                                   double z, 
                                   bool constraint_flag, 
                                   vector<geometry_msgs::Pose> goal, 
                                   geometry_msgs::Point robot_center, 
                                   double robot_free_rad, 
                                   vector<int> color_RGB)
{
  geometry_msgs::Point32 po;
  po.x = x;
  po.y = y;
  po.z = z;

  if( constraint_flag && (isOccupied(x, y, z) || isOccupiedByGoal(x, y, z, goal) || isInCube(po, robot_center, robot_free_rad + map_resolution_)) )
  {
    return false;
  }
  else
  {
    oct -> updateNode(x, y, z, true);
    oct -> setNodeColor(oct -> coordToKey(x, y, z), color_RGB[0], color_RGB[1], color_RGB[2]);
    fillOctMsgFromOct();
    addStaticObstacleByResolutionScale2PCMsg(po);
    return true;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<bool> MapUtility::addStaticObstacle(sensor_msgs::PointCloud& pcd, 
                                           bool constraint_flag, 
                                           vector<geometry_msgs::Pose> goal, 
                                           geometry_msgs::Point robot_center, 
                                           double robot_free_rad, 
                                           vector<int> color_RGB)
{
  vector<bool> pc_add_result;
  int pcd_size = pcd.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    pc_add_result.push_back( addStaticObstacle(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z, constraint_flag, goal, robot_center, robot_free_rad, color_RGB) );
  }
  return pc_add_result;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addMoveitCollisionObjects()
{
  std::cout << "[MapUtility::addMoveitCollisionObjects] START" << std::endl;

  // Add the object
  moveit_msgs::CollisionObject mco;
  mco.id = "table1";
  
  // Define the primitive and its dimensions.
  mco.primitives.resize(1);
  mco.primitives[0].type = mco.primitives[0].BOX;
  mco.primitives[0].dimensions.resize(3);
  mco.primitives[0].dimensions[0] = 0.2;
  mco.primitives[0].dimensions[1] = 1.4;
  mco.primitives[0].dimensions[2] = 0.4;

  // Define the pose of the table.
  /*
  mco.primitive_poses.resize(1);
  mco.primitive_poses[0].position.x = 0.0;
  mco.primitive_poses[0].position.y = 0.0;
  mco.primitive_poses[0].position.z = 0.0;
  mco.primitive_poses[0].orientation.x = 0.0;
  mco.primitive_poses[0].orientation.y = 0.0;
  mco.primitive_poses[0].orientation.z = 0.0;
  mco.primitive_poses[0].orientation.w = 1.0;
  */

  mco.pose.position.x = 1.0;
  mco.pose.position.y = 0.0;
  mco.pose.position.z = 0.2;

  mco.pose.orientation.x = 0.0;
  mco.pose.orientation.y = 0.0;
  mco.pose.orientation.z = 0.0;
  mco.pose.orientation.w = 1.0;

  mco.operation = mco.ADD;

  mco.header.frame_id = "base_link";
  mco.header.stamp = ros::Time::now();
  mco.header.seq++;

  moveit_collision_objects_.push_back(mco);
  pub_moveit_collision_object_.publish(mco);

  std::cout << "[MapUtility::addMoveitCollisionObjects] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addMoveitCollisionObjects(std::string& obj_id,
                                           geometry_msgs::Point& obj_dim, 
                                           geometry_msgs::Pose& obj_pose_wrt_world)
{
  //std::cout << "[MapUtility::addMoveitCollisionObjects(3)] START" << std::endl;

  std::string base_frame_name = "base_link";
  geometry_msgs::Pose obj_pose_wrt_base = obj_pose_wrt_world;
  transformPose(world_frame_name_, base_frame_name, obj_pose_wrt_base);

  moveit_msgs::CollisionObject co;
  co.id = obj_id;
  co.header.frame_id = base_frame_name;

  co.primitives.resize(1);
  co.primitives[0].type = co.primitives[0].BOX;
  co.primitives[0].dimensions.resize(3);
  co.primitives[0].dimensions[0] = obj_dim.x;
  co.primitives[0].dimensions[1] = obj_dim.y;
  co.primitives[0].dimensions[2] = obj_dim.z;

  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.0;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.0;
  co.primitive_poses[0].orientation.x = 0.0;
  co.primitive_poses[0].orientation.y = 0.0;
  co.primitive_poses[0].orientation.z = 0.0;
  co.primitive_poses[0].orientation.w = 1.0;

  co.pose = obj_pose_wrt_base;

  co.operation = co.ADD;

  moveit_collision_objects_.push_back(co);

  //std::cout << "[MapUtility::addMoveitCollisionObjects(3)] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateMoveitCollisionObjects()
{
  //cout << "[MapUtility::updateMoveitCollisionObjects] START" << std::endl;

  moveit_collision_objects_.clear();

  gazebo_msgs::ModelStates ms = gz_model_states_;

  std::string gz_model_name_tmp, tf_name_tmp;

  for (size_t i = 0; i < ms.name.size(); i++)
  {
    gz_model_name_tmp = ms.name[i];

    for (size_t j = 0; j < vec_frame_name_ign_.size(); j++)
    {
      tf_name_tmp = vec_frame_name_ign_[j];

      // NUA TODO: Try to fix this by finding a way to generalize for multiple objects with the same namespace!
      if ( tf_name_tmp == "actor" && (gz_model_name_tmp == "0" || gz_model_name_tmp == "1") )
      {
        tf_name_tmp += gz_model_name_tmp;
        gz_model_name_tmp = tf_name_tmp;
      }

      if (gz_model_name_tmp == tf_name_tmp)
      {
        addMoveitCollisionObjects(gz_model_name_tmp, vec_obj_dim_ign_[j], ms.pose[i]);
        break;
      }
    }
  }

  //cout << "[MapUtility::updateMoveitCollisionObjects] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::createRandomStaticObstacleMap(int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(x_range[0], x_range[1]);
    rp.position.y = randdouble(y_range[0], y_range[1]);
    rp.position.z = randdouble(z_range[0], z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution_)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(x_range[0], x_range[1]);
      rp.position.y = randdouble(y_range[0], y_range[1]);
      rp.position.z = randdouble(z_range[0], z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);  
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution_)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Point32 rp;
    rp.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.z = randdouble(new_z_range[0], new_z_range[1]);

    // Check for duplicates, goal and initial robot occupancy
    while( isOccupied(rp) || (constraint_flag && isOccupiedByGoal(rp, goal) || isInCube(rp, robot_center, robot_free_rad + map_resolution_)) )
    {
      rp.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.x, rp.y, rp.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.x, rp.y, rp.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    addStaticObstacleByResolutionScale2PCMsg(rp);
  }
  fillOctMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::createRandomMapSet(std::string mapset_name, int map_cnt, int map_occupancy_count)
{
  vector<double> goal_x_range;
  goal_x_range.push_back(x_range[0] + 4);
  goal_x_range.push_back(x_range[1] - 4);

  vector<double> goal_y_range;
  goal_y_range.push_back(y_range[0] + 4);
  goal_y_range.push_back(y_range[1] - 4);

  vector<double> goal_z_range;
  goal_z_range.push_back(z_range[0] + 4);
  goal_z_range.push_back(z_range[1] - 4);
  
  for (int i = 0; i < map_cnt; i++)
  {
    createRandomStaticObstacleMap(map_occupancy_count);
    saveMap("mapset/" + mapset_name + "/map" + to_string(i));
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::crop(sensor_msgs::PointCloud2& cloud_in, octomap::point3d lowerBound, octomap::point3d upperBound, octomap::Pointcloud& cloud_out, bool keep_in)
{
  cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  float x,y,z;

  min_x = lowerBound(0); min_y = lowerBound(1); min_z = lowerBound(2);
  max_x = upperBound(0); max_y = upperBound(1); max_z = upperBound(2);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      if( (*iter_x >= min_x) && (*iter_x <= max_x) &&
          (*iter_y >= min_y) && (*iter_y <= max_y) &&
          (*iter_z >= min_z) && (*iter_z <= max_z) )
      {
        if(keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
      else
      {
        if(!keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::cropOctPCFromPC2Msg(octomap::point3d lowerBound, octomap::point3d upperBound, bool keep_in)
{
  oct_pc.clear();

  /*
  if (pc2_msg.data.size() > 0 && laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve( (pc2_msg.data.size() + laser_pc2_msg.data.size()) / (pc2_msg.point_step + laser_pc2_msg.point_step) );
  }
  else if(pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);
  }
  else if(laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);
  }
  */

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);

            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
        else
        {
          if(!keep_in)
          { 
            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
        else
        {
          if(!keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateOctPC()
{
  oct_pc.clear();

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateOct()
{
  sensor_msgs::PointCloud2 pc2_msg_scan = pc2_msg_scan_;
  if (pc2_msg_scan.data.size() > 0)
  {
    oct->clear();
    addToOct(pc2_msg_scan);
    fillOctMsgFromOct();
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateOct(std::string oct_msg_name)
{
  sub_oct_msg_ = nh_.subscribe(oct_msg_name, 10, &MapUtility::octMsgCallback, this);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateObjectOct()
{
  std::vector<string> vec_frame_name_obj = vec_frame_name_obj_;
  std::vector<sensor_msgs::PointCloud2> vec_pc2_msg_obj_wrt_world = vec_pc2_msg_obj_wrt_world_;

  for (size_t i = 0; i < vec_pc2_msg_obj_wrt_world.size(); i++)
  {
    if (vec_pc2_msg_obj_wrt_world[i].data.size() > 0)
    {
      updateObjectOct(vec_pc2_msg_obj_wrt_world[i], vec_frame_name_obj[i]);
      fillOctMsgFromOct(vec_frame_name_obj[i]);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateEgoGrid()
{
  //std::cout << "[MapUtility::updateEgoGrid] START" << std::endl;

  int vox_index;
  int total_voxel_cnt = egrid_pos_.size();
  egrid_occ_.clear();
  egrid_hist_.clear();
  egrid_occ_.resize(total_voxel_cnt);
  egrid_hist_.resize(total_voxel_cnt);
  egrid_occ_pc_msg_.points.clear();

  tf::Vector3 egrid_bbx_min(egrid_bbx_min_.x, egrid_bbx_min_.y, egrid_bbx_min_.z);
  tf::Vector3 egrid_bbx_max(egrid_bbx_max_.x, egrid_bbx_max_.y, egrid_bbx_max_.z);

  try
  {
    tflistener->waitForTransform(world_frame_name_, egrid_frame_name_, ros::Time(0), ros::Duration(5.0));
    tflistener->lookupTransform(world_frame_name_, egrid_frame_name_, ros::Time(0), transform_grid_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::updateEgoGrid] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  for(octomap::ColorOcTree::iterator it = oct->begin(); it != oct->end(); ++it)
  {
    tf::Vector3 op_wrt_world(it.getX(), it.getY(), it.getZ());
    tf::Vector3 op_wrt_robot = transform_grid_wrt_world_.inverse() * op_wrt_world;

    if ( isInBBx(op_wrt_robot, egrid_bbx_min, egrid_bbx_max) && isOccupied(op_wrt_world.x(), op_wrt_world.y(), op_wrt_world.z()) )
    {
      vox_index = toEgridLinIndex(op_wrt_robot);

      if ( vox_index >= 0 && vox_index < total_voxel_cnt )
      {
        egrid_hist_[vox_index] += 1;
        egrid_occ_[vox_index] += getOctOccupancy(op_wrt_world);

        geometry_msgs::Point32 po;
        po.x = op_wrt_world.x();
        po.y = op_wrt_world.y();
        po.z = op_wrt_world.z();
        egrid_occ_pc_msg_.points.push_back(po);
      }
    }
  }

  for (size_t i = 0; i < total_voxel_cnt; i++)
  {
    if (egrid_hist_[i] > 0)
    {
      egrid_occ_[vox_index] /= egrid_hist_[vox_index];
    }
  }
  

  //std::cout << "[MapUtility::updateEgoGrid] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateOccGrid()
{
  //std::cout << "[MapUtility::updateOccGrid] START" << std::endl;

  occ_grid_msg_.data.clear();

  int nx = abs(egrid_bbx_max_.x - egrid_bbx_min_.x) / egrid_resolution_;
  int ny = abs(egrid_bbx_max_.y - egrid_bbx_min_.y) / egrid_resolution_;
  int nz = abs(egrid_bbx_max_.z - egrid_bbx_min_.z) / egrid_resolution_;

  //std::cout << "[MapUtility::updateOccGrid] nx: " << nx << std::endl;
  //std::cout << "[MapUtility::updateOccGrid] ny: " << ny << std::endl;
  //std::cout << "[MapUtility::updateOccGrid] nz: " << nz << std::endl;

  occ_grid_msg_.info.map_load_time = ros::Time::now();
  occ_grid_msg_.info.resolution = egrid_resolution_;
  occ_grid_msg_.info.width = ny;
  occ_grid_msg_.info.height = nx;
  occ_grid_msg_.info.origin.position = egrid_pos_[0];
  //tf::quaternionTFToMsg (transform_grid_wrt_world_.getRotation(), occ_grid_msg_.info.origin.orientation);

  std::vector<double> occ_val_vec;
  double occ_val;
  double occ_val_mean;
  double occ_val_mean_tmp;
  int offset = 0;
  int nxy = nx*ny;
  int nxyz = nx*ny*nz;
  bool flag;

  //std::cout << "[MapUtility::updateOccGrid] nxy: " << nxy << std::endl;
  //std::cout << "[MapUtility::updateOccGrid] nxyz: " << nxyz << std::endl;

  for (int i = 0; i < nxy; i++)
  {
    occ_val_vec.clear();
    flag = false;
    for (int j = 0; j < nz; j++)
    {
      offset = j * nxy;
      occ_val_vec.push_back(egrid_occ_[i + offset]);      
    }

    occ_val = *max_element(occ_val_vec.begin(), occ_val_vec.end()) * 100;

    if (occ_val >= egrid_occ_threshold_)
    {
      occ_grid_msg_.data.push_back(100);
    }
    else if (occ_val < 0)
    {
      occ_grid_msg_.data.push_back(-1);
    }
    else
    {
      occ_grid_msg_.data.push_back(0);
    }
  }

  //std::cout << "[MapUtility::updateOccGrid] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::pointcloud2ToOctPc2(const sensor_msgs::PointCloud2& cloud_pc2, octomap::Pointcloud& cloud_octomap)
{
  cloud_octomap.reserve(cloud_pc2.data.size() / cloud_pc2.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_pc2, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      cloud_octomap.push_back(*iter_x, *iter_y, *iter_z);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishOctMsg()
{
  oct_msg.header.frame_id = world_frame_name_;
  //oct_msg.header.seq++;
  oct_msg.header.stamp = ros::Time::now();
  pub_oct_msg_.publish(oct_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishObjectOctMsg()
{
  //std::cout << "[MapUtility::publishObjectOctMsg] START" << std::endl;

  oct_msg_conveyor_.header.frame_id = world_frame_name_;
  oct_msg_conveyor_.header.stamp = ros::Time::now();
  pub_oct_msg_conveyor_.publish(oct_msg_conveyor_);

  oct_msg_pkg_normal_.header.frame_id = world_frame_name_;
  oct_msg_pkg_normal_.header.stamp = ros::Time::now();
  pub_oct_msg_pkg_normal_.publish(oct_msg_pkg_normal_);

  oct_msg_pkg_long_.header.frame_id = world_frame_name_;
  oct_msg_pkg_long_.header.stamp = ros::Time::now();
  pub_oct_msg_pkg_long_.publish(oct_msg_pkg_long_);

  oct_msg_pkg_longwide_.header.frame_id = world_frame_name_;
  oct_msg_pkg_longwide_.header.stamp = ros::Time::now();
  pub_oct_msg_pkg_longwide_.publish(oct_msg_pkg_longwide_);

  oct_msg_red_cube_.header.frame_id = world_frame_name_;
  oct_msg_red_cube_.header.stamp = ros::Time::now();
  pub_oct_msg_red_cube_.publish(oct_msg_red_cube_);

  oct_msg_green_cube_.header.frame_id = world_frame_name_;
  oct_msg_green_cube_.header.stamp = ros::Time::now();
  pub_oct_msg_green_cube_.publish(oct_msg_green_cube_);

  oct_msg_blue_cube_.header.frame_id = world_frame_name_;
  oct_msg_blue_cube_.header.stamp = ros::Time::now();
  pub_oct_msg_blue_cube_.publish(oct_msg_blue_cube_);

  oct_msg_actor0_.header.frame_id = world_frame_name_;
  oct_msg_actor0_.header.stamp = ros::Time::now();
  pub_oct_msg_actor0_.publish(oct_msg_actor0_);

  oct_msg_actor1_.header.frame_id = world_frame_name_;
  oct_msg_actor1_.header.stamp = ros::Time::now();
  pub_oct_msg_actor1_.publish(oct_msg_actor1_);

  oct_msg_bin_.header.frame_id = world_frame_name_;
  oct_msg_bin_.header.stamp = ros::Time::now();
  pub_oct_msg_bin_.publish(oct_msg_bin_);

  //std::cout << "[MapUtility::publishObjectOctMsg] START" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishEgoGridPcMsg()
{
  sensor_msgs::PointCloud egrid_pc_msg = egrid_pc_msg_;
  egrid_pc_msg.header.frame_id = egrid_frame_name_;
  //oct_msg.header.seq++;
  egrid_pc_msg.header.stamp = ros::Time::now();
  pub_egrid_pc_msg_.publish(egrid_pc_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishEgoTargetPcMsg()
{
  sensor_msgs::PointCloud egrid_target_pc_msg = egrid_target_pc_msg_;
  egrid_target_pc_msg.header.frame_id = egrid_frame_name_;
  //oct_msg.header.seq++;
  egrid_target_pc_msg.header.stamp = ros::Time::now();
  pub_egrid_target_pc_msg_.publish(egrid_target_pc_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishEgoGridOccPcMsg()
{
  sensor_msgs::PointCloud egrid_occ_pc_msg = egrid_occ_pc_msg_;
  egrid_occ_pc_msg.header.frame_id = world_frame_name_;
  //oct_msg.header.seq++;
  egrid_occ_pc_msg.header.stamp = ros::Time::now();
  pub_egrid_occ_pc_msg_.publish(egrid_occ_pc_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishOccGridMsg()
{
  nav_msgs::OccupancyGrid occ_grid_msg = occ_grid_msg_;
  occ_grid_msg.header.frame_id = egrid_frame_name_;
  //oct_msg.header.seq++;
  occ_grid_msg.header.stamp = ros::Time::now();
  pub_occ_grid_msg_.publish(occ_grid_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishPCMsg()
{
  pc_msg.header.frame_id = world_frame_name_;
  pc_msg.header.seq++;
  //pc_msg.header.stamp = ros::Time(0);
  pc_msg.header.stamp = ros::Time::now();
  pc_msg_pub.publish(pc_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishPC2Msg()
{
  pc2_msg.header.frame_id = world_frame_name_;
  pc2_msg.header.seq++;
  //pc2_msg.header.stamp = ros::Time(0);
  pc2_msg.header.stamp = ros::Time::now();
  pc2_msg_pub.publish(pc2_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishPC2MsgGzScan()
{
  sensor_msgs::PointCloud2 pc2_msg_scan = pc2_msg_scan_;
  pc2_msg_scan.header.frame_id = world_frame_name_;
  pc2_msg_scan.header.seq++;
  pc2_msg_scan.header.stamp = ros::Time::now();
  pub_pc2_msg_scan_.publish(pc2_msg_scan);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/// NUA TODO: MAKE IT GENERALIZABLE!!!
void MapUtility::publishPC2MsgGzPkgIgn(int index_pkg_ign)
{
  //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] START" << std::endl;

  vec_pc2_msg_gz_ign_[index_pkg_ign].header.frame_id = vec_frame_name_ign_[index_pkg_ign];
  //vec_pc2_msg_gz_ign_[index_pkg_ign].header.seq++;
  vec_pc2_msg_gz_ign_[index_pkg_ign].header.stamp = ros::Time::now();

  //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] vec_pc2_msg_gz_ign_ size: " << vec_pc2_msg_gz_ign_[index_pkg_ign].data.size() << std::endl;

  switch (index_pkg_ign)
  {
    case 0:
      pub_pc2_msg_conveyor_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 0 END" << std::endl;
      break;

    case 1:
      pub_pc2_msg_pkg_normal_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 1 END" << std::endl;
      break;

    case 2:
      pub_pc2_msg_pkg_long_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 2 END" << std::endl;
      break;

    case 3:
      pub_pc2_msg_pkg_longwide_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 3 END" << std::endl;
      break;

    case 4:
      pub_pc2_msg_red_cube_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 4 END" << std::endl;
      break;

    case 5:
      pub_pc2_msg_green_cube_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 5 END" << std::endl;
      break;

    case 6:
      pub_pc2_msg_blue_cube_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 6 END" << std::endl;
      break;

    case 7:
      pub_pc2_msg_actor0_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 7 END" << std::endl;
      break;

    /*
    case 8:
      pub_pc2_msg_actor1_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      break;
    */

    case 8:
      pub_pc2_msg_bin_.publish(vec_pc2_msg_gz_ign_[index_pkg_ign]);
      //std::cout << "[MapUtility::publishPC2MsgGzPkgIgn] 8 END" << std::endl;
      break;
    
    default:
      break;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishPC2MsgGzPkgMan(int index_pkg_man)
{
  vec_pc2_msg_gz_man_[index_pkg_man].header.frame_id = vec_frame_name_man_[index_pkg_man];
  vec_pc2_msg_gz_man_[index_pkg_man].header.seq++;
  vec_pc2_msg_gz_man_[index_pkg_man].header.stamp = ros::Time::now();
  
  switch (index_pkg_man)
  {
    case 0:
      pub_pc2_msg_gz_man_pkg_normal_.publish(vec_pc2_msg_gz_man_[index_pkg_man]);
      break;

    case 1:
      pub_pc2_msg_gz_man_pkg_long_.publish(vec_pc2_msg_gz_man_[index_pkg_man]);
      break;

    case 2:
      pub_pc2_msg_gz_man_pkg_longwide_.publish(vec_pc2_msg_gz_man_[index_pkg_man]);
      break;
    
    default:
      break;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishOccDistanceVisu(geometry_msgs::Point p0, geometry_msgs::Point p1)
{
  visualization_msgs::Marker visu_occ_distance;
  visu_occ_distance.ns = "occupancy_distance";
  visu_occ_distance.id = 1;
  visu_occ_distance.type = visualization_msgs::Marker::ARROW;
  visu_occ_distance.action = visualization_msgs::Marker::ADD;
  visu_occ_distance.scale.x = 0.03;
  visu_occ_distance.scale.y = 0.05;
  visu_occ_distance.scale.z = 0.05;
  visu_occ_distance.color.r = 1.0;
  visu_occ_distance.color.g = 0.0;
  visu_occ_distance.color.b = 1.0;
  visu_occ_distance.color.a = 1.0;

  visu_occ_distance.points.clear();
  visu_occ_distance.points.push_back(p0);
  visu_occ_distance.points.push_back(p1);

  visu_occ_distance.header.frame_id = world_frame_name_;
  visu_occ_distance.header.seq++;
  visu_occ_distance.header.stamp = ros::Time::now();

  pub_visu_occ_distance_.publish(visu_occ_distance);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishOccDistanceArrayVisu(vector<geometry_msgs::Point> p0_vec, vector<geometry_msgs::Point> p1_vec)
{
  visualization_msgs::MarkerArray visu_array_occ_distance;

  for (size_t i = 0; i < p0_vec.size(); i++)
  {
    visualization_msgs::Marker occ_distance_visu;
    occ_distance_visu.ns = "occupancy_distance_" + i;
    occ_distance_visu.id = 1;
    occ_distance_visu.header.frame_id = world_frame_name_;
    occ_distance_visu.type = visualization_msgs::Marker::ARROW;
    occ_distance_visu.action = visualization_msgs::Marker::ADD;
    occ_distance_visu.scale.x = 0.03;
    occ_distance_visu.scale.y = 0.05;
    occ_distance_visu.scale.z = 0.05;
    occ_distance_visu.color.r = 1.0;
    occ_distance_visu.color.g = 0.0;
    occ_distance_visu.color.b = 1.0;
    occ_distance_visu.color.a = 1.0;

    occ_distance_visu.points.clear();
    occ_distance_visu.points.push_back(p0_vec[i]);
    occ_distance_visu.points.push_back(p1_vec[i]);

    occ_distance_visu.header.frame_id = world_frame_name_;
    occ_distance_visu.header.seq++;
    occ_distance_visu.header.stamp = ros::Time::now();

    visu_array_occ_distance.markers.push_back(occ_distance_visu);
  }

  pub_visu_array_occ_distance_.publish(visu_array_occ_distance);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishWorldFrame(std::string& world_frame_name, std::string& origin_frame_name)
{
  //std::cout << "[MapUtility::publishGroundTruthWorldFrame] START" << std::endl;

  static tf::TransformBroadcaster br_world;
  tf::Transform tf_gt;

  gazebo_msgs::ModelStates ms = gz_model_states_;

  for (size_t i = 0; i < ms.name.size(); i++)
  {
    if (ms.name[i] == "mobiman")
    {
      tf_gt.setOrigin(tf::Vector3(ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z));
      tf_gt.setRotation(tf::Quaternion(ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w));
      br_world.sendTransform(tf::StampedTransform(tf_gt, ros::Time::now(), world_frame_name_, origin_frame_name));
      break;
    }
  }

  //std::cout << "[MapUtility::publishGroundTruthWorldFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishVirtualFrames(vector<std::string>& virtual_frame_names, std::string& origin_frame_name)
{
  //std::cout << "[MapUtility::publishVirtualFrames] START" << std::endl;

  static tf::TransformBroadcaster br_virtual;
  tf::StampedTransform stf_virtual;

  try
  {
    tflistener -> waitForTransform(world_frame_name_, origin_frame_name, ros::Time(0), ros::Duration(5.0));
    tflistener->lookupTransform(world_frame_name_, origin_frame_name, ros::Time(0), stf_virtual);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::publishVirtualFrames] Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  for (size_t i = 0; i < virtual_frame_names.size(); i++)
  {
    tf::Transform tf_virtual;
    tf_virtual.setOrigin(stf_virtual.getOrigin());
    tf_virtual.setRotation(stf_virtual.getRotation());

    br_virtual.sendTransform(tf::StampedTransform(tf_virtual, ros::Time::now(), world_frame_name_, virtual_frame_names[i]));
  }

  //std::cout << "[MapUtility::publishVirtualFrames] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishMoveitCollisionObjects()
{
  //std::cout << "[MapUtility::publishMoveitCollisionObjects] START" << std::endl;

  std::vector<moveit_msgs::CollisionObject> moveit_collision_objects = moveit_collision_objects_;
  for (size_t i = 0; i < moveit_collision_objects.size(); i++)
  {
    //std::cout << "[MapUtility::publishMoveitCollisionObjects] ADDED " << i << std::endl;
    moveit_collision_objects[i].operation = moveit_collision_objects[i].ADD;
    moveit_collision_objects[i].header.stamp = ros::Time::now();
    moveit_collision_objects[i].header.seq++;
    pub_moveit_collision_object_.publish(moveit_collision_objects[i]);
  }

  //std::cout << "[MapUtility::publishMoveitCollisionObjects] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishDebugArrayVisu()
{
  for (int i = 0; i < debug_array_visu.markers.size(); ++i)
  {
    debug_array_visu.markers[i].header.seq++;
    //debug_array_visu.markers[i].header.stamp = ros::Time(0);
    debug_array_visu.markers[i].header.stamp = ros::Time::now();
  }
  debug_array_visu_pub.publish(debug_array_visu);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::publishDebugVisu()
{
  debug_visu.header.seq++;
  //debug_visu.header.stamp = ros::Time(0);
  debug_visu.header.stamp = ros::Time::now();
  debug_visu_pub.publish(debug_visu);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::printDataSize()
{
  cout << "" << endl;
  cout << "map_utility::printDataSize -> oct: " << oct -> size() << endl;
  cout << "map_utility::printDataSize -> oct_msg: " << oct_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> oct_pc: " << oct_pc.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc_msg: " << measured_pc_msg.points.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc2_msg: " << measured_pc2_msg.data.size() << endl;
  //cout << "map_utility::printDataSize -> measured_laser_msg: " << measured_laser_msg.ranges.size() << endl;
  //cout << "map_utility::printDataSize -> pc_msg: " << pc_msg.points.size() << endl;
  cout << "map_utility::printDataSize -> pc2_msg: " << pc2_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> laser_pc2_msg: " << laser_pc2_msg.data.size() << endl;
  cout << "" << endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::saveMap(std::string filename)
{
  if(filename == "")
  {
    filename = createFileName();
  }

  ofstream map_file;
  std::string tentabot_path = ros::package::getPath("tentabot") + "/";
  map_file.open (tentabot_path + "dataset/map_utility/" + filename + ".csv");

  if( map_file.is_open() )
  {
    map_file << "map_name," + map_name + "\n";
    map_file << "world_frame_name," + world_frame_name_ + "\n";
    map_file << "x_range," + to_string(x_range[0]) + "," + to_string(x_range[1]) + "\n";
    map_file << "y_range," + to_string(y_range[0]) + "," + to_string(y_range[1]) + "\n";
    map_file << "z_range," + to_string(z_range[0]) + "," + to_string(z_range[1]) + "\n";
    map_file << "map_resolution," + to_string(map_resolution_) + "\n";
    map_file << "pc_resolution_scale," + to_string(pc_resolution_scale) + "\n";
    map_file << "max_occupancy_belief_value," + to_string(max_occupancy_belief_value) + "\n";

    map_file << "map,\n";

    for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
    {
      map_file << to_string(it.getCoordinate().x()) + "," + to_string(it.getCoordinate().y()) + "," + to_string(it.getCoordinate().z()) + "\n";
    }
    map_file.close();
  }
  else
  {
    ROS_WARN("MapUtility::saveMap -> Unable to open map file to save.");
  }
}

// NUA TODO: get the benchmark path
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::loadMap(std::string filename)
{
  std::string tentabot_path = ros::package::getPath("tentabot") + "/";
  ifstream map_file(tentabot_path + "dataset/map_utility/" + filename + ".csv");

  if( map_file.is_open() )
  {
    ROS_INFO_STREAM("" << filename << " is loading from the file...");

    oct -> clear();

    std::string line = "";
    bool map_flag = false;
    while( getline(map_file, line) )
    {
      vector<std::string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(","));

      if(vec[0] == "map")
      {
        map_flag = true;
        continue;
      }

      if(map_flag)
      {          
        addStaticObstacle( atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()) );
      }
    }

    // Close the File
    map_file.close();
    cout << filename + " is loaded!" << endl;
  }
  else
  {
    ROS_WARN_STREAM("MapUtility::loadMap -> Unable to open " << filename << " file to load.");
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::mapPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //cout << "MapUtility::mapPoseCallback -> Incoming data..." << endl;
  measured_map_pose = *msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::mapOdometryCallback(const nav_msgs::Odometry& msg)
{
  //cout << "MapUtility::mapOdometryCallback -> Incoming data..." << endl;
  measured_map_pose = msg.pose.pose;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "[MapUtility::pc2Callback] Incoming data..." << endl;
  sensor_pc2_frame_name = msg->header.frame_id;
  //cout << "MapUtility::pc2Callback -> sensor_pc2_frame_name: " << sensor_pc2_frame_name << endl;

  try
  {
    tflistener->waitForTransform(world_frame_name_, msg->header.frame_id, ros::Time(0), ros::Duration(5.0));
    tflistener->lookupTransform(world_frame_name_, msg->header.frame_id, ros::Time(0), measured_transform_sensor_pc2_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::pc2Callback] Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  measured_sensor_pc2_pose.position.x = measured_transform_sensor_pc2_wrt_world.getOrigin().x();
  measured_sensor_pc2_pose.position.y = measured_transform_sensor_pc2_wrt_world.getOrigin().y();
  measured_sensor_pc2_pose.position.z = measured_transform_sensor_pc2_wrt_world.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor_pc2_wrt_world.getRotation(), measured_sensor_pc2_pose.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name_, measured_transform_sensor_pc2_wrt_world, *msg, measured_pc2_msg);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //cout << "[MapUtility::laserCallback] Incoming data..." << endl;
  sensor_laser_frame_name = msg->header.frame_id;
  //cout << "[MapUtility::laserCallback] sensor_laser_frame_name: " << sensor_laser_frame_name << endl;
  measured_laser_msg = *msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::octMsgCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  oct = std::shared_ptr<octomap::ColorOcTree> (dynamic_cast<octomap::ColorOcTree*> (octomap_msgs::msgToMap(*msg)));
  fillOctMsgFromOct();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::gazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  //cout << "[MapUtility::gazeboModelCallback] START" << std::endl;

  gz_model_states_ = *msg;

  //cout << "[MapUtility::gazeboModelCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::updateModelPc2Scan()
{
  //cout << "[MapUtility::updateModelPc2Scan] START" << std::endl;

  gazebo_msgs::ModelStates ms = gz_model_states_;

  bool initFlag = true;
  sensor_msgs::PointCloud2 pc2_msg_scan;
  std::string gz_model_name_tmp, tf_name_tmp;

  vec_transform_ign_.clear();
  vec_transform_man_.clear();
  vec_frame_name_obj_.clear();
  vec_pc2_msg_obj_wrt_world_.clear();

  for (size_t i = 0; i < ms.name.size(); i++)
  {
    //cout << "[MapUtility::updateModelPc2Scan] GAZEBO_MSG_RECEIVED!" << std::endl;
    if (ns_ != "/")
    {
      gz_model_name_tmp = ns_ + "/" + ms.name[i];
    }
    else
    {
      gz_model_name_tmp = ms.name[i];
    }

    for (size_t j = 0; j < vec_frame_name_ign_.size(); j++)
    {
      tf_name_tmp = vec_frame_name_ign_[j];

      //cout << "[MapUtility::updateModelPc2Scan] gz_model_name_tmp: " << gz_model_name_tmp << std::endl;
      //cout << "[MapUtility::updateModelPc2Scan] tf_name_tmp: " << tf_name_tmp << std::endl;

      // NUA TODO: Try to fix this by finding a way to generalize for multiple objects with the same namespace!
      /*
      if ( tf_name_tmp == "actor" && (gz_model_name_tmp == "0" || gz_model_name_tmp == "1") )
      {
        tf_name_tmp += gz_model_name_tmp;
        gz_model_name_tmp = tf_name_tmp;
      }
      */

      if (gz_model_name_tmp == tf_name_tmp)
      {
        //cout << "[MapUtility::updateModelPc2Scan] tf_name_tmp: " << tf_name_tmp << std::endl;
        
        tf::Transform transform_pkg_ign;

        sensor_msgs::PointCloud2 vec_pc2_msg_gz_ign_wrt_world;
        transform_pkg_ign.setIdentity();
        transform_pkg_ign.setOrigin(tf::Vector3(ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z));
        transform_pkg_ign.setRotation(tf::Quaternion(ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w));
      
        pcl_ros::transformPointCloud(world_frame_name_, transform_pkg_ign, vec_pc2_msg_gz_ign_[j], vec_pc2_msg_gz_ign_wrt_world);

        static tf::TransformBroadcaster br_gz_pkg_ign;
        br_gz_pkg_ign.sendTransform(tf::StampedTransform(transform_pkg_ign, ros::Time::now(), world_frame_name_, tf_name_tmp));
        //br_gz_pkg_ign.sendTransform(tf::StampedTransform(transform_pkg_ign, ros::Time::now(), world_frame_name_, ros::this_node::getNamespace() + "/" + tf_name_tmp));

        vec_frame_name_obj_.push_back(tf_name_tmp);
        vec_pc2_msg_obj_wrt_world_.push_back(vec_pc2_msg_gz_ign_wrt_world);

        vec_frame_name_obj_.push_back(tf_name_tmp);
        vec_pc2_msg_obj_wrt_world_.push_back(vec_pc2_msg_gz_ign_wrt_world);

        //cout << "[MapUtility::updateModelPc2Scan] BEFORE publishPC2MsgGzPkgIgn " << tf_name_tmp << std::endl;
        publishPC2MsgGzPkgIgn(j);
        //cout << "[MapUtility::updateModelPc2Scan] AFTER publishPC2MsgGzPkgIgn " << tf_name_tmp << std::endl << std::endl;

        if (initFlag)
        {
          pc2_msg_scan = vec_pc2_msg_gz_ign_wrt_world;
          initFlag = false;
        }
        else
        {
          pcl::concatenatePointCloud(pc2_msg_scan, vec_pc2_msg_gz_ign_wrt_world, pc2_msg_scan);
        }

        vec_transform_ign_.push_back(transform_pkg_ign);
        break;
      }
    }

    for (size_t j = 0; j < vec_frame_name_man_.size(); j++)
    {
      tf_name_tmp = vec_frame_name_man_[j];

      //cout << "[MapUtility::updateModelPc2Scan] gz_model_name_tmp: " << gz_model_name_tmp << std::endl;
      //cout << "[MapUtility::updateModelPc2Scan] tf_name_tmp: " << tf_name_tmp << std::endl;

      if (gz_model_name_tmp == tf_name_tmp)
      {
        tf::Transform transform_pkg_man;

        transform_pkg_man.setIdentity();
        transform_pkg_man.setOrigin(tf::Vector3(ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z));
        transform_pkg_man.setRotation(tf::Quaternion(ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w));
      
        static tf::TransformBroadcaster br_gz_pkg_man;
        br_gz_pkg_man.sendTransform(tf::StampedTransform(transform_pkg_man, ros::Time::now(), world_frame_name_, tf_name_tmp));
        //cout << "[MapUtility::updateModelPc2Scan] world_frame_name_: " << world_frame_name_ << std::endl;
        //cout << "[MapUtility::updateModelPc2Scan] tf_name_tmp: " << tf_name_tmp << std::endl;
        //cout << "[MapUtility::updateModelPc2Scan] ns: " << ros::this_node::getNamespace() << std::endl;
        //br_gz_pkg_man.sendTransform(tf::StampedTransform(transform_pkg_man, ros::Time::now(), world_frame_name_, ros::this_node::getNamespace() + "/" + tf_name_tmp));

        //publishPC2MsgGzPkgMan(j);

        vec_transform_man_.push_back(transform_pkg_man);
        break;
      }
    }
  }

  pc2_msg_scan_ = pc2_msg_scan;

  //cout << "[MapUtility::updateModelPc2Scan] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::update_states()
{
  //map_pose = measured_map_pose;
  sensor_pc2_pose = measured_sensor_pc2_pose;

  try
  {
    // scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment
    tflistener -> waitForTransform(world_frame_name_, map_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name_, map_frame_name, ros::Time(0), transform_map_wrt_world);

    tflistener -> waitForTransform(world_frame_name_, sensor_pc2_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name_, sensor_pc2_frame_name, ros::Time(0), transform_sensor_pc2_wrt_world);

    if (sensor_laser_frame_name != "")
    {
      tflistener -> waitForTransform(world_frame_name_, sensor_laser_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
      tflistener -> lookupTransform(world_frame_name_, sensor_laser_frame_name, ros::Time(0), transform_sensor_laser_wrt_world);
      sensor_laser_projector.transformLaserScanToPointCloud(world_frame_name_, measured_laser_msg, laser_pc2_msg, *tflistener);
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MapUtility::reset_map_utility] Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("world_frame_name_: %s", world_frame_name_.c_str());
    ROS_ERROR("map_frame_name: %s", map_frame_name.c_str());
    ROS_ERROR("sensor_pc2_frame_name: %s", sensor_pc2_frame_name.c_str());
    ROS_ERROR("sensor_laser_frame_name: %s", sensor_laser_frame_name.c_str());
  }

  pc2_msg = measured_pc2_msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::update_map()
{
  // DELETE MAP IN THE SENSOR RANGE
  tf::Vector3 query_wrt_world;
  tf::Vector3 query_wrt_map;
  tf::Vector3 query_wrt_sensor_pc2;
  tf::Vector3 query_wrt_sensor_laser;

  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); it++)
  {
    query_wrt_world.setValue(it.getX(), it.getY(), it.getZ());
    query_wrt_map = transform_map_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_pc2 = transform_sensor_pc2_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_laser = transform_sensor_laser_wrt_world.inverse() * query_wrt_world;

    if (  ( !isInBBx(query_wrt_map, bbx_x_min, bbx_x_max, bbx_y_min, bbx_y_max, bbx_z_min, bbx_z_max) ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorCameraRange(query_wrt_sensor_pc2, sensor_pc2_msg_name != "") ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorLaserRange(query_wrt_sensor_laser, sensor_laser_msg_name != "") ) )
    {
      oct -> deleteNode(it.getKey());
    }
  }

  // CROP RECENT POINTCLOUD2 DATA AND KEEP AS OCTOMAP PC
  octomap::point3d lowerBound(crop_x_min, crop_y_min, crop_z_min);
  octomap::point3d upperBound(crop_x_max, crop_y_max, crop_z_max);
  cropOctPCFromPC2Msg(lowerBound, upperBound, false);
  
  // INSERT RECENT OCTOMAP PC INTO THE MAP
  point3d sensor_pc2_origin(sensor_pc2_pose.position.x, sensor_pc2_pose.position.y, sensor_pc2_pose.position.z);
  insertToOctFromOctPC(sensor_pc2_origin);

  // UPDATE OCTOMAP MSG FROM THE MAP
  fillOctMsgFromOct();

  if (skip_cnt == skip_cnt_reset_sensor_range)
  {
    skip_cnt = 0;
  }
  else
  {
    skip_cnt++;
  }
}

/*
bool MapUtility::reset_map_utility(tentabot::reset_map_utility::Request &req, tentabot::reset_map_utility::Response &res)
{
  cout << "[MapUtility::reset_map_utility] Map is NOT reset! parity: " << req.parity << endl;

  oct -> clear();
  fillOctMsgFromOct();

  cout << "[MapUtility::reset_map_utility] Map is reset!" << endl;

  res.success = true;
  return true;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MapUtility::sensorMsgToOctomapCallback(const ros::TimerEvent& e)
{
  if (!local_map_flag)
  {
    oct -> clear();
    fillOctMsgFromOct();
  }

  update_states();

  update_map();

  publishOctMsg();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getNearestOccupancyDist(double x, double y, double z, bool pub_flag)
{
  //std::cout << "[MapUtility::getNearestOccupancyDist] START" << std::endl;

  double dist_min, dist_tmp;
  std::string name_min;
  int idx_min;
  geometry_msgs::Point collision_p;
  geometry_msgs::Point min_p;
  geometry_msgs::Point query_p;
  query_p.x = x;
  query_p.y = y;
  query_p.z = z;
  
  if (vec_transform_ign_.size() > 0)
  {
    collision_p.x = vec_transform_ign_[0].getOrigin().x();
    collision_p.y = vec_transform_ign_[0].getOrigin().y();
    collision_p.z = vec_transform_ign_[0].getOrigin().z();
    dist_min = find_Euclidean_distance(query_p, collision_p);
    name_min = vec_frame_name_ign_[0];
    idx_min = 0;
  }
  else
  {
    return -1;
  }

  //std::cout << "[MapUtility::getNearestOccupancyDist] vec_transform_ign_.size(): " << vec_transform_ign_.size() << std::endl;
  //std::cout << "[MapUtility::getNearestOccupancyDist] vec_frame_name_ign_.size(): " << vec_frame_name_ign_.size() << std::endl;

  for (size_t i = 0; i < vec_transform_ign_.size(); i++)
  {
    collision_p.x = vec_transform_ign_[i].getOrigin().x();
    collision_p.y = vec_transform_ign_[i].getOrigin().y();
    collision_p.z = vec_transform_ign_[i].getOrigin().z();
    dist_tmp = find_Euclidean_distance(query_p, collision_p);

    if (dist_tmp < dist_min)
    {
      dist_min = dist_tmp;
      name_min = vec_frame_name_ign_[i];
      idx_min = i;
    }
  }
 
  //std::cout << "[MapUtility::getNearestOccupancyDist] dist_min: " << dist_min << std::endl;
  //std::cout << "[MapUtility::getNearestOccupancyDist] name_min: " << name_min << std::endl;
  //std::cout << "[MapUtility::getNearestOccupancyDist] idx_min: " << idx_min << std::endl;

  tf::Transform transform_pc2;
  transform_pc2.setIdentity();
  transform_pc2.setOrigin(vec_transform_ign_[idx_min].getOrigin());
  transform_pc2.setRotation(vec_transform_ign_[idx_min].getRotation());

  sensor_msgs::PointCloud2 pc2_wrt_world;
  pcl_ros::transformPointCloud(world_frame_name_, transform_pc2, vec_pc2_msg_gz_ign_[idx_min], pc2_wrt_world);
  
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_wrt_world, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_wrt_world, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_wrt_world, "z");

  if (iter_x != iter_x.end())
  {
    collision_p.x = *iter_x;
    collision_p.y = *iter_y;
    collision_p.z = *iter_z;
    dist_min = find_Euclidean_distance(query_p, collision_p);
    min_p = collision_p;
  }
  else
  {
    return -1;
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    collision_p.x = *iter_x;
    collision_p.y = *iter_y;
    collision_p.z = *iter_z;
    dist_tmp = find_Euclidean_distance(query_p, collision_p);

    if (dist_tmp < dist_min)
    {
      dist_min = dist_tmp;
      min_p = collision_p;
    }
  }

  if (pub_flag)
  {
    publishOccDistanceVisu(query_p, min_p);
  }

  //std::cout << "[MapUtility::getNearestOccupancyDist] END" << std::endl;

  return dist_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double MapUtility::getNearestOccupancyDist2(double x, double y, double z, bool pub_flag) const
{
  //std::cout << "[MapUtility::getNearestOccupancyDist2] START" << std::endl;

  octomap::ColorOcTree::iterator it = oct -> begin();  

  if (it == NULL)
  {
    return -8;
  }
  else
  {
    double dist_min, dist_tmp;
    geometry_msgs::Point collision_p;
    geometry_msgs::Point min_p;
    geometry_msgs::Point query_p;
    query_p.x = x;
    query_p.y = y;
    query_p.z = z;

    if (it != oct -> end())
    {
      collision_p.x = it.getX();
      collision_p.y = it.getY();
      collision_p.z = it.getZ();
      dist_min = find_Euclidean_distance(query_p, collision_p);
      min_p = collision_p;
    }
    else
    {
      return -1;
    }

    for(it = oct -> begin(); it != oct -> end(); it++)
    {
      collision_p.x = it.getX();
      collision_p.y = it.getY();
      collision_p.z = it.getZ();
      dist_tmp = find_Euclidean_distance(query_p, collision_p);

      if (dist_tmp < dist_min)
      {
        dist_min = dist_tmp;
        min_p = collision_p;
      }
    }

    if (pub_flag)
    {
      //publishOccDistanceVisu(query_p, min_p);
    }

    //std::cout << "[MapUtility::getNearestOccupancyDist2] END" << std::endl;

    return dist_min;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MapUtility::getNearestOccupancyDistSrv(mobiman_simulation::getNearestOccDist::Request &req, 
                                            mobiman_simulation::getNearestOccDist::Response &res)
{
  //std::cout << "[MapUtility::getNearestOccupancyDistSrv] START" << std::endl;

  res.distance = getNearestOccupancyDist2(req.x, req.y, req.z);

  if (res.distance < 0)
  {
    //std::cout << "[MapUtility::getNearestOccupancyDistSrv] FALSE END" << std::endl << std::endl;
    return false;
  }
  else
  {
    //std::cout << "[MapUtility::getNearestOccupancyDistSrv] TRUE END" << std::endl << std::endl;
    return true;
  }

  return true;
}