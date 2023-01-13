// LAST UPDATE: 2023.01.13
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
// 
// TODO:
//

// --CUSTOM LIBRARIES--
#include "scan_utility.h"

ScanUtility::ScanUtility(NodeHandle& nh,
                         string world_frame_name,
                         vector<string> pc2_msg_name_vec_,
                         double bbx_x_min,
                         double bbx_x_max,
                         double bbx_y_min,
                         double bbx_y_max,
                         double bbx_z_min,
                         double bbx_z_max,
                         double oct_resolution)
{
  tflistener_ = new tf::TransformListener;

  world_frame_name_ = world_frame_name;

  for (size_t i = 0; i < pc2_msg_name_vec_.size(); i++)
  {
    switch (i)
    {
      case 0:
        pc2_msg_name_sensor1_ = pc2_msg_name_vec_[i];
        break;
      case 1:
        pc2_msg_name_sensor2_ = pc2_msg_name_vec_[i];
        break;
      case 2:
        pc2_msg_name_sensor3_ = pc2_msg_name_vec_[i];
        break;
      case 3:
        pc2_msg_name_sensor4_ = pc2_msg_name_vec_[i];
        break;
      default:
        pc2_msg_name_sensor1_ = pc2_msg_name_vec_[i];
        break;
    }
  }

  bbx_x_min_ = bbx_x_min;
  bbx_x_max_ = bbx_x_max;
  bbx_y_min_ = bbx_y_min;
  bbx_y_max_ = bbx_y_max;
  bbx_z_min_ = bbx_z_min;
  bbx_z_max_ = bbx_z_max;

  oct_resolution_ = oct_resolution;
  oct_ = std::make_shared<octomap::ColorOcTree>(oct_resolution_);

  point3d bbx_mini(bbx_x_min, bbx_y_min, bbx_z_min);
  point3d bbx_maxi(bbx_x_max, bbx_y_max, bbx_z_max);
  oct_->setBBXMin(bbx_mini);
  oct_->setBBXMax(bbx_maxi);

  // Subscribers
  sub_pc2_sensor1_ = nh.subscribe(pc2_msg_name_sensor1_, 1000, &ScanUtility::pc2CallbackSensor1, this);
  sub_pc2_sensor2_ = nh.subscribe(pc2_msg_name_sensor2_, 1000, &ScanUtility::pc2CallbackSensor2, this);
  sub_pc2_sensor3_ = nh.subscribe(pc2_msg_name_sensor3_, 1000, &ScanUtility::pc2CallbackSensor3, this);
  sub_pc2_sensor4_ = nh.subscribe(pc2_msg_name_sensor4_, 1000, &ScanUtility::pc2CallbackSensor4, this);

  // Publishers
  pub_oct_msg_ = nh.advertise<octomap_msgs::Octomap>("scan_octomap", 100);
  pub_pc2_msg_out_ = nh.advertise<sensor_msgs::PointCloud2>("scan_pc2_out", 10);
  pub_debug_array_visu_ = nh.advertise<visualization_msgs::MarkerArray>("scan_debug_array", 10);
  pub_debug_visu_ = nh.advertise<visualization_msgs::Marker>("scan_debug", 10);
}

ScanUtility::~ScanUtility()
{
  //std::cout << "[ScanUtility::~ScanUtility] Calling Destructor for ScanUtility..." << std::endl;
  delete[] tflistener_;
}

ScanUtility::ScanUtility(const ScanUtility& su)
{
  tflistener_ = su.tflistener_;
  
  world_frame_name_ = su.world_frame_name_;

  pc2_msg_name_sensor1_ =  su.pc2_msg_name_sensor1_;
  pc2_msg_name_sensor2_ =  su.pc2_msg_name_sensor2_;
  pc2_msg_name_sensor3_ =  su.pc2_msg_name_sensor3_;
  pc2_msg_name_sensor4_ =  su.pc2_msg_name_sensor4_;
  
  pc2_frame_name_sensor1_ = su.pc2_frame_name_sensor1_;
  pc2_frame_name_sensor2_ = su.pc2_frame_name_sensor2_;
  pc2_frame_name_sensor3_ = su.pc2_frame_name_sensor3_;
  pc2_frame_name_sensor4_ = su.pc2_frame_name_sensor4_;

  bbx_x_min_ = su.bbx_x_min_;
  bbx_x_max_ = su.bbx_x_max_;
  bbx_y_min_ = su.bbx_y_min_;
  bbx_y_max_ = su.bbx_y_max_;
  bbx_z_min_ = su.bbx_z_min_;
  bbx_z_max_ = su.bbx_z_max_;
  
  pc2_msg_sensor1_ = su.pc2_msg_sensor1_;
  pc2_msg_sensor2_ = su.pc2_msg_sensor2_;
  pc2_msg_sensor3_ = su.pc2_msg_sensor3_;
  pc2_msg_sensor4_ = su.pc2_msg_sensor4_;

  debug_array_visu_ = su.debug_array_visu_;
  debug_visu_ = su.debug_visu_;
    
  sub_pc2_sensor1_ = su.sub_pc2_sensor1_;
  sub_pc2_sensor2_ = su.sub_pc2_sensor2_;
  sub_pc2_sensor3_ = su.sub_pc2_sensor3_;
  sub_pc2_sensor4_ = su.sub_pc2_sensor4_;

  pub_debug_array_visu_ = su.pub_debug_array_visu_;
  pub_debug_visu_ = su.pub_debug_visu_;
}

ScanUtility& ScanUtility::operator = (const ScanUtility& su) 
{
  tflistener_ = su.tflistener_;
  
  world_frame_name_ = su.world_frame_name_;

  pc2_msg_name_sensor1_ =  su.pc2_msg_name_sensor1_;
  pc2_msg_name_sensor2_ =  su.pc2_msg_name_sensor2_;
  pc2_msg_name_sensor3_ =  su.pc2_msg_name_sensor3_;
  pc2_msg_name_sensor4_ =  su.pc2_msg_name_sensor4_;
  
  pc2_frame_name_sensor1_ = su.pc2_frame_name_sensor1_;
  pc2_frame_name_sensor2_ = su.pc2_frame_name_sensor2_;
  pc2_frame_name_sensor3_ = su.pc2_frame_name_sensor3_;
  pc2_frame_name_sensor4_ = su.pc2_frame_name_sensor4_;

  bbx_x_min_ = su.bbx_x_min_;
  bbx_x_max_ = su.bbx_x_max_;
  bbx_y_min_ = su.bbx_y_min_;
  bbx_y_max_ = su.bbx_y_max_;
  bbx_z_min_ = su.bbx_z_min_;
  bbx_z_max_ = su.bbx_z_max_;

  pc2_msg_sensor1_ = su.pc2_msg_sensor1_;
  pc2_msg_sensor2_ = su.pc2_msg_sensor2_;
  pc2_msg_sensor3_ = su.pc2_msg_sensor3_;
  pc2_msg_sensor4_ = su.pc2_msg_sensor4_;

  debug_array_visu_ = su.debug_array_visu_;
  debug_visu_ = su.debug_visu_;
    
  sub_pc2_sensor1_ = su.sub_pc2_sensor1_;
  sub_pc2_sensor2_ = su.sub_pc2_sensor2_;
  sub_pc2_sensor3_ = su.sub_pc2_sensor3_;
  sub_pc2_sensor4_ = su.sub_pc2_sensor4_;

  pub_debug_array_visu_ = su.pub_debug_array_visu_;
  pub_debug_visu_ = su.pub_debug_visu_;

  return *this;
}

void ScanUtility::pc2CallbackSensor1(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "[ScanUtility::pc2CallbackSensor1] Incoming data..." << endl;

  pc2_frame_name_sensor1_ = msg -> header.frame_id;
  try
  {
    //tflistener_ -> waitForTransform(world_frame_name_, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener_ -> lookupTransform(world_frame_name_, msg -> header.frame_id, ros::Time(0), measured_transform_sensor1_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("ScanUtility::pc2CallbackSensor1 -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  sensor1_pose_.position.x = measured_transform_sensor1_wrt_world_.getOrigin().x();
  sensor1_pose_.position.y = measured_transform_sensor1_wrt_world_.getOrigin().y();
  sensor1_pose_.position.z = measured_transform_sensor1_wrt_world_.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor1_wrt_world_.getRotation(), sensor1_pose_.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name_, measured_transform_sensor1_wrt_world_, *msg, measured_pc2_msg_sensor1_);
}

void ScanUtility::pc2CallbackSensor2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "[ScanUtility::pc2CallbackSensor2] Incoming data..." << endl;
  
  pc2_frame_name_sensor2_ = msg -> header.frame_id;
  try
  {
    //tflistener_ -> waitForTransform(world_frame_name_, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener_ -> lookupTransform(world_frame_name_, msg -> header.frame_id, ros::Time(0), measured_transform_sensor2_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("ScanUtility::pc2CallbackSensor2 -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  sensor2_pose_.position.x = measured_transform_sensor2_wrt_world_.getOrigin().x();
  sensor2_pose_.position.y = measured_transform_sensor2_wrt_world_.getOrigin().y();
  sensor2_pose_.position.z = measured_transform_sensor2_wrt_world_.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor2_wrt_world_.getRotation(), sensor2_pose_.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name_, measured_transform_sensor2_wrt_world_, *msg, measured_pc2_msg_sensor2_);
}

void ScanUtility::pc2CallbackSensor3(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "[ScanUtility::pc2CallbackSensor3] Incoming data..." << endl;
  
  pc2_frame_name_sensor3_ = msg -> header.frame_id;
  try
  {
    //tflistener_ -> waitForTransform(world_frame_name_, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener_ -> lookupTransform(world_frame_name_, msg -> header.frame_id, ros::Time(0), measured_transform_sensor3_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("ScanUtility::pc2CallbackSensor3 -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  sensor3_pose_.position.x = measured_transform_sensor3_wrt_world_.getOrigin().x();
  sensor3_pose_.position.y = measured_transform_sensor3_wrt_world_.getOrigin().y();
  sensor3_pose_.position.z = measured_transform_sensor3_wrt_world_.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor3_wrt_world_.getRotation(), sensor3_pose_.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name_, measured_transform_sensor3_wrt_world_, *msg, measured_pc2_msg_sensor3_);
}

void ScanUtility::pc2CallbackSensor4(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "[ScanUtility::pc2CallbackSensor4] Incoming data..." << endl;
  
  pc2_frame_name_sensor4_ = msg -> header.frame_id;
  try
  {
    //tflistener_ -> waitForTransform(world_frame_name_, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener_ -> lookupTransform(world_frame_name_, msg -> header.frame_id, ros::Time(0), measured_transform_sensor4_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("ScanUtility::pc2CallbackSensor4 -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  sensor4_pose_.position.x = measured_transform_sensor4_wrt_world_.getOrigin().x();
  sensor4_pose_.position.y = measured_transform_sensor4_wrt_world_.getOrigin().y();
  sensor4_pose_.position.z = measured_transform_sensor4_wrt_world_.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor4_wrt_world_.getRotation(), sensor4_pose_.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name_, measured_transform_sensor4_wrt_world_, *msg, measured_pc2_msg_sensor4_);
}

void ScanUtility::octomapToPclPointcloud()
{
  pcl_cloud_.clear();
  for (octomap::ColorOcTree::iterator it = oct_ -> begin(); it != oct_ -> end(); ++it)
  {
    if (oct_->isNodeOccupied(*it))
    {
      if (!std::isnan(it.getX()) && !std::isnan(it.getY()) && !std::isnan(it.getZ()))
      {
        pcl_cloud_.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
      }
    }
  }

  /*
  for (octomap::ColorOcTree::leaf_iterator it = oct_ -> begin_leafs(); it != oct_ -> end_leafs(); ++it)
  {
    if (oct_->isNodeOccupied(*it))
    {
      if (!std::isnan(it.getX()) && !std::isnan(it.getY()) && !std::isnan(it.getZ()))
      {
        pcl_cloud_.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
      }
    }
  }
  */
}

void ScanUtility::fillOctMsgFromOct()
{
  oct_msg_.data.clear();
  oct_msg_.header.frame_id = world_frame_name_;
  oct_msg_.binary = false;
  oct_msg_.id = "object_scan";
  oct_msg_.resolution = oct_resolution_;
  octomap_msgs::fullMapToMsg(*oct_, oct_msg_);
}

void ScanUtility::publishOctMsg()
{
  oct_msg_.header.frame_id = world_frame_name_;
  //oct_msg.header.seq++;
  //oct_msg.header.stamp = ros::Time(0);
  oct_msg_.header.stamp = ros::Time::now();
  pub_oct_msg_.publish(oct_msg_);
}

void ScanUtility::publishPC2Msg()
{
  pc2_msg_out_.header.frame_id = world_frame_name_;
  pc2_msg_out_.header.seq++;
  //pc2_msg.header.stamp = ros::Time(0);
  pc2_msg_out_.header.stamp = ros::Time::now();
  pub_pc2_msg_out_.publish(pc2_msg_out_);
}

void ScanUtility::mainCallback(const ros::TimerEvent& e)
{
  //cout << "[ScanUtility::mainCallback] START" << endl;

  pc2_msg_sensor1_ = measured_pc2_msg_sensor1_;
  pc2_msg_sensor2_ = measured_pc2_msg_sensor2_;
  pc2_msg_sensor3_ = measured_pc2_msg_sensor3_;
  pc2_msg_sensor4_ = measured_pc2_msg_sensor4_;

  pointCloud2ToOctomap(pc2_msg_sensor1_, oct_pc_sensor1_);
  pointCloud2ToOctomap(pc2_msg_sensor2_, oct_pc_sensor2_);
  pointCloud2ToOctomap(pc2_msg_sensor3_, oct_pc_sensor3_);
  pointCloud2ToOctomap(pc2_msg_sensor4_, oct_pc_sensor4_);

  point3d sensor1_origin(sensor1_pose_.position.x, sensor1_pose_.position.y, sensor1_pose_.position.z);
  point3d sensor2_origin(sensor2_pose_.position.x, sensor2_pose_.position.y, sensor2_pose_.position.z);
  point3d sensor3_origin(sensor3_pose_.position.x, sensor3_pose_.position.y, sensor3_pose_.position.z);
  point3d sensor4_origin(sensor4_pose_.position.x, sensor4_pose_.position.y, sensor4_pose_.position.z);

  oct_ -> insertPointCloud(oct_pc_sensor1_, sensor1_origin, 10, false, true);
  oct_ -> insertPointCloud(oct_pc_sensor2_, sensor2_origin, 10, false, true);
  oct_ -> insertPointCloud(oct_pc_sensor3_, sensor3_origin, 10, false, true);
  oct_ -> insertPointCloud(oct_pc_sensor4_, sensor4_origin, 10, false, true);

  octomapToPclPointcloud();

  //cout << "[ScanUtility::mainCallback] pcl_cloud_ size: " << pcl_cloud_.size() << endl;

  pcl::toROSMsg(pcl_cloud_, pc2_msg_out_);

  fillOctMsgFromOct();
  
  publishOctMsg();
  publishPC2Msg();

  //cout << "[ScanUtility::mainCallback] END" << endl;
  //cout << "" << endl;
}