#ifndef MAP_UTILITY_H
#define MAP_UTILITY_H

// LAST UPDATE: 2023.01.12
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --EXTERNAL LIBRARIES--
#include <boost/algorithm/string.hpp>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointField.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <fcl/fcl.h>
#include <fcl/geometry/shape/convex.h>
#include <ros/package.h>
#include <octomap_ros/conversions.h>

// --CUSTOM LIBRARIES--
//#include "common_utility.h"

// --NAMESPACES--
using namespace std;
using namespace ros;
using namespace octomap;

// DESCRIPTION: TODO...
class ScanUtility
{
  public:

    // DESCRIPTION: TODO...
    ScanUtility(NodeHandle& nh,
                string world_frame_name,
                vector<string> pc2_msg_name_vec_,
                double bbx_x_min,
                double bbx_x_max,
                double bbx_y_min,
                double bbx_y_max,
                double bbx_z_min,
                double bbx_z_max,
                double oct_resolution);

    // DESCRIPTION: TODO...
  	ScanUtility(const ScanUtility& su);

    // DESCRIPTION: TODO...
  	~ScanUtility();

    // DESCRIPTION: TODO...
  	ScanUtility& operator = (const ScanUtility& su);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor1(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor2(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor3(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor4(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void octomapToPclPointcloud();

    // DESCRIPTION: TODO...
    void fillOctMsgFromOct();

    // DESCRIPTION: TODO...
    void publishOctMsg();

    // DESCRIPTION: TODO...
    void publishPC2Msg();

    // DESCRIPTION: TODO...
    void mainCallback(const ros::TimerEvent& e);

  private:

    tf::TransformListener* tflistener_;

    string world_frame_name_;

    string pc2_msg_name_sensor1_;
    string pc2_msg_name_sensor2_;
    string pc2_msg_name_sensor3_;
    string pc2_msg_name_sensor4_;

    string pc2_frame_name_sensor1_;
    string pc2_frame_name_sensor2_;
    string pc2_frame_name_sensor3_;
    string pc2_frame_name_sensor4_;

    double bbx_x_min_;
    double bbx_x_max_;
    double bbx_y_min_;
    double bbx_y_max_;
    double bbx_z_min_;
    double bbx_z_max_;

    sensor_msgs::PointCloud2 pc2_msg_sensor1_;
    sensor_msgs::PointCloud2 pc2_msg_sensor2_;
    sensor_msgs::PointCloud2 pc2_msg_sensor3_;
    sensor_msgs::PointCloud2 pc2_msg_sensor4_;

    visualization_msgs::MarkerArray debug_array_visu_;
    visualization_msgs::Marker debug_visu_;
    
    ros::Subscriber sub_pc2_sensor1_;
    ros::Subscriber sub_pc2_sensor2_;
    ros::Subscriber sub_pc2_sensor3_;
    ros::Subscriber sub_pc2_sensor4_;

    ros::Publisher pub_debug_array_visu_;
    ros::Publisher pub_debug_visu_;

    // NUA TODO: Add in copy Constructors
    tf::StampedTransform measured_transform_sensor1_wrt_world_;
    tf::StampedTransform measured_transform_sensor2_wrt_world_;
    tf::StampedTransform measured_transform_sensor3_wrt_world_;
    tf::StampedTransform measured_transform_sensor4_wrt_world_;

    sensor_msgs::PointCloud2 measured_pc2_msg_sensor1_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor2_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor3_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor4_;

    octomap::Pointcloud oct_pc_sensor1_;
    octomap::Pointcloud oct_pc_sensor2_;
    octomap::Pointcloud oct_pc_sensor3_;
    octomap::Pointcloud oct_pc_sensor4_;

    double oct_resolution_;
    
    shared_ptr<octomap::ColorOcTree> oct_;
    octomap_msgs::Octomap oct_msg_;
    point3d_list oct_point3d_list_;
    
    geometry_msgs::Pose sensor1_pose_;
    geometry_msgs::Pose sensor2_pose_;
    geometry_msgs::Pose sensor3_pose_;
    geometry_msgs::Pose sensor4_pose_;

    sensor_msgs::PointCloud2 pc2_msg_out_;
    pcl::PCLPointCloud2 point_cloud2;

    ros::Publisher pub_pc2_msg_out_;
    ros::Publisher pub_oct_msg_;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_;
    pcl::PCLPointCloud2 pcl_pc2_;

};//END of class ScanUtility

#endif