#ifndef SCAN_UTILITY_H
#define SCAN_UTILITY_H

// LAST UPDATE: 2023.05.18
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --EXTERNAL LIBRARIES--
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <octomap_ros/conversions.h>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// --CUSTOM LIBRARIES--
#include <mobiman_simulation/common_utility.h>

// --NAMESPACES--
using namespace std;
using namespace ros;
using namespace octomap;
using json = nlohmann::json;

// DESCRIPTION: TODO...
class ScanUtility
{
  public:

    // DESCRIPTION: TODO...
    ScanUtility(NodeHandle& nh);

    // DESCRIPTION: TODO...
    ScanUtility(NodeHandle& nh,
                string data_path);

    // DESCRIPTION: TODO...
    ScanUtility(NodeHandle& nh,
                string obj_name,
                string data_dir,
                string world_frame_name,
                vector<string> pc2_msg_name_vec_,
                double scan_bbx_x_min,
                double scan_bbx_x_max,
                double scan_bbx_y_min,
                double scan_bbx_y_max,
                double scan_bbx_z_min,
                double scan_bbx_z_max,
                double oct_resolution);

    // DESCRIPTION: TODO...
  	ScanUtility(const ScanUtility& su);

    // DESCRIPTION: TODO...
  	~ScanUtility();

    // DESCRIPTION: TODO...
  	ScanUtility& operator=(const ScanUtility& su);

    // DESCRIPTION: TODO...
    sensor_msgs::PointCloud2 getPC2MsgScan();

    // DESCRIPTION: TODO...
    geometry_msgs::Point getObjBbxMin();

    // DESCRIPTION: TODO...
    geometry_msgs::Point getObjBbxMax();

    // DESCRIPTION: TODO...
    geometry_msgs::Point getObjDim();

    // DESCRIPTION: TODO...
    void getPointcloud2wrtWorld(const sensor_msgs::PointCloud2& msg_in, 
                                sensor_msgs::PointCloud2& msg_out);

    // DESCRIPTION: TODO...
    void getSensorPoseAndTransformPointcloud2(const sensor_msgs::PointCloud2& msg_in, 
                                              geometry_msgs::Pose& sensor_pose, 
                                              sensor_msgs::PointCloud2& msg_out);

    // DESCRIPTION: TODO...
    void waitAndCheckForPointCloud2Message(string msg_name, double duration_time, sensor_msgs::PointCloud2& pc_msg);

    /*
    // DESCRIPTION: TODO...
    void pc2CallbackSensor1(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor2(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor3(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void pc2CallbackSensor4(const sensor_msgs::PointCloud2::ConstPtr& msg);
    */

    // DESCRIPTION: TODO...
    void getScanPointcloud2(string data_path, sensor_msgs::PointCloud2& pc2_msg);

    // DESCRIPTION: TODO...
    void octomapToPclPointcloud();

    // DESCRIPTION: TODO...
    void PclPointcloudToVec(vector<double>& pcl_pc_scan_x, 
                            vector<double>& pcl_pc_scan_y, 
                            vector<double>& pcl_pc_scan_z);

    // DESCRIPTION: TODO...
    void vecToPclPointcloud(vector<double>& pcl_pc_scan_x, 
                            vector<double>& pcl_pc_scan_y, 
                            vector<double>& pcl_pc_scan_z);

    // DESCRIPTION: TODO...
    void updateObjBbxDim(vector<double>& pcl_pc_scan_x, 
                         vector<double>& pcl_pc_scan_y, 
                         vector<double>& pcl_pc_scan_z);

    // DESCRIPTION: TODO...
    void fillOctMsgFromOct();

    // DESCRIPTION: TODO...
    void publishOctMsg();

    // DESCRIPTION: TODO...
    void publishPC2Msg();

    // DESCRIPTION: TODO...
    //void mainCallback(const ros::TimerEvent& e);

    // DESCRIPTION: TODO...
    void scanner();

    // DESCRIPTION: TODO...
    void writePointcloud2Data();

    // DESCRIPTION: TODO...
    void readPointcloud2Data(string& data_path);

    // DESCRIPTION: TODO...
    void readObjBbxDim(string& data_path);

  private:

    tf::TransformListener* tflistener_;

    string obj_name_;
    string pkg_dir_;
    string data_dir_;
    string data_path_;
    string world_frame_name_;

    string pc2_msg_name_sensor1_;
    string pc2_msg_name_sensor2_;
    string pc2_msg_name_sensor3_;
    string pc2_msg_name_sensor4_;

    double scan_bbx_x_min_;
    double scan_bbx_x_max_;
    double scan_bbx_y_min_;
    double scan_bbx_y_max_;
    double scan_bbx_z_min_;
    double scan_bbx_z_max_;

    sensor_msgs::PointCloud2 measured_pc2_msg_sensor1_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor2_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor3_;
    sensor_msgs::PointCloud2 measured_pc2_msg_sensor4_;

    geometry_msgs::Pose pose_sensor1_;
    geometry_msgs::Pose pose_sensor2_;
    geometry_msgs::Pose pose_sensor3_;
    geometry_msgs::Pose pose_sensor4_;

    sensor_msgs::PointCloud2 pc2_msg_sensor1_;
    sensor_msgs::PointCloud2 pc2_msg_sensor2_;
    sensor_msgs::PointCloud2 pc2_msg_sensor3_;
    sensor_msgs::PointCloud2 pc2_msg_sensor4_;

    octomap::Pointcloud oct_pc_sensor1_;
    octomap::Pointcloud oct_pc_sensor2_;
    octomap::Pointcloud oct_pc_sensor3_;
    octomap::Pointcloud oct_pc_sensor4_;

    double oct_resolution_;    
    shared_ptr<octomap::ColorOcTree> oct_;
    octomap_msgs::Octomap oct_msg_;

    pcl::PointCloud<pcl::PointXYZ> pcl_pc_scan_;
    sensor_msgs::PointCloud2 pc2_msg_scan_;

    geometry_msgs::Point obj_bbx_min_;
    geometry_msgs::Point obj_bbx_max_;
    geometry_msgs::Point obj_dim_;

    //visualization_msgs::MarkerArray debug_array_visu_;
    //visualization_msgs::Marker debug_visu_;
    
    ros::Subscriber sub_pc2_sensor1_;
    ros::Subscriber sub_pc2_sensor2_;
    ros::Subscriber sub_pc2_sensor3_;
    ros::Subscriber sub_pc2_sensor4_;

    ros::Publisher pub_pc2_msg_scan_;
    ros::Publisher pub_oct_msg_;
    ros::Publisher pub_debug_array_visu_;
    ros::Publisher pub_debug_visu_;

};//END of class ScanUtility

#endif
