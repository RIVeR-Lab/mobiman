#ifndef TRAJECTORY_SAMPLING_UTILITY_H
#define TRAJECTORY_SAMPLING_UTILITY_H

// LAST UPDATE: 2024.03.10
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] N. Ü. Akmandor and T. Padir, "A 3D Reactive Navigation Algorithm 
//     for Mobile Robots by Using Tentacle-Based Sampling," 2020 Fourth 
//     IEEE International Conference on Robotic Computing (IRC), Taichung, 
//     Taiwan, 2020, pp. 9-16, doi: 10.1109/IRC.2020.00009.
//
// TODO:
// - Add copy constructors.

// --OUTSOURCE LIBRARIES--
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>

// --CUSTOM LIBRARIES--
#include "mobiman_simulation/common_utility.h"

// --NAMESPACES--
using namespace std;
using namespace ros;

// --GLOBAL VARIABLES--
#define PI 3.141592653589793
#define INF std::numeric_limits<double>::infinity()
#define INFINT std::numeric_limits<int>::max()
#define FMAX std::numeric_limits<float>::max()
#define FMIN std::numeric_limits<float>::min()

// DESCRIPTION: TODO...
class TrajectorySamplingUtility
{
  public:

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh, string tframe="");

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh, vector<vector<geometry_msgs::Point>> tdata, 
                              string tframe,
                              string tsdataset_path);

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh,
                              string tframe,
                              double tlen,
                              int tsamp_cnt,
                              double tyaw,
                              double tpitch,
                              int tyaw_cnt, 
                              int tpitch_cnt,
                              string tsdataset_path,
                              string tyaw_samp_type="",
                              string tpitch_samp_type="");

    // DESCRIPTION: TODO...Constructor
    TrajectorySamplingUtility(NodeHandle& nh,
                              string tframe,
                              double tlen,
                              int tsamp_cnt,
                              int lat_velo_cnt,
                              int ang_velo_cnt,
                              double max_lat_velo,
                              double max_yaw_velo,
                              double dt,
                              string tsdataset_path,
                              string tyaw_samp_type="",
                              string tpitch_samp_type="");
    
    // DESCRIPTION: Destructor
    ~TrajectorySamplingUtility();

    // DESCRIPTION: TODO...
    vector<vector<geometry_msgs::Point>> get_trajectory_data();

    // DESCRIPTION: TODO...
    vector<vector<double>> get_velocity_control_data();

    // DESCRIPTION: TODO...
    vector<string> get_trajectory_lrm_data();

    // DESCRIPTION: TODO...
    string get_trajectory_sampling_dataset_path();

    // DESCRIPTION: TODO...
    string get_trajectory_data_path();

    // DESCRIPTION: TODO...
    string get_trajectory_frame();

    // DESCRIPTION: TODO...
    string get_trajectory_generation_type();

    // DESCRIPTION: TODO...
    double get_trajectory_time();

    // DESCRIPTION: TODO...
    double get_trajectory_length();

    // DESCRIPTION: TODO...
    double get_trajectory_yaw();

    // DESCRIPTION: TODO...
    double get_trajectory_pitch();

    // DESCRIPTION: TODO...
    int get_trajectory_sampling_count();

    // DESCRIPTION: TODO...
    int get_lateral_velocity_sampling_count();

    // DESCRIPTION: TODO...
    int get_angular_velocity_sampling_count();

    // DESCRIPTION: TODO...
    int get_trajectory_yaw_sampling_count();

    // DESCRIPTION: TODO...
    int get_trajectory_pitch_sampling_count();

    // DESCRIPTION: TODO...
    string get_trajectory_yaw_sampling_type();

    // DESCRIPTION: TODO...
    string get_trajectory_pitch_sampling_type();

    // DESCRIPTION: TODO...
    ros::Publisher get_trajectory_visu_pub();

    // DESCRIPTION: TODO...
    ros::Publisher get_trajectory_sampling_visu_pub();

    // DESCRIPTION: TODO...
    ros::Publisher get_trajectory_sampling_arrow_visu_pub();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray get_trajectory_visu();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray get_trajectory_sampling_visu();

    // DESCRIPTION: TODO...
    visualization_msgs::MarkerArray get_trajectory_sampling_arrow_visu();

    // DESCRIPTION: TODO...
    void set_workspace_name(std::string ws_name);

    // DESCRIPTION: TODO...
    void set_trajectory_name(std::string trajectory_name);

    // DESCRIPTION: TODO...
    void set_trajectory_data(vector<vector<geometry_msgs::Point>> new_trajectory_data);

    // DESCRIPTION: TODO...
    void set_velocity_control_data(vector<vector<double>> new_velocity_control_data);

    // DESCRIPTION: TODO...
    void set_trajectory_lrm_data(vector<string> new_trajectory_lrm_data);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_dataset_path(string new_trajectory_sampling_dataset_path);

    // DESCRIPTION: TODO...
    void set_trajectory_data_path(string new_trajectory_data_path);

    // DESCRIPTION: TODO...
    void set_trajectory_frame(string new_trajectory_frame);

    // DESCRIPTION: TODO...
    void set_trajectory_generation_type(string new_trajectory_generation_type);

    // DESCRIPTION: TODO...
    void set_trajectory_time(double new_trajectory_time);

    // DESCRIPTION: TODO...
    void set_trajectory_length(double new_trajectory_length);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw(double new_trajectory_yaw);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch(double new_trajectory_pitch);

    // DESCRIPTION: TODO...
    void set_sampling_x_min(double sampling_x_min);
    
    // DESCRIPTION: TODO...
    void set_sampling_x_max(double sampling_x_max);

    // DESCRIPTION: TODO...
    void set_sampling_x_cnt(int sampling_x_cnt);
    
    // DESCRIPTION: TODO...
    void set_sampling_y_min(double sampling_y_min);
    
    // DESCRIPTION: TODO...
    void set_sampling_y_max(double sampling_y_max);

    // DESCRIPTION: TODO...
    void set_sampling_y_cnt(int sampling_y_cnt);

    // DESCRIPTION: TODO...
    void set_sampling_z_min(double sampling_z_min);

    // DESCRIPTION: TODO...
    void set_sampling_z_max(double sampling_z_max);

    // DESCRIPTION: TODO...
    void set_sampling_z_cnt(int sampling_z_cnt);

    // DESCRIPTION: TODO...
    void set_sampling_roll_min(double sampling_roll_min);

    // DESCRIPTION: TODO...
    void set_sampling_roll_max(double sampling_roll_max);

    // DESCRIPTION: TODO...
    void set_sampling_roll_cnt(int sampling_roll_cnt);

    // DESCRIPTION: TODO...
    void set_sampling_pitch_min(double sampling_pitch_min);

    // DESCRIPTION: TODO...
    void set_sampling_pitch_max(double sampling_pitch_max);

    // DESCRIPTION: TODO...
    void set_sampling_pitch_cnt(int sampling_pitch_cnt);

    // DESCRIPTION: TODO...
    void set_sampling_yaw_min(double sampling_yaw_min);

    // DESCRIPTION: TODO...
    void set_sampling_yaw_max(double sampling_yaw_max);

    // DESCRIPTION: TODO...
    void set_sampling_yaw_cnt(int sampling_yaw_cnt);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_count(int new_trajectory_sampling_count);

    // DESCRIPTION: TODO...
    void set_lateral_velocity_sampling_count(int new_lateral_velocity_sampling_count);

    // DESCRIPTION: TODO...
    void set_angular_velocity_sampling_count(int new_angular_velocity_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw_sampling_count(int new_trajectory_yaw_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch_sampling_count(int new_trajectory_pitch_sampling_count);

    // DESCRIPTION: TODO...
    void set_trajectory_yaw_sampling_type(string new_trajectory_yaw_sampling_type);

    // DESCRIPTION: TODO...
    void set_trajectory_pitch_sampling_type(string new_trajectory_pitch_sampling_type);

    // DESCRIPTION: TODO...
    void set_trajectory_visu_pub(ros::Publisher new_trajectory_visu_pub);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_visu_pub(ros::Publisher new_trajectory_sampling_visu_pub);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_arrow_visu_pub(ros::Publisher new_trajectory_sampling_arrow_visu_pub);

    // DESCRIPTION: TODO...
    void set_trajectory_visu(visualization_msgs::MarkerArray new_trajectory_visu);

    // DESCRIPTION: TODO...
    void set_trajectory_sampling_visu(visualization_msgs::MarkerArray new_trajectory_sampling_visu);

    // DESCRIPTION: TODO...
    void clear_trajectory_data();

    // DESCRIPTION: TODO...
    void clear_velocity_control_data();

    // DESCRIPTION: TODO...
    vector<double> angle_sampling_func(double angle, int dcount);

    // DESCRIPTION: TODO...
    bool isInsideTriangle(double x, double y, double edge_length, double half_angle);

    // DESCRIPTION: TODO...
    void construct_trajectory_data_by_geometry_cone(bool no_restriction=true);

    // DESCRIPTION: TODO...
    void construct_trajectory_data_by_geometry_cube(int sample_start_index=0);

    // DESCRIPTION: TODO...
    // x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    // u = [v(m/s), omega(rad/s)]
    vector<double> simple_car_model(vector<double>& x, vector<double>& u, double dt);

    // DESCRIPTION: TODO...
    vector<geometry_msgs::Point> construct_trajectory_by_model(string model_name, vector<double> x_init, vector<double> u, double dt=0.01);

    // DESCRIPTION: TODO...
    void construct_trajectory_data_by_simple_car_model(double robot_min_lat_velo, 
                                                       double robot_max_lat_velo, 
                                                       double robot_max_yaw_velo, 
                                                       double dt=0.01,
                                                       int sample_start_index=0);

    // DESCRIPTION: TODO...
    void read_input_data(string tdata_path);

    // DESCRIPTION: TODO...
    void read_trajectory_data(string tdata_path);

    // DESCRIPTION: TODO...
    void read_sampling_data(string tdata_path);

    // DESCRIPTION: TODO...
    void read_velocity_control_data(string tdata_path);

    // DESCRIPTION: TODO...
    void fill_trajectory_sampling_visu();

    // DESCRIPTION: TODO...
    void publishFrame(string origin_frame_name, string frame_name, geometry_msgs::Pose frame_pose);

    // DESCRIPTION: TODO...
    void publishFrame(string origin_frame_name, vector<vector<geometry_msgs::Point>> frame_trajectory_point_vec);

    // DESCRIPTION: TODO...
    void publishFrame(string origin_frame_name, vector<geometry_msgs::Pose> frame_pose);

    // DESCRIPTION: TODO...
    void publish_trajectory_sampling();

    // DESCRIPTION: TODO...
    void create_trajectory_data_path();

    // DESCRIPTION: TODO...
    void save_input_data();

    // DESCRIPTION: TODO...
    void save_trajectory_data();

  private:

    tf::TransformBroadcaster br_;

    string ns_;

    string ws_name_;
    string ws_path_;

    string trajectory_name_;

    vector<vector<geometry_msgs::Point>> trajectory_data_;
    vector<geometry_msgs::Pose> sampling_data_pose_;
    vector<vector<double>> velocity_control_data_;
    vector<string> trajectory_lrm_data_;
    
    string trajectory_sampling_dataset_path_;
    string trajectory_data_path_;
    string trajectory_frame_;
    string trajectory_generation_type_;
    string geo_type_;
    
    double dt_;
    double trajectory_time_;
    double trajectory_length_;
    double trajectory_yaw_;
    double trajectory_pitch_;
    
    int trajectory_sampling_count_;                              // TODO: Review: number of sample points on the tentacle, range: 1 <= tsamp_cnt, E Z+
    int lateral_velocity_sampling_count_;
    int angular_velocity_sampling_count_;
    int trajectory_yaw_sampling_count_;                          // TODO: Review: number of tentacles along yaw direction, range: 1 <= tyaw_cnt, E Z+
    int trajectory_pitch_sampling_count_;                        // TODO: Review: number of tentacles along pitch direction, range: 1 <= tpitch_cnt, E Z+
    
    double robot_min_lat_velo_; 
    double robot_max_lat_velo_; 
    double robot_max_yaw_velo_;

    string trajectory_yaw_sampling_type_;                        // TODO: Review: parameter to adjust yaw angle sampling type of tentacles    
    string trajectory_pitch_sampling_type_;                      // TODO: Review: parameter to adjust pitch angle sampling type of tentacles
    
    double sampling_x_min_;
    double sampling_x_max_;
    int sampling_x_cnt_;

    double sampling_y_min_;
    double sampling_y_max_;
    int sampling_y_cnt_;

    double sampling_z_min_;
    double sampling_z_max_;
    int sampling_z_cnt_;

    double sampling_roll_min_;
    double sampling_roll_max_;
    int sampling_roll_cnt_;

    double sampling_pitch_min_;
    double sampling_pitch_max_;
    int sampling_pitch_cnt_;

    double sampling_yaw_min_;
    double sampling_yaw_max_;
    int sampling_yaw_cnt_;

    bool flag_geometric = false;
    bool flag_kinematic = false;

    ros::Publisher trajectory_visu_pub_;
    ros::Publisher trajectory_sampling_visu_pub_;
    ros::Publisher trajectory_sampling_arrow_visu_pub_;
    ros::Publisher sampling_arrow_visu_pub_;
    
    visualization_msgs::MarkerArray trajectory_visu_;
    visualization_msgs::MarkerArray trajectory_sampling_visu_;
    visualization_msgs::MarkerArray trajectory_sampling_arrow_visu_;
    visualization_msgs::MarkerArray sampling_arrow_visu_;

}; // END of class TrajectorySamplingUtility

#endif