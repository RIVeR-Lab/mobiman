// LAST UPDATE: 2024.03.11
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

// --CUSTOM LIBRARIES--
#include "mobiman_simulation/trajectory_sampling_utility.h"


//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TrajectorySamplingUtility::TrajectorySamplingUtility(NodeHandle& nh, string tframe)
{
  ns_ = nh.getNamespace().substr(1);

  std::cout << "[TrajectorySamplingUtility::TrajectorySamplingUtility(2)] ns_: " << ns_ << std::endl;

  if (tframe != "")
  {
    string tframe_tmp = tframe;
    if (ns_ != "")
    {
      tframe_tmp = ns_ + "/" + tframe_tmp;
    }

    set_trajectory_frame(tframe_tmp);
  }

  trajectory_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
  trajectory_sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_arrows", 10);
  sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sampling_data_arrows", 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh, vector<vector<geometry_msgs::Point>> tdata, 
                                                      string tframe, 
                                                      string tsdataset_path)
{
  ns_ = nh.getNamespace().substr(1);

  std::cout << "[TrajectorySamplingUtility::TrajectorySamplingUtility(3)] ns_: " << ns_ << std::endl;

  string tframe_tmp = tframe;
  if (ns_ != "")
  {
    tframe_tmp = ns_ + "/" + tframe_tmp;
  }

  set_trajectory_frame(tframe_tmp);

  set_trajectory_data(tdata);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
  trajectory_sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_arrows", 10);
  sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sampling_data_arrows", 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh,
                                                      string tframe,
                                                      double tlen,
                                                      int tsamp_cnt,
                                                      double tyaw,
                                                      double tpitch,
                                                      int tyaw_cnt, 
                                                      int tpitch_cnt,
                                                      string tsdataset_path,
                                                      string tyaw_samp_type,
                                                      string tpitch_samp_type)
{
  ns_ = nh.getNamespace().substr(1);

  std::cout << "[TrajectorySamplingUtility::TrajectorySamplingUtility(11)] ns_: " << ns_ << std::endl;

  string tframe_tmp = tframe;
  if (ns_ != "")
  {
    tframe_tmp = ns_ + "/" + tframe_tmp;
  }

  set_trajectory_frame(tframe_tmp);

  set_trajectory_generation_type("geometric");
  set_trajectory_length(tlen);
  set_trajectory_sampling_count(tsamp_cnt);
  set_trajectory_yaw(tyaw);
  set_trajectory_pitch(tpitch);
  set_trajectory_yaw_sampling_count(tyaw_cnt);
  set_trajectory_pitch_sampling_count(tpitch_cnt);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  set_trajectory_yaw_sampling_type(tyaw_samp_type);
  set_trajectory_pitch_sampling_type(tpitch_samp_type);
  
  construct_trajectory_data_by_geometry_cone();
  
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
  trajectory_sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_arrows", 10);
  sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sampling_data_arrows", 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TrajectorySamplingUtility::TrajectorySamplingUtility( NodeHandle& nh,
                                                      string tframe,
                                                      double tlen,
                                                      int tsamp_cnt,
                                                      int lat_velo_cnt,
                                                      int ang_velo_cnt,
                                                      double max_lat_velo,
                                                      double max_yaw_velo,
                                                      double dt,
                                                      string tsdataset_path,
                                                      string tyaw_samp_type,
                                                      string tpitch_samp_type)
{
  ns_ = nh.getNamespace().substr(1);

  std::cout << "[TrajectorySamplingUtility::TrajectorySamplingUtility(12)] ns_: " << ns_ << std::endl;

  string tframe_tmp = tframe;
  if (ns_ != "")
  {
    tframe_tmp = ns_ + "/" + tframe_tmp;
  }

  set_trajectory_frame(tframe_tmp);

  set_trajectory_generation_type("kinematic");
  set_trajectory_length(tlen);
  set_trajectory_sampling_count(tsamp_cnt);
  set_lateral_velocity_sampling_count(lat_velo_cnt);
  set_angular_velocity_sampling_count(ang_velo_cnt);
  set_trajectory_sampling_dataset_path(tsdataset_path);
  set_trajectory_yaw_sampling_type(tyaw_samp_type);
  set_trajectory_pitch_sampling_type(tpitch_samp_type);

  construct_trajectory_data_by_simple_car_model(max_lat_velo, max_yaw_velo, dt);
  
  fill_trajectory_sampling_visu();
  save_trajectory_data();
  save_input_data();

  trajectory_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectories", 10);
  trajectory_sampling_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_points", 10);
  trajectory_sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_sampling_arrows", 10);
  sampling_arrow_visu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sampling_data_arrows", 10);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TrajectorySamplingUtility::~TrajectorySamplingUtility()
{
  //ROS_INFO( "Calling Destructor for Trajectory_Sampling_Utility..." );
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<vector<geometry_msgs::Point>> TrajectorySamplingUtility::get_trajectory_data()
{
  return trajectory_data_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<vector<double>> TrajectorySamplingUtility::get_velocity_control_data()
{
  return velocity_control_data_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<string> TrajectorySamplingUtility::get_trajectory_lrm_data()
{
  return trajectory_lrm_data_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_sampling_dataset_path()
{
  return trajectory_sampling_dataset_path_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_data_path()
{
  return trajectory_data_path_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_frame()
{
  return trajectory_frame_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_generation_type()
{
  return trajectory_generation_type_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double TrajectorySamplingUtility::get_trajectory_time()
{
  return trajectory_time_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double TrajectorySamplingUtility::get_trajectory_length()
{
  return trajectory_length_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double TrajectorySamplingUtility::get_trajectory_yaw()
{
  return trajectory_yaw_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double TrajectorySamplingUtility::get_trajectory_pitch()
{
  return trajectory_pitch_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TrajectorySamplingUtility::get_trajectory_sampling_count()
{
  return trajectory_sampling_count_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TrajectorySamplingUtility::get_lateral_velocity_sampling_count()
{
  return lateral_velocity_sampling_count_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TrajectorySamplingUtility::get_angular_velocity_sampling_count()
{
  return angular_velocity_sampling_count_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TrajectorySamplingUtility::get_trajectory_yaw_sampling_count()
{
  return trajectory_yaw_sampling_count_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TrajectorySamplingUtility::get_trajectory_pitch_sampling_count()
{
  return trajectory_pitch_sampling_count_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_yaw_sampling_type()
{
  return trajectory_yaw_sampling_type_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
string TrajectorySamplingUtility::get_trajectory_pitch_sampling_type()
{
  return trajectory_pitch_sampling_type_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher TrajectorySamplingUtility::get_trajectory_visu_pub()
{
  return trajectory_visu_pub_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher TrajectorySamplingUtility::get_trajectory_sampling_visu_pub()
{
  return trajectory_sampling_visu_pub_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ros::Publisher TrajectorySamplingUtility::get_trajectory_sampling_arrow_visu_pub()
{
  return trajectory_sampling_arrow_visu_pub_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::MarkerArray TrajectorySamplingUtility::get_trajectory_visu()
{
  return trajectory_visu_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::MarkerArray TrajectorySamplingUtility::get_trajectory_sampling_visu()
{
  return trajectory_sampling_visu_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_workspace_name(std::string ws_name)
{
  ws_name_ = ws_name;

  ws_path_ = ros::package::getPath(ws_name_) + "/";
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_name(std::string trajectory_name)
{
  trajectory_name_ = trajectory_name;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_data(vector<vector<geometry_msgs::Point>> new_trajectory_data)
{
  trajectory_data_ = new_trajectory_data;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_velocity_control_data(vector<vector<double>> new_velocity_control_data)
{
  velocity_control_data_ = new_velocity_control_data;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_lrm_data(vector<string> new_trajectory_lrm_data)
{
  trajectory_lrm_data_ = new_trajectory_lrm_data;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_sampling_dataset_path(string new_trajectory_sampling_dataset_path)
{
  trajectory_sampling_dataset_path_ = new_trajectory_sampling_dataset_path;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_data_path(string new_trajectory_data_path)
{
  trajectory_data_path_ = new_trajectory_data_path;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_frame(string new_trajectory_frame)
{
  trajectory_frame_ = new_trajectory_frame;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_generation_type(string new_trajectory_generation_type)
{
  trajectory_generation_type_ = new_trajectory_generation_type;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_time(double new_trajectory_time)
{
  trajectory_time_ = new_trajectory_time;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_length(double new_trajectory_length)
{
  trajectory_length_ = new_trajectory_length;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_yaw(double new_trajectory_yaw)
{
  trajectory_yaw_ = new_trajectory_yaw;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_pitch(double new_trajectory_pitch)
{
  trajectory_pitch_ = new_trajectory_pitch;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_x_min(double sampling_x_min)
{
  sampling_x_min_ = sampling_x_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_x_max(double sampling_x_max)
{
  sampling_x_max_ = sampling_x_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_x_cnt(int sampling_x_cnt)
{
  sampling_x_cnt_ = sampling_x_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_y_min(double sampling_y_min)
{
  sampling_y_min_ = sampling_y_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_y_max(double sampling_y_max)
{
  sampling_y_max_ = sampling_y_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_y_cnt(int sampling_y_cnt)
{
  sampling_y_cnt_ = sampling_y_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_z_min(double sampling_z_min)
{
  sampling_z_min_ = sampling_z_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_z_max(double sampling_z_max)
{
  sampling_z_max_ = sampling_z_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_z_cnt(int sampling_z_cnt)
{
  sampling_z_cnt_ = sampling_z_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_roll_min(double sampling_roll_min)
{
  sampling_roll_min_ = sampling_roll_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_roll_max(double sampling_roll_max)
{
  sampling_roll_max_ = sampling_roll_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_roll_cnt(int sampling_roll_cnt)
{
  sampling_roll_cnt_ = sampling_roll_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_pitch_min(double sampling_pitch_min)
{
  sampling_pitch_min_ = sampling_pitch_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_pitch_max(double sampling_pitch_max)
{
  sampling_pitch_max_ = sampling_pitch_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_pitch_cnt(int sampling_pitch_cnt)
{
  sampling_pitch_cnt_ = sampling_pitch_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_yaw_min(double sampling_yaw_min)
{
  sampling_yaw_min_ = sampling_yaw_min;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_yaw_max(double sampling_yaw_max)
{
  sampling_yaw_max_ = sampling_yaw_max;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_sampling_yaw_cnt(int sampling_yaw_cnt)
{
  sampling_yaw_cnt_ = sampling_yaw_cnt;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_sampling_count(int new_trajectory_sampling_count)
{
  trajectory_sampling_count_ = new_trajectory_sampling_count;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_lateral_velocity_sampling_count(int new_lateral_velocity_sampling_count)
{
  lateral_velocity_sampling_count_ = new_lateral_velocity_sampling_count;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_angular_velocity_sampling_count(int new_angular_velocity_sampling_count)
{
  angular_velocity_sampling_count_ = new_angular_velocity_sampling_count;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_yaw_sampling_count(int new_trajectory_yaw_sampling_count)
{
  trajectory_yaw_sampling_count_ = new_trajectory_yaw_sampling_count;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_pitch_sampling_count(int new_trajectory_pitch_sampling_count)
{
  trajectory_pitch_sampling_count_ = new_trajectory_pitch_sampling_count;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_yaw_sampling_type(string new_trajectory_yaw_sampling_type)
{
  trajectory_yaw_sampling_type_ = new_trajectory_yaw_sampling_type;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_pitch_sampling_type(string new_trajectory_pitch_sampling_type)
{
  trajectory_pitch_sampling_type_ = new_trajectory_pitch_sampling_type;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_visu_pub(ros::Publisher new_trajectory_visu_pub)
{
  trajectory_visu_pub_ = new_trajectory_visu_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_sampling_visu_pub(ros::Publisher new_trajectory_sampling_visu_pub)
{
  trajectory_sampling_visu_pub_ =  new_trajectory_sampling_visu_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_sampling_arrow_visu_pub(ros::Publisher trajectory_sampling_arrow_visu_pub)
{
  trajectory_sampling_arrow_visu_pub_ =  trajectory_sampling_arrow_visu_pub;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_visu(visualization_msgs::MarkerArray new_trajectory_visu)
{
  trajectory_visu_ = new_trajectory_visu;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::set_trajectory_sampling_visu(visualization_msgs::MarkerArray new_trajectory_sampling_visu)
{
  trajectory_sampling_visu_ = new_trajectory_sampling_visu;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::clear_trajectory_data()
{
  for(int i = 0; i < trajectory_data_.size(); i++)
  {
    trajectory_data_[i].clear();
  }
  trajectory_data_.clear();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::clear_velocity_control_data()
{
  for(int i = 0; i < velocity_control_data_.size(); i++)
  {
    velocity_control_data_[i].clear();
  }
  velocity_control_data_.clear();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<double> TrajectorySamplingUtility::angle_sampling_func(double angle, int dcount)
{
  vector<double> av;
  av.clear();

  if(dcount <= 0)
  { 
    return av;
  }
  else if(dcount == 1)
  {
    av.push_back(0);
    return av;
  }
  else if(dcount == 2)
  {
    av.push_back(0);
    av.push_back(-0.5*angle);
    return av;
  }
  else if(dcount % 2 == 0)
  {
    av.push_back(0);
    
    int left_count = 0.5 * (dcount - 2) + 1;
    double left_delta = (0.5*angle) / left_count;
    for (int i = 1; i <= left_count; ++i)
    {
      av.push_back(i*left_delta);
    }

    int right_count = left_count - 1;
    double right_delta = (0.5*angle) / right_count;
    for (int i = 1; i <= right_count; ++i)
    {
      av.push_back(-i*right_delta);
    }
    return av;
  }
  else
  {
    av.push_back(0);
    
    int count = 0.5 * (dcount - 1);
    double delta = (0.5*angle) / count;
    for (int i = 1; i <= count; ++i)
    {
      av.push_back(i*delta);
    }
    for (int i = 1; i <= count; ++i)
    {
      av.push_back(-i*delta);
    }
    return av;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TrajectorySamplingUtility::isInsideTriangle(double x, double y, double edge_length, double half_angle)
{
  double angle = atan2(y, x);
  return (abs(angle) <= abs(half_angle)) && (abs(x) <= edge_length * cos(abs(angle)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cone(bool no_restriction)
{
  geo_type_ = "cone";

  clear_trajectory_data();
  trajectory_lrm_data_.clear();

  // SAMPLE YAW
  vector<double> yaw_samples = angle_sampling_func(trajectory_yaw_, trajectory_yaw_sampling_count_);

  // SAMPLE PITCH
  vector<double> pitch_samples = angle_sampling_func(trajectory_pitch_, trajectory_pitch_sampling_count_);

  // CONSTRUCT TENTACLE (SAMPLED TRAJECTORY) DATA
  int pcount = 0;
  int tcount = 0;
  double x;
  double y;
  double z;
  double ds = 1;
  double delta_dist = trajectory_length_ / trajectory_sampling_count_;

  // TENTACLE 0 (Contains single sampling point at the center of the robot)
  /*
  vector<geometry_msgs::Point> trajectory0;
  geometry_msgs::Point p0;
  p0.x = 0;
  p0.y = 0;
  p0.z = 0;
  trajectory0.push_back(p0);
  trajectory_data_.push_back(trajectory0);
  trajectory_lrm_data_.push_back("M");
  */

  // ALL OTHER TENTACLES
  for (int j = 0; j < pitch_samples.size(); ++j)
  {
    for (int i = 0; i < yaw_samples.size(); ++i)
    {
      vector<geometry_msgs::Point> trajectory;
      pcount = 0;
      x = 0;
      y = 0;
      z = 0;

      if (no_restriction)
      {
        while( (pcount+1) * delta_dist <= trajectory_length_ )
        {
          if (trajectory_generation_type_ == "circular")
          {
            ds = (pcount+1);
          }

          if (abs(yaw_samples[i]) > 0.5*PI)
          {
            x += -1 * delta_dist * cos(ds* ( PI - abs(yaw_samples[i]) ) ) * cos(pitch_samples[j]);
            y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
            z += delta_dist * sin(pitch_samples[j]);
          }
          else
          {
            x += delta_dist * cos(ds*yaw_samples[i]) * cos(pitch_samples[j]);
            y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
            z += delta_dist * sin(pitch_samples[j]);
          }

          geometry_msgs::Point p;
          p.x = x;
          p.y = y;
          p.z = z;
          trajectory.push_back(p);

          pcount++;
        }
        tcount++;
      }
      else
      {
        while(isInsideTriangle(x, y, trajectory_length_, 0.5*trajectory_yaw_) && abs(ds*yaw_samples[i]) <= PI)
        {
          if (trajectory_generation_type_ == "circular")
          {
            ds = (pcount+1);
          }

          x += delta_dist * cos(ds*yaw_samples[i]) * cos(pitch_samples[j]);
          y += delta_dist * sin(ds*yaw_samples[i]) * cos(pitch_samples[j]);
          z += delta_dist * sin(pitch_samples[j]);

          geometry_msgs::Point p;
          p.x = x;
          p.y = y;
          p.z = z;
          trajectory.push_back(p);

          pcount++;
        }
        tcount++;
      }

      // ADD TRAJECTORY DATA
      trajectory_data_.push_back(trajectory);

      if (yaw_samples[i] > 0)
      {
        trajectory_lrm_data_.push_back("L");
      }
      else if (yaw_samples[i] < 0)
      {
        trajectory_lrm_data_.push_back("R");
      }
      else
      {
        trajectory_lrm_data_.push_back("M");
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube(int sample_start_index)
{
  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] START" << std::endl;

  geo_type_ = "cube";

  sampling_data_pose_.clear();

  vector<double> samp_x;
  vector<double> samp_y;
  vector<double> samp_z;

  // Sample x, y, z ranges
  if (sampling_x_min_ < 0 && sample_start_index > 0)
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] CUSTOM x" << std::endl;
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] sampling_x_cnt_: " << sampling_x_cnt_ << std::endl;

    int sc;
    if (sampling_x_cnt_ % 2 == 0)
    {
      //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] EVEN" << std::endl;
      sc = 0.5 * sampling_x_cnt_;
    }
    else
    {
      //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] ODD" << std::endl;
      sc = 0.5 * (sampling_x_cnt_-1);
    }

    
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] sc: " << sc << std::endl;
    
    
    vector<double> samp_x_up = sampleRange(0.0, sampling_x_max_, sampling_x_cnt_-sc+1);
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_x_up.size(): " << samp_x_up.size() << std::endl;
    print(samp_x_up);
    for (size_t i = sample_start_index; i < samp_x_up.size(); i++)
    {
      //std::cout << i << std::endl;
      samp_x.push_back(samp_x_up[i]);
    }

    vector<double> samp_x_down = sampleRange(sampling_x_min_, 0.0, sc+1);    
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_x_down.size(): " << samp_x_down.size() << std::endl;
    print(samp_x_down);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] sample_start_index: " << sample_start_index << std::endl;
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] idx: " << samp_x_down.size()-1-sample_start_index << std::endl;

    for (size_t i = 0; i < samp_x_down.size()-sample_start_index; i++)
    {
      //std::cout << i << std::endl;
      samp_x.push_back(samp_x_down[i]);
    }

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_x_up size: " << samp_x_up.size() << std::endl;
    //print(samp_x_up);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_x_down size: " << samp_x_down.size() << std::endl;
    //print(samp_x_down);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_x size: " << samp_x.size() << std::endl;
    //print(samp_x);
  }
  else
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] REGULAR x" << std::endl;
    samp_x = sampleRange(sampling_x_min_, sampling_x_max_, sampling_x_cnt_);
  }

  if (sampling_y_min_ < 0 && sample_start_index > 0)
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] CUSTOM y" << std::endl;

    int sc;
    if (sampling_y_cnt_ % 2 == 0)
    {
      sc = 0.5 * sampling_y_cnt_;
    }
    else
    {
      sc = 0.5 * (sampling_y_cnt_-1);
    }

    vector<double> samp_y_up = sampleRange(0.0, sampling_y_max_, sampling_y_cnt_-sc+1);
    for (size_t i = sample_start_index; i < samp_y_up.size(); i++)
    {
      samp_y.push_back(samp_y_up[i]);
    }
    
    vector<double> samp_y_down = sampleRange(sampling_y_min_, 0.0, sc+1);
    for (size_t i = 0; i < samp_y_down.size()-sample_start_index; i++)
    {
      samp_y.push_back(samp_y_down[i]);
    }

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_y_up size: " << samp_y_up.size() << std::endl;
    //print(samp_y_up);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_y_down size: " << samp_y_down.size() << std::endl;
    //print(samp_y_down);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_y size: " << samp_y.size() << std::endl;
    //print(samp_y);
  }
  else
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] REGULAR y" << std::endl;
    samp_y = sampleRange(sampling_y_min_, sampling_y_max_, sampling_y_cnt_);
  }

  if (sampling_z_min_ < 0 && sample_start_index > 0)
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] CUSTOM z" << std::endl;

    int sc;
    if (sampling_z_cnt_ % 2 == 0)
    {
      sc = 0.5 * sampling_z_cnt_;
    }
    else
    {
      sc = 0.5 * (sampling_z_cnt_-1);
    }

    vector<double> samp_z_up = sampleRange(0.0, sampling_z_max_, sampling_z_cnt_-sc+1);
    for (size_t i = sample_start_index; i < samp_z_up.size(); i++)
    {
      samp_z.push_back(samp_z_up[i]);
    }
    
    vector<double> samp_z_down = sampleRange(sampling_z_min_, 0.0, sc+1);
    for (size_t i = 0; i < samp_z_down.size()-sample_start_index; i++)
    {
      samp_z.push_back(samp_z_down[i]);
    }

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_z_up size: " << samp_z_up.size() << std::endl;
    //print(samp_z_up);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_z_down size: " << samp_z_down.size() << std::endl;
    //print(samp_z_down);

    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_z size: " << samp_z.size() << std::endl;
    //print(samp_z);
  }
  else
  {
    //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] REGULAR z" << std::endl;
    samp_z = sampleRange(sampling_z_min_, sampling_z_max_, sampling_z_cnt_);
  }

  vector<double> samp_roll = sampleRange(sampling_roll_min_, sampling_roll_max_, sampling_roll_cnt_, true);
  vector<double> samp_pitch = sampleRange(sampling_pitch_min_, sampling_pitch_max_, sampling_pitch_cnt_, true);
  vector<double> samp_yaw = sampleRange(sampling_yaw_min_, sampling_yaw_max_, sampling_yaw_cnt_, true);

  std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_roll size: " << samp_roll.size() << std::endl;
  print(samp_roll);

  std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_pitch size: " << samp_pitch.size() << std::endl;
  print(samp_pitch);

  std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_yaw size: " << samp_yaw.size() << std::endl;
  print(samp_yaw);

  for (size_t r = 0; r < samp_roll.size(); r++)
  {
    for (size_t p = 0; p < samp_pitch.size(); p++)
    {
      for (size_t y = 0; y < samp_yaw.size(); y++)
      {
        for (size_t k = 0; k < samp_z.size(); k++)
        {
          for (size_t j = 0; j < samp_y.size(); j++)
          {
            for (size_t i = 0; i < samp_x.size(); i++)
            {
              geometry_msgs::Pose po;
              po.position.x = samp_x[i];
              po.position.y = samp_y[j];
              po.position.z = samp_z[k];

              //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] roll: " << samp_roll[r]*180/PI << std::endl;
              //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] samp_pitch: " << samp_pitch[p]*180/PI << std::endl;
              //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] yaw: " << samp_yaw[y]*180/PI << std::endl;

              tf2::Quaternion qu = getQuaternionFromRPY(samp_roll[r], samp_pitch[p], samp_yaw[y]);

              po.orientation.x = qu.x();
              po.orientation.y = qu.y();
              po.orientation.z = qu.z();
              po.orientation.w = qu.w();

              sampling_data_pose_.push_back(po);
            }
          }
        }
      }
    }
  }

  std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] sampling_data_pose_ size: " << sampling_data_pose_.size() << std::endl;

  std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_geometry_cube] END" << std::endl; 
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<double> TrajectorySamplingUtility::simple_car_model(vector<double>& x, vector<double>& u, double dt)
{
  x[2] += u[1] * dt;
  x[0] += u[0] * cos(x[2]) * dt;
  x[1] += u[0] * sin(x[2]) * dt;

  x[3] = u[0];
  x[4] = u[1];

  return x;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
vector<geometry_msgs::Point> TrajectorySamplingUtility::construct_trajectory_by_model(string model_name, vector<double> x_init, vector<double> u, double dt)
{
  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_by_model] START" << std::endl;

  vector<geometry_msgs::Point> trajectory;
  vector<double> x = x_init;
  double dl;

  double current_ttime = 0;
  double current_tlength = 0;

  geometry_msgs::Point p_new;
  geometry_msgs::Point p_pre;
  p_pre.x = 0;
  p_pre.y = 0;
  p_pre.z = 0;

  //while(current_tlength < trajectory_length_ && current_ttime < trajectory_time_)
  while(current_ttime < trajectory_time_)
  {
    if (model_name == "simple_car")
    {
      x = simple_car_model(x, u, dt);
    }

    p_new.x = x[0];
    p_new.y = x[1];
    p_new.z = x[2];

    current_ttime += dt;

    dl = find_Euclidean_distance(p_new, p_pre);
    p_pre = p_new;
    current_tlength += dl;
    trajectory.push_back(p_new);

    if (dl == 0)
    {
      break;
    }
  }

  //std::cout << "TrajectorySamplingUtility::construct_trajectory_by_model -> trajectory size: " << trajectory.size() << std::endl;
  //print(trajectory);

  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_by_model] END" << std::endl;

  return trajectory;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model(double robot_min_lat_velo, 
                                                                              double robot_max_lat_velo, 
                                                                              double robot_max_yaw_velo, 
                                                                              double dt, 
                                                                              int sample_start_index)
{
  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model] START" << std::endl;

  flag_kinematic_ = true;
  dt_ = dt;
  robot_min_lat_velo_ = robot_min_lat_velo;
  robot_max_lat_velo_ = robot_max_lat_velo;
  robot_max_yaw_velo_ = robot_max_yaw_velo;

  clear_trajectory_data();
  trajectory_lrm_data_.clear();
  clear_velocity_control_data();

  if (trajectory_data_path_ == "")
  {
    create_trajectory_data_path();
  }

  ofstream velocity_control_data_stream;
  velocity_control_data_stream.open(ws_path_ + trajectory_data_path_ + "velocity_control_data.csv");
  //velocity_control_data_stream << "lateral_velocity_samples[m/s],angular_velocity_samples[rad/s]\n";

  vector<double> lateral_velocity_samples = sampling_func(robot_min_lat_velo, robot_max_lat_velo, lateral_velocity_sampling_count_);
  vector<double> angular_velocity_samples = angle_sampling_func(2*robot_max_yaw_velo, angular_velocity_sampling_count_);

  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model] lateral_velocity_samples" << std::endl;
  //print(lateral_velocity_samples);

  // x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
  vector<double> x_init{0, 0, 0, 0, 0};

  // u = [v(m/s), omega(rad/s)]
  vector<double> u;

  vector<geometry_msgs::Point> traj;
  for (int i = 0; i < lateral_velocity_samples.size(); ++i)
  {
    for (int j = 0; j < angular_velocity_samples.size(); ++j)
    {
      u.push_back(lateral_velocity_samples[i]);
      u.push_back(angular_velocity_samples[j]);
      velocity_control_data_.push_back(u);
      velocity_control_data_stream << lateral_velocity_samples[i] << "," << angular_velocity_samples[j] << "\n";

      traj = construct_trajectory_by_model("simple_car", x_init, u, dt);
      subsample(traj, trajectory_sampling_count_);

      if (sample_start_index > 0)
      {
        //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model] DO NOT INCLUDE FIRST " << sample_start_index << " POINTS!" << std::endl;
        
        vector<geometry_msgs::Point> traj_tmp;
        for (size_t i = sample_start_index; i < traj.size(); i++)
        {
          traj_tmp.push_back(traj[i]);
        }
        traj = traj_tmp;
      }
      
      trajectory_data_.push_back(traj);
      
      u.clear();
    }
  }
  velocity_control_data_stream.close();

  //std::cout << "[TrajectorySamplingUtility::construct_trajectory_data_by_simple_car_model] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::update_sampling_data_from_trajectory_data()
{
  sampling_data_pose_.clear();

  for (size_t i = 0; i < trajectory_data_.size(); i++)
  {
    for (size_t j = 0; j < trajectory_data_[i].size(); j++)
    {
      geometry_msgs::Pose po;
      po.position = trajectory_data_[i][j];
      po.orientation.x = 0.0;
      po.orientation.y = 0.0;
      po.orientation.z = 0.0;
      po.orientation.w = 1.0;

      if (trajectory_generation_type_ == "kinematic")
      {
        po.position.z = 0.0;

        double yaw = trajectory_data_[i][j].z;
        tf2::Quaternion quat = getQuaternionFromRPY(0.0, 0.0, yaw);
        po.orientation.x = quat.x();
        po.orientation.y = quat.y();
        po.orientation.z = quat.z();
        po.orientation.w = quat.w();
      }
      
      sampling_data_pose_.push_back(po);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_input_data(string tdata_path)
{
  std::cout << "[TrajectorySamplingUtility::read_input_data] START" << std::endl;

  std::string tmp;

  tmp = read_input_data(tdata_path, "ws_name");
  if (tmp != "")
  {
    ws_name_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_name");
  if (tmp != "")
  {
    trajectory_name_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_data_path");
  if (tmp != "")
  {
    trajectory_data_path_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_sampling_dataset_path");
  if (tmp != "")
  {
    trajectory_sampling_dataset_path_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_frame");
  if (tmp != "")
  {
    trajectory_frame_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_gen_type");
  if (tmp != "")
  {
    trajectory_generation_type_ = tmp;
  }

  tmp = read_input_data(tdata_path, "geo_type");
  if (tmp != "")
  {
    geo_type_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_length");
  if (tmp != "")
  {
    trajectory_length_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_yaw");
  if (tmp != "")
  {
    trajectory_yaw_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_pitch");
  if (tmp != "")
  {
    trajectory_pitch_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_yaw_sampling_count");
  if (tmp != "")
  {
    trajectory_yaw_sampling_count_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_pitch_sampling_count");
  if (tmp != "")
  {
    trajectory_pitch_sampling_count_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_yaw_sampling_type");
  if (tmp != "")
  {
    trajectory_yaw_sampling_type_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_pitch_sampling_type");
  if (tmp != "")
  {
    trajectory_pitch_sampling_type_ = tmp;
  }

  tmp = read_input_data(tdata_path, "trajectory_sampling_count");
  if (tmp != "")
  {
    trajectory_sampling_count_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_x_min");
  if (tmp != "")
  {
    sampling_x_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_x_max");
  if (tmp != "")
  {
    sampling_x_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_x_cnt");
  if (tmp != "")
  {
    sampling_x_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_y_min");
  if (tmp != "")
  {
    sampling_y_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_y_max");
  if (tmp != "")
  {
    sampling_y_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_y_cnt");
  if (tmp != "")
  {
    sampling_y_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_z_min");
  if (tmp != "")
  {
    sampling_z_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_z_max");
  if (tmp != "")
  {
    sampling_z_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_z_cnt");
  if (tmp != "")
  {
    sampling_z_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_roll_min");
  if (tmp != "")
  {
    sampling_roll_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_roll_max");
  if (tmp != "")
  {
    sampling_roll_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_roll_cnt");
  if (tmp != "")
  {
    sampling_roll_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_pitch_min");
  if (tmp != "")
  {
    sampling_pitch_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_pitch_max");
  if (tmp != "")
  {
    sampling_pitch_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_pitch_cnt");
  if (tmp != "")
  {
    sampling_pitch_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_yaw_min");
  if (tmp != "")
  {
    sampling_yaw_min_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_yaw_max");
  if (tmp != "")
  {
    sampling_yaw_max_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "sampling_yaw_cnt");
  if (tmp != "")
  {
    sampling_yaw_cnt_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "trajectory_time");
  if (tmp != "")
  {
    trajectory_time_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "dt");
  if (tmp != "")
  {
    dt_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "lateral_velocity_sampling_count");
  if (tmp != "")
  {
    lateral_velocity_sampling_count_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "angular_velocity_sampling_count");
  if (tmp != "")
  {
    angular_velocity_sampling_count_ = stoi(tmp);
  }

  tmp = read_input_data(tdata_path, "robot_min_lat_velo");
  if (tmp != "")
  {
    robot_min_lat_velo_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "robot_max_lat_velo");
  if (tmp != "")
  {
    robot_max_lat_velo_ = stod(tmp);
  }

  tmp = read_input_data(tdata_path, "robot_max_yaw_velo");
  if (tmp != "")
  {
    robot_max_yaw_velo_ = stod(tmp);
  }

  std::cout << "[TrajectorySamplingUtility::read_input_data] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string TrajectorySamplingUtility::read_input_data(string tdata_path, string data_name)
{
  //std::cout << "[trajectory_sampling_utility::read_input_data(2)] START" << std::endl;

  ifstream tdata_stream(ws_path_ + tdata_path + "input_data.csv");

  string::size_type sz;
  string line, spoint, sval;

  if (tdata_stream.is_open())
  {
    int c = 0;
    while ( getline(tdata_stream, line) )
    {
      stringstream s_line(line);

      getline(s_line, spoint, ',');
      //std::cout << "[trajectory_sampling_utility::read_input_data(2)] spoint 1: " << spoint << std::endl;

      if (spoint == data_name)
      {
        getline(s_line, spoint, ',');
        //std::cout << "[trajectory_sampling_utility::read_input_data(2)] spoint 2: " << spoint << std::endl;
        return spoint;
      }

      //std::cout << "[trajectory_sampling_utility::read_input_data(2)] line " << c << "-> " << line << std::endl;
      c++;
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_input_data(2)] ERROR: Unable to open file!" << std::endl;
  }

  //std::cout << "[trajectory_sampling_utility::read_input_data] END(2)" << std::endl;
  return "";
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_trajectory_data(string tdata_path)
{
  string tgen_type = read_input_data(tdata_path, "trajectory_gen_type");

  if (tgen_type == "kinematic")
  {
    flag_kinematic_ = true;
  }

  std::cout << "[trajectory_sampling_utility::read_trajectory_data] tgen_type: " << tgen_type << std::endl;

  clear_trajectory_data();

  string::size_type sz;
  string line, spoint, sval;

  std::cout << "[trajectory_sampling_utility::read_trajectory_data] tdata_path: " << tdata_path << std::endl;

  ifstream tdata_stream(ws_path_ + tdata_path + "trajectory_data.csv");

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<geometry_msgs::Point> traj;
      stringstream s_line(line);

      while( getline(s_line, spoint, ',') ) 
      {
        vector<float> pv;
        stringstream s_val(spoint);

        while( getline(s_val, sval, ' ') ) 
        {
          pv.push_back(stod(sval, &sz));
        }

        geometry_msgs::Point p;
        p.x = pv[0];
        p.y = pv[1];
        p.z = pv[2];
        
        traj.push_back(p);
      }
      trajectory_data_.push_back(traj);
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_trajectory_data] ERROR: Unable to open file!" << std::endl;
  }

  fill_trajectory_sampling_visu();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_trajectory_data(string tdata_path, vector<vector<geometry_msgs::Point>>& trajectory_data)
{
  string tgen_type = read_input_data(tdata_path, "trajectory_gen_type");

  if (tgen_type == "kinematic")
  {
    flag_kinematic_ = true;
  }

  std::cout << "[trajectory_sampling_utility::read_trajectory_data(2)] tgen_type: " << tgen_type << std::endl;

  trajectory_data.clear();

  string::size_type sz;
  string line, spoint, sval;

  std::cout << "[trajectory_sampling_utility::read_trajectory_data(2)] tdata_path: " << tdata_path << std::endl;

  ifstream tdata_stream(ws_path_ + tdata_path + "trajectory_data.csv");

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<geometry_msgs::Point> traj;
      stringstream s_line(line);

      while( getline(s_line, spoint, ',') ) 
      {
        vector<float> pv;
        stringstream s_val(spoint);

        while( getline(s_val, sval, ' ') ) 
        {
          pv.push_back(stod(sval, &sz));
        }

        geometry_msgs::Point p;
        p.x = pv[0];
        p.y = pv[1];
        p.z = pv[2];
        
        traj.push_back(p);
      }
      trajectory_data.push_back(traj);
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_trajectory_data(2)] ERROR: Unable to open file!" << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_sampling_data(string tdata_path)
{
  std::cout << "[trajectory_sampling_utility::read_sampling_data] START" << std::endl;

  sampling_data_pose_.clear();

  string::size_type sz;
  string line, spoint, sval;

  std::string file_path = ws_path_ + tdata_path + "sampling_data.csv";
  std::cout << "[trajectory_sampling_utility::read_sampling_data] file_path: " << file_path << std::endl;
  ifstream tdata_stream(file_path);

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<geometry_msgs::Point> traj;
      stringstream s_line(line);

      getline(s_line, spoint, ',');
      stringstream s_val(spoint);

      vector<float> pv;
      while( getline(s_val, sval, ' ') ) 
      {
        pv.push_back(stod(sval, &sz));    
      }

      geometry_msgs::Pose po;
      po.position.x = pv[0];
      po.position.y = pv[1];
      po.position.z = pv[2];
      
      tf2::Quaternion quat = getQuaternionFromRPY(pv[3], pv[4], pv[5]);
      
      po.orientation.x = quat.x();
      po.orientation.y = quat.y();
      po.orientation.z = quat.z();
      po.orientation.w = quat.w();
      sampling_data_pose_.push_back(po);
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_sampling_data] ERROR: Unable to open file!" << std::endl;
  }

  fill_trajectory_sampling_visu();

  std::cout << "[trajectory_sampling_utility::read_sampling_data] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_sampling_data(std::vector<std::string>& tdata_path_multi)
{
  std::cout << "[trajectory_sampling_utility::read_sampling_data] START" << std::endl;

  sampling_data_pose_.clear();

  string::size_type sz;
  string line, spoint, sval;

  for (size_t i = 0; i < tdata_path_multi.size(); i++)
  {
    std::string file_path = ws_path_ + tdata_path_multi[i] + "sampling_data.csv";
    std::cout << "[trajectory_sampling_utility::read_sampling_data] file_path: " << file_path << std::endl;
    ifstream tdata_stream(file_path);

    if (tdata_stream.is_open())
    {
      while ( getline(tdata_stream, line) )
      {
        vector<geometry_msgs::Point> traj;
        stringstream s_line(line);

        getline(s_line, spoint, ',');
        stringstream s_val(spoint);

        vector<float> pv;
        while( getline(s_val, sval, ' ') ) 
        {
          pv.push_back(stod(sval, &sz));    
        }

        geometry_msgs::Pose po;
        po.position.x = pv[0];
        po.position.y = pv[1];
        po.position.z = pv[2];
        
        tf2::Quaternion quat = getQuaternionFromRPY(pv[3], pv[4], pv[5]);
        
        po.orientation.x = quat.x();
        po.orientation.y = quat.y();
        po.orientation.z = quat.z();
        po.orientation.w = quat.w();
        sampling_data_pose_.push_back(po);
      }
      tdata_stream.close();
    }
    else 
    {
      std::cout << "[trajectory_sampling_utility::read_sampling_data] ERROR: Unable to open file!" << std::endl;
    }
  }

  fill_trajectory_sampling_visu();

  std::cout << "[trajectory_sampling_utility::read_sampling_data] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_sampling_data(string tdata_path, vector<geometry_msgs::Pose>& sampling_data_pose)
{
  std::cout << "[trajectory_sampling_utility::read_sampling_data(2)] START" << std::endl;

  sampling_data_pose.clear();

  std::string::size_type sz;
  std::string line, spoint, sval;

  std::string file_path = ws_path_ + tdata_path + "sampling_data.csv";
  std::cout << "[trajectory_sampling_utility::read_sampling_data(2)] file_path: " << file_path << std::endl;
  ifstream tdata_stream(file_path);

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<geometry_msgs::Point> traj;
      stringstream s_line(line);

      getline(s_line, spoint, ',');
      stringstream s_val(spoint);

      vector<float> pv;
      while( getline(s_val, sval, ' ') ) 
      {
        pv.push_back(stod(sval, &sz));    
      }

      geometry_msgs::Pose po;
      po.position.x = pv[0];
      po.position.y = pv[1];
      po.position.z = pv[2];
      
      tf2::Quaternion quat = getQuaternionFromRPY(pv[3], pv[4], pv[5]);
      
      po.orientation.x = quat.x();
      po.orientation.y = quat.y();
      po.orientation.z = quat.z();
      po.orientation.w = quat.w();
      sampling_data_pose.push_back(po);
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_sampling_data(2)] ERROR: Unable to open file!" << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::read_velocity_control_data(string tdata_path)
{
  clear_velocity_control_data();

  string::size_type sz;
  string line, spoint, sval;

  ifstream tdata_stream(ws_path_ + tdata_path + "velocity_control_data.csv");

  if (tdata_stream.is_open())
  {
    while ( getline(tdata_stream, line) )
    {
      vector<double> velo_control;
      stringstream s_line(line);

      while( getline(s_line, sval, ',') ) 
      {
        velo_control.push_back(stod(sval, &sz));
      }
      velocity_control_data_.push_back(velo_control);
    }
    tdata_stream.close();
  }
  else 
  {
    std::cout << "[trajectory_sampling_utility::read_velocity_control_data] ERROR: Unable to open file!" << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::fill_trajectory_sampling_visu()
{
  std::cout << "[trajectory_sampling_utility::fill_trajectory_sampling_visu] START" << std::endl;

  vector<vector<geometry_msgs::Point>> trajectory_data = trajectory_data_;

  std::cout << "[trajectory_sampling_utility::fill_trajectory_sampling_visu] trajectory_data size: " << trajectory_data.size() << std::endl;
  std::cout << "[trajectory_sampling_utility::fill_trajectory_sampling_visu] trajectory_frame_: " << trajectory_frame_ << std::endl;

  trajectory_visu_.markers.clear();
  trajectory_sampling_visu_.markers.clear();
  int tsamp_cnt;

  // VISUALIZE TRAJECTORY
  for(int k = 0; k < trajectory_data.size(); k++)
  {
    // SET TRAJECTORY VISUALIZATION SETTINGS
    visualization_msgs::Marker trajectory_line_strip;
    trajectory_line_strip.ns = "trajectory" + to_string(k);
    trajectory_line_strip.id = k;
    trajectory_line_strip.header.frame_id = trajectory_frame_;
    trajectory_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_line_strip.action = visualization_msgs::Marker::ADD;
    trajectory_line_strip.pose.orientation.w = 1.0;
    trajectory_line_strip.scale.x = 0.02;
    trajectory_line_strip.color.r = 1.0;
    trajectory_line_strip.color.g = 1.0;
    trajectory_line_strip.color.b = 1.0;
    trajectory_line_strip.color.a = 1.0;

    // SET SAMPLING POINTS VISUALIZATION SETTINGS
    visualization_msgs::Marker trajectory_tsamp;
    trajectory_tsamp.ns = "sp_" + to_string(k);;
    trajectory_tsamp.id = k;
    trajectory_tsamp.header.frame_id = trajectory_frame_;
    trajectory_tsamp.type = visualization_msgs::Marker::SPHERE_LIST;
    trajectory_tsamp.action = visualization_msgs::Marker::ADD;
    trajectory_tsamp.pose.orientation.w = 1.0;
    trajectory_tsamp.scale.x = 0.05;
    trajectory_tsamp.scale.y = 0.05;
    trajectory_tsamp.scale.z = 0.05;
    trajectory_tsamp.color.r = 0.0;
    trajectory_tsamp.color.g = 0.0;
    trajectory_tsamp.color.b = 0.0;
    trajectory_tsamp.color.a = 1;

    tsamp_cnt = trajectory_data[k].size();
    for(int p = 0; p < tsamp_cnt; p++)
    {
      visualization_msgs::Marker tsamp_arrow;
      if (flag_kinematic_)
      {
        // SET SAMPLING ARROWS VISUALIZATION SETTINGS
              
        tsamp_arrow.id = k*tsamp_cnt+p;
        tsamp_arrow.ns = "tsa_" + to_string(tsamp_arrow.id);
        tsamp_arrow.header.frame_id = trajectory_frame_;
        tsamp_arrow.type = visualization_msgs::Marker::ARROW;
        tsamp_arrow.action = visualization_msgs::Marker::ADD;
        
        tsamp_arrow.pose.position.x = trajectory_data[k][p].x;
        tsamp_arrow.pose.position.y = trajectory_data[k][p].y;
        tsamp_arrow.pose.position.z = 0.0;
        
        tf2::Quaternion tsamp_quat = getQuaternionFromRPY(0.0, 0.0, trajectory_data[k][p].z);
        tsamp_arrow.pose.orientation.x = tsamp_quat.x();
        tsamp_arrow.pose.orientation.y = tsamp_quat.y();
        tsamp_arrow.pose.orientation.z = tsamp_quat.z();
        tsamp_arrow.pose.orientation.w = tsamp_quat.w();
        
        tsamp_arrow.scale.x = 0.1;
        tsamp_arrow.scale.y = 0.02;
        tsamp_arrow.scale.z = 0.05;
        tsamp_arrow.color.r = 0.0;
        tsamp_arrow.color.g = 0.0;
        tsamp_arrow.color.b = 1.0;
        tsamp_arrow.color.a = 1;

        /// NUA NOTE: WHEN TRAJECTORY IS LINEMATICALLY CREATED WITH SIMPLE CAR MODEL, Z IS YAW! 
        trajectory_data[k][p].z = 0;
      }
      
      if (tsamp_cnt > 1)
      {
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
      }
      else
      {
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
        trajectory_line_strip.points.push_back(trajectory_data[k][p]);
      }

      trajectory_tsamp.points.push_back(trajectory_data[k][p]);

      if (flag_kinematic_)
      {
        trajectory_sampling_arrow_visu_.markers.push_back(tsamp_arrow);
      }
    }
    trajectory_visu_.markers.push_back(trajectory_line_strip);
    trajectory_sampling_visu_.markers.push_back(trajectory_tsamp);
  }

  std::cout << "[TrajectorySamplingUtility::fill_trajectory_sampling_visu] trajectory_sampling_arrow_visu_ size: " << trajectory_sampling_arrow_visu_.markers.size() << std::endl;
  std::cout << "[TrajectorySamplingUtility::fill_trajectory_sampling_visu] trajectory_visu_ size: " << trajectory_visu_.markers.size() << std::endl;
  std::cout << "[TrajectorySamplingUtility::fill_trajectory_sampling_visu] trajectory_sampling_visu_ size: " << trajectory_sampling_visu_.markers.size() << std::endl;

  // --------

  vector<geometry_msgs::Pose> sampling_data_pose = sampling_data_pose_;
  sampling_arrow_visu_.markers.clear();

  std::cout << "[TrajectorySamplingUtility::fill_trajectory_sampling_visu] sampling_data_pose size: " << sampling_data_pose.size() << std::endl;

  for (size_t i = 0; i < sampling_data_pose.size(); i++)
  {
    visualization_msgs::Marker samp_arrow_visu;
    samp_arrow_visu.ns = "sp_" + to_string(i);
    samp_arrow_visu.id = i;
    samp_arrow_visu.header.frame_id = trajectory_frame_;
    samp_arrow_visu.type = visualization_msgs::Marker::ARROW;
    samp_arrow_visu.action = visualization_msgs::Marker::ADD;
    
    samp_arrow_visu.pose = sampling_data_pose[i];

    samp_arrow_visu.scale.x = 0.2;
    samp_arrow_visu.scale.y = 0.02;
    samp_arrow_visu.scale.z = 0.05;
    samp_arrow_visu.color.r = 0.0;
    samp_arrow_visu.color.g = 0.0;
    samp_arrow_visu.color.b = 1.0;
    samp_arrow_visu.color.a = 1;

    sampling_arrow_visu_.markers.push_back(samp_arrow_visu);
  }

  std::cout << "[trajectory_sampling_utility::fill_trajectory_sampling_visu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::publishFrame(string origin_frame_name, string frame_name, geometry_msgs::Pose frame_pose)
{
  //std::cout << "[TrajectorySamplingUtility::publishFrame] START" << std::endl;

  static tf::TransformBroadcaster br;
  tf::Transform tf_wrt_origin;

  string frame_name_tmp = frame_name;
  if (ns_ != "")
  {
    frame_name_tmp = ns_ + "/" + frame_name_tmp;
  }

  tf_wrt_origin.setOrigin(tf::Vector3(frame_pose.position.x, frame_pose.position.y, frame_pose.position.z));
  tf_wrt_origin.setRotation(tf::Quaternion(frame_pose.orientation.x, frame_pose.orientation.y, frame_pose.orientation.z, frame_pose.orientation.w));
  br.sendTransform(tf::StampedTransform(tf_wrt_origin, ros::Time::now(), origin_frame_name, frame_name_tmp));

  //std::cout << "[TrajectorySamplingUtility::publishFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::publishFrame(string origin_frame_name, vector<vector<geometry_msgs::Point>> frame_trajectory_point_vec)
{
  //std::cout << "[TrajectorySamplingUtility::publishFrame] START" << std::endl;

  //static tf::TransformBroadcaster br;
  
  for (size_t i = 0; i < frame_trajectory_point_vec.size(); i++)
  {
    for (size_t j = 0; j < frame_trajectory_point_vec[i].size(); j++)
    {
      int idx =  i * frame_trajectory_point_vec[i].size() + j;
      string frame_name = "tp_" + to_string(idx);
      if (ns_ != "")
      {
        frame_name = ns_ + "/" + frame_name;
      }

      //std::cout << "[TrajectorySamplingUtility::publishFrame] frame_name: " << frame_name << std::endl;

      double yaw = frame_trajectory_point_vec[i][j].z;
      tf2::Quaternion quat = getQuaternionFromRPY(0.0, 0.0, yaw);

      /*
      std::cout << "[TrajectorySamplingUtility::publishFrame] i -> " << i << std::endl;
      std::cout << "position: " << frame_pose_vec[i].position.x << ", " << frame_pose_vec[i].position.y << ", " << frame_pose_vec[i].position.z << std::endl;
      std::cout << "orientation (rpy): " << roll*180/PI << ", " << pitch*180/PI << ", " << yaw*180/PI << std::endl;
      std::cout << "orientation (quaternion): " << frame_pose_vec[i].orientation.x << ", " 
                                  << frame_pose_vec[i].orientation.y << ", " 
                                  << frame_pose_vec[i].orientation.z << ", " 
                                  << frame_pose_vec[i].orientation.w << std::endl;
      std::cout << "" << std::endl;
      */

      tf::Transform tf_wrt_origin;
      tf_wrt_origin.setOrigin(tf::Vector3(frame_trajectory_point_vec[i][j].x, frame_trajectory_point_vec[i][j].y, 0.0));
      tf_wrt_origin.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
      br_.sendTransform(tf::StampedTransform(tf_wrt_origin, ros::Time::now(), origin_frame_name, frame_name));
    }
  }

  //std::cout << "[TrajectorySamplingUtility::publishFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::publishFrame(string origin_frame_name, vector<geometry_msgs::Pose> frame_pose_vec)
{
  //std::cout << "[TrajectorySamplingUtility::publishFrame] START" << std::endl;

  //static tf::TransformBroadcaster br;
  
  for (size_t i = 0; i < frame_pose_vec.size(); i++)
  {
    string frame_name = "s_" + to_string(i);
    if (ns_ != "")
    {
      frame_name = ns_ + "/" + frame_name;
    }

    tf::Transform tf_wrt_origin;

    double roll, pitch, yaw;
    tf2::Quaternion quat(frame_pose_vec[i].orientation.x, frame_pose_vec[i].orientation.y, frame_pose_vec[i].orientation.z, frame_pose_vec[i].orientation.w); 
    getRPYFromQuaternion(quat, roll, pitch, yaw);

    /*
    std::cout << "[TrajectorySamplingUtility::publishFrame] i -> " << i << std::endl;
    std::cout << "position: " << frame_pose_vec[i].position.x << ", " << frame_pose_vec[i].position.y << ", " << frame_pose_vec[i].position.z << std::endl;
    std::cout << "orientation (rpy): " << roll*180/PI << ", " << pitch*180/PI << ", " << yaw*180/PI << std::endl;
    std::cout << "orientation (quaternion): " << frame_pose_vec[i].orientation.x << ", " 
                                 << frame_pose_vec[i].orientation.y << ", " 
                                 << frame_pose_vec[i].orientation.z << ", " 
                                 << frame_pose_vec[i].orientation.w << std::endl;
    std::cout << "" << std::endl;
    */

    tf_wrt_origin.setOrigin(tf::Vector3(frame_pose_vec[i].position.x, frame_pose_vec[i].position.y, frame_pose_vec[i].position.z));
    tf_wrt_origin.setRotation(tf::Quaternion(frame_pose_vec[i].orientation.x, frame_pose_vec[i].orientation.y, frame_pose_vec[i].orientation.z, frame_pose_vec[i].orientation.w));
    br_.sendTransform(tf::StampedTransform(tf_wrt_origin, ros::Time::now(), origin_frame_name, frame_name));
  }

  //std::cout << "[TrajectorySamplingUtility::publishFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::publish_trajectory_sampling(bool flag_publish_frame)
{
  for(int k = 0; k < trajectory_visu_.markers.size(); k++)
  {
    // UPDATE SEQUENCE AND STAMP FOR TRAJECTORY
    //trajectory_visu_.markers[k].header.seq++;
    //trajectory_visu_.markers[k].header.stamp = ros::Time(0);
    trajectory_visu_.markers[k].header.stamp = ros::Time::now();

    // UPDATE SEQUENCE AND STAMP FOR SAMPLING POINTS ON TRAJECTORY 
    //trajectory_sampling_visu_.markers[k].header.seq++;
    //trajectory_sampling_visu_.markers[k].header.stamp = ros::Time(0);
    trajectory_sampling_visu_.markers[k].header.stamp = ros::Time::now();
  
    trajectory_visu_pub_.publish(trajectory_visu_);
    trajectory_sampling_visu_pub_.publish(trajectory_sampling_visu_);

    if (flag_kinematic_)
    {
      trajectory_sampling_arrow_visu_pub_.publish(trajectory_sampling_arrow_visu_);
    }
  }

  //cout << "[TrajectorySamplingUtility::fill_trajectory_sampling_visu] sampling_arrow_visu_ size: " << sampling_arrow_visu_.markers.size() << std::endl;

  for (size_t i = 0; i < sampling_arrow_visu_.markers.size(); i++)
  {  
    sampling_arrow_visu_.markers[i].header.stamp = ros::Time::now();
    sampling_arrow_visu_pub_.publish(sampling_arrow_visu_);
  }

  //geometry_msgs::Pose base_link_pose;
  //base_link_pose.position.x = 0.0;
  //base_link_pose.position.y = 0.0;
  //base_link_pose.position.z = 0.0;
  //base_link_pose.orientation.x = 0.0;
  //base_link_pose.orientation.y = 0.0;
  //base_link_pose.orientation.z = 0.0;
  //base_link_pose.orientation.w = 1.0;
  //publishFrame(trajectory_frame_, "base_link", base_link_pose);

  /*
  if (trajectory_data_.size() > 0)
  {
    publishFrame(trajectory_frame_, trajectory_data_);
  }
  */

  if (flag_publish_frame && sampling_data_pose_.size() > 0)
  {
    publishFrame(trajectory_frame_, sampling_data_pose_);
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::create_trajectory_data_path()
{
  if (trajectory_sampling_dataset_path_ == "")
  {
    std::cout << "[TrajectorySamplingUtility::create_trajectory_data_path] trajectory_sampling_dataset_path is not defined!" << std::endl;
  }
  else
  {
    string trajectory_data_tag = createFileName();
    trajectory_data_path_ = trajectory_sampling_dataset_path_ + trajectory_data_tag + "/";
    boost::filesystem::create_directories(ws_path_ + trajectory_data_path_);
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::save_input_data()
{
  if (trajectory_data_path_ == "")
  {
    create_trajectory_data_path();
  }

  string input_data_path = ws_path_ + trajectory_data_path_ + "input_data.csv";
  std::cout << "[TrajectorySamplingUtility::save_input_data] input_data_path: " << input_data_path << std::endl;

  ofstream input_data_stream;
  input_data_stream.open(input_data_path);
  input_data_stream << "ws_name" << "," << ws_name_ << "\n";
  input_data_stream << "trajectory_name" << "," << trajectory_name_ << "\n";
  input_data_stream << "trajectory_data_path" << "," << trajectory_data_path_ << "\n";
  input_data_stream << "trajectory_sampling_dataset_path" << "," << trajectory_sampling_dataset_path_ << "\n";
  input_data_stream << "trajectory_frame" << "," << trajectory_frame_ << "\n";
  input_data_stream << "trajectory_gen_type" << "," << trajectory_generation_type_ << "\n";
  input_data_stream << "geo_type" << "," << geo_type_ << "\n";

  if (trajectory_generation_type_ == "geometric")
  {
    if (geo_type_ == "cone")
    {
      // GEOMETRIC: CONE
      input_data_stream << "trajectory_length" << "," << trajectory_length_ << "\n";
      input_data_stream << "trajectory_yaw" << "," << trajectory_yaw_ << "\n";
      input_data_stream << "trajectory_pitch" << "," << trajectory_pitch_ << "\n";
      input_data_stream << "trajectory_yaw_sampling_count" << "," << trajectory_yaw_sampling_count_ << "\n";
      input_data_stream << "trajectory_pitch_sampling_count" << "," << trajectory_pitch_sampling_count_ << "\n";
      input_data_stream << "trajectory_yaw_sampling_type" << "," << trajectory_yaw_sampling_type_ << "\n";
      input_data_stream << "trajectory_pitch_sampling_type" << "," << trajectory_pitch_sampling_type_ << "\n";
      input_data_stream << "trajectory_sampling_count" << "," << trajectory_sampling_count_ << "\n";
    }
    else if (geo_type_ == "cube")
    {
      // GEOMETRIC: CUBE
      input_data_stream << "sampling_x_min" << "," << sampling_x_min_ << "\n";
      input_data_stream << "sampling_x_max" << "," << sampling_x_max_ << "\n";
      input_data_stream << "sampling_x_cnt" << "," << sampling_x_cnt_ << "\n";
      input_data_stream << "sampling_y_min" << "," << sampling_y_min_ << "\n";
      input_data_stream << "sampling_y_max" << "," << sampling_y_max_ << "\n";
      input_data_stream << "sampling_y_cnt" << "," << sampling_y_cnt_ << "\n";
      input_data_stream << "sampling_z_min" << "," << sampling_z_min_ << "\n";
      input_data_stream << "sampling_z_max" << "," << sampling_z_max_ << "\n";
      input_data_stream << "sampling_z_cnt" << "," << sampling_z_cnt_ << "\n";
      input_data_stream << "sampling_roll_min" << "," << sampling_roll_min_ << "\n";
      input_data_stream << "sampling_roll_max" << "," << sampling_roll_max_ << "\n";
      input_data_stream << "sampling_roll_cnt" << "," << sampling_roll_cnt_ << "\n";
      input_data_stream << "sampling_pitch_min" << "," << sampling_pitch_min_ << "\n";
      input_data_stream << "sampling_pitch_max" << "," << sampling_pitch_max_ << "\n";
      input_data_stream << "sampling_pitch_cnt" << "," << sampling_pitch_cnt_ << "\n";
      input_data_stream << "sampling_yaw_min" << "," << sampling_yaw_min_ << "\n";
      input_data_stream << "sampling_yaw_max" << "," << sampling_yaw_max_ << "\n";
      input_data_stream << "sampling_yaw_cnt" << "," << sampling_yaw_cnt_ << "\n";
    }
  }
  else if (trajectory_generation_type_ == "kinematic")
  {
    // KINEMATIC
    input_data_stream << "trajectory_time" << "," << trajectory_time_ << "\n";
    input_data_stream << "dt" << "," << dt_ << "\n";
    input_data_stream << "lateral_velocity_sampling_count" << "," << lateral_velocity_sampling_count_ << "\n";
    input_data_stream << "angular_velocity_sampling_count" << "," << angular_velocity_sampling_count_ << "\n";
    input_data_stream << "robot_min_lat_velo" << "," << robot_min_lat_velo_ << "\n";
    input_data_stream << "robot_max_lat_velo" << "," << robot_max_lat_velo_ << "\n";
    input_data_stream << "robot_max_yaw_velo" << "," << robot_max_yaw_velo_ << "\n";
  }  
  
  input_data_stream.close();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::save_input_data(std::string data_field, std::string data)
{
  //std::cout << "[TrajectorySamplingUtility::save_input_data(2)] START" << std::endl;

  string input_data_path = ws_path_ + trajectory_data_path_ + "input_data.csv";
  //std::cout << "[TrajectorySamplingUtility::save_input_data(2)] input_data_path: " << input_data_path << std::endl;

  ofstream input_data_stream;
  input_data_stream.open(input_data_path, std::ios::app);
  
  if (input_data_stream.is_open())
  {
    input_data_stream << data_field << "," << data << "\n";
    input_data_stream.close();
  }
  else
  {
    std::cout << "[TrajectorySamplingUtility::save_input_data(2)] ERROR: Unable to open file: " << input_data_path << std::endl;
  }
  
  //std::cout << "[TrajectorySamplingUtility::save_input_data(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::save_trajectory_data(bool flag_append, bool flag_save_input)
{
  if (trajectory_data_path_ == "")
  {
    create_trajectory_data_path();
  }

  if (trajectory_data_.size() > 0)
  {
    ofstream trajectory_data_stream;
    if (flag_append)
    {
      trajectory_data_stream.open(ws_path_ + trajectory_data_path_ + "trajectory_data.csv", std::ios::app);
    }
    else
    {
      trajectory_data_stream.open(ws_path_ + trajectory_data_path_ + "trajectory_data.csv");
    }

    for (int i = 0; i < trajectory_data_.size(); ++i)
    {
      for (int j = 0; j < trajectory_data_[i].size(); ++j)
      {
        trajectory_data_stream << to_string(trajectory_data_[i][j].x) + " " + to_string(trajectory_data_[i][j].y) + " " + to_string(trajectory_data_[i][j].z) + ",";
      }
      trajectory_data_stream << "\n";
    }
    trajectory_data_stream.close();
  }
  
  if (sampling_data_pose_.size() > 0)
  {
    ofstream data_stream;
    
    if (flag_append)
    {
      data_stream.open(ws_path_ + trajectory_data_path_ + "sampling_data.csv", std::ios::app);
    }
    else
    {
      data_stream.open(ws_path_ + trajectory_data_path_ + "sampling_data.csv");
    }

    double roll, pitch, yaw;
    for (int i = 0; i < sampling_data_pose_.size(); ++i)
    {
      tf2::Quaternion quat(sampling_data_pose_[i].orientation.x, sampling_data_pose_[i].orientation.y, sampling_data_pose_[i].orientation.z, sampling_data_pose_[i].orientation.w); 
      getRPYFromQuaternion(quat, roll, pitch, yaw);
      data_stream << to_string(sampling_data_pose_[i].position.x) + " " + to_string(sampling_data_pose_[i].position.y) + " " + to_string(sampling_data_pose_[i].position.z) + " " + 
                     to_string(roll) + " " + to_string(pitch) + " " + to_string(yaw) + ",";

      data_stream << "\n";
    }
    data_stream.close();
  }

  if (flag_save_input)
  {
    save_input_data();
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TrajectorySamplingUtility::copy_data(std::string copy_from, std::string copy_to, std::string copy_data_type, std::string copy_data_field)
{
  std::cout << "[TrajectorySamplingUtility::copy_data] START" << std::endl;
  
  std::cout << "[TrajectorySamplingUtility::copy_data] copy_from: " << copy_from << std::endl;
  std::cout << "[TrajectorySamplingUtility::copy_data] copy_to: " << copy_to << std::endl;
  std::cout << "[TrajectorySamplingUtility::copy_data] copy_data_type: " << copy_data_type << std::endl;
  std::cout << "[TrajectorySamplingUtility::copy_data] copy_data_field: " << copy_data_field << std::endl;

  read_input_data(copy_to);

  //trajectory_data_path_ = copy_to;

  std::cout << "[TrajectorySamplingUtility::copy_data] trajectory_data_path_: " << trajectory_data_path_ << std::endl;

  if (copy_data_type == "trajectory_data")
  {
    vector<vector<geometry_msgs::Point>> trajectory_data_from;
    read_trajectory_data(copy_from, trajectory_data_from);
    read_trajectory_data(copy_to);

    std::cout << "[TrajectorySamplingUtility::copy_data] NUA NOTE: NOT IMPLEMENTED YET!" << std::endl;
  }
  else if (copy_data_type == "sampling_data_pose")
  {
    vector<geometry_msgs::Pose> sampling_data_pose_from;
    read_sampling_data(copy_from, sampling_data_pose_from);
    read_sampling_data(copy_to);

    std::cout << "[TrajectorySamplingUtility::copy_data] sampling_data_pose_from.size(): " << sampling_data_pose_from.size() << std::endl;
    std::cout << "[TrajectorySamplingUtility::copy_data] sampling_data_pose_.size(): " << sampling_data_pose_.size() << std::endl;

    if (sampling_data_pose_from.size() == sampling_data_pose_.size())
    {
      for (size_t i = 0; i < sampling_data_pose_from.size(); i++)
      {
        if (copy_data_field == "position")
        {
          sampling_data_pose_[i].position.x = sampling_data_pose_from[i].position.x;
          sampling_data_pose_[i].position.y = sampling_data_pose_from[i].position.y;
          sampling_data_pose_[i].position.z = sampling_data_pose_from[i].position.z;
        }
        else if (copy_data_field == "orientation")
        {
          sampling_data_pose_[i].orientation.x = sampling_data_pose_from[i].orientation.x;
          sampling_data_pose_[i].orientation.y = sampling_data_pose_from[i].orientation.y;
          sampling_data_pose_[i].orientation.z = sampling_data_pose_from[i].orientation.z;
          sampling_data_pose_[i].orientation.w = sampling_data_pose_from[i].orientation.w;
        }
        else
        {
          sampling_data_pose_[i]= sampling_data_pose_from[i];
        }
      }

      save_trajectory_data(false, false);
      save_input_data("copy_from", copy_from);
    }
    else
    {
      std::cout << "[TrajectorySamplingUtility::copy_data] ERROR: Size mismatch between copy_from and copy_to data!" << std::endl;
    }    
  }
  else
  {
    std::cout << "[TrajectorySamplingUtility::copy_data] ERROR: Invalid copy_data_type!" << std::endl;
  }

  std::cout << "[TrajectorySamplingUtility::copy_data] END" << std::endl;
}