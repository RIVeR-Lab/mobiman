// LAST UPDATE: 2024.03.12
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

// --CUSTOM LIBRARIES--
#include "mobiman_simulation/trajectory_sampling_utility.h"

int main(int argc, char** argv)
{
  // INITIALIZE ROS
  ros::init(argc, argv, "trajectory_sampling_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener* listener = new tf::TransformListener;

  // Get config parameters
  std::vector<std::string> traj_data_path_multi;
  string ws_name, copy_from, copy_to, copy_data_type, copy_data_field, traj_name, traj_data_path, robot_frame_name;
  bool flag_save, flag_copy, flag_read_only;
  pnh.param<string>("/ws_name", ws_name, "");
  pnh.param<bool>("/flag_save", flag_save, false);
  pnh.param<bool>("/flag_copy", flag_copy, false);
  pnh.param<string>("/copy_from", copy_from, "");
  pnh.param<string>("/copy_to", copy_to, "");
  pnh.param<string>("/copy_data_type", copy_data_type, "");
  pnh.param<string>("/copy_data_field", copy_data_field, "");
  pnh.param<vector<string>>("/trajectory_data_path_multi", traj_data_path_multi, {""});

  TrajectorySamplingUtility tsu(nh);
  tsu.set_workspace_name(ws_name);

  if (flag_copy)
  {
    tsu.copy_data(copy_from, copy_to, copy_data_type, copy_data_field);
  }
  else
  {
    pnh.param<string>("/trajectory_name", traj_name, "");
  
    pnh.param<string>("/trajectory_data_path", traj_data_path, "");
    pnh.param<bool>("/flag_read_only", flag_read_only, true);
    pnh.param<string>("/tss_robot_frame_name", robot_frame_name, "");

    cout << "[trajectory_sampling_server::main] trajectory_data_path: " << traj_data_path << endl;
    cout << "[trajectory_sampling_server::main] flag_read_only: " << flag_read_only << endl;
    cout << "[trajectory_sampling_server::main] robot_frame_name: " << robot_frame_name << endl;

    double dt, ttime, tlen, tyaw, tpitch, min_lat_velo, max_lat_velo, max_yaw_velo, 
            sampling_x_min, sampling_x_max,
            sampling_y_min, sampling_y_max,
            sampling_z_min, sampling_z_max,
            sampling_roll_min, sampling_roll_max,
            sampling_pitch_min, sampling_pitch_max,
            sampling_yaw_min, sampling_yaw_max;
    int tsamp_cnt, tyaw_cnt, tpitch_cnt, lat_velo_samp_cnt, ang_velo_samp_cnt, sample_start_index,
        sampling_x_cnt, sampling_y_cnt, sampling_z_cnt,
        sampling_roll_cnt, sampling_pitch_cnt, sampling_yaw_cnt;
    string traj_samp_dataset_path, trajectory_gen_type, geo_type;

    std::string ns = nh.getNamespace().substr(1);
    if (ns != "")
    {
      robot_frame_name = ns + "/" + robot_frame_name;
    }

    tsu.set_trajectory_name(traj_name);
    tsu.set_trajectory_frame(robot_frame_name);

    if (traj_data_path != "" && flag_read_only)
    {
      tsu.read_trajectory_data(traj_data_path);
      tsu.read_sampling_data(traj_data_path);
      tsu.read_velocity_control_data(traj_data_path);
    }
    else if (traj_data_path_multi.size() > 0 && flag_read_only)
    {
      //tsu.read_trajectory_data(traj_data_path_multi);
      tsu.read_sampling_data(traj_data_path_multi);
      //tsu.read_velocity_control_data(traj_data_path_multi);
    }
    else
    {
      pnh.param<string>("/trajectory_sampling_dataset_path", traj_samp_dataset_path, "");
      pnh.param<string>("/trajectory_gen_type", trajectory_gen_type, "");

      tsu.set_trajectory_data_path(traj_data_path);
      tsu.set_trajectory_sampling_dataset_path(traj_samp_dataset_path);
      tsu.set_trajectory_generation_type(trajectory_gen_type);

      if (trajectory_gen_type == "geometric")
      {
        pnh.param<string>("/geo_type", geo_type, "");
        pnh.param<double>("/tlen", tlen, 0);
        pnh.param<int>("/tsamp_cnt", tsamp_cnt, 0);
        pnh.param<double>("/tyaw", tyaw, 0.0);
        pnh.param<double>("/tpitch", tpitch, 0.0);
        pnh.param<int>("/tyaw_cnt", tyaw_cnt, 0);
        pnh.param<int>("/tpitch_cnt", tpitch_cnt, 0);

        if (geo_type == "cone")
        {
          tsu.set_trajectory_sampling_count(tsamp_cnt);
          tsu.set_trajectory_length(tlen);
          tsu.set_trajectory_yaw(tyaw);
          tsu.set_trajectory_pitch(tpitch);
          tsu.set_trajectory_yaw_sampling_count(tyaw_cnt);
          tsu.set_trajectory_pitch_sampling_count(tpitch_cnt);

          tsu.construct_trajectory_data_by_geometry_cone();
        }
        else if (geo_type == "cube")
        {
          pnh.param<double>("/sampling_x_min", sampling_x_min, 0.0);
          pnh.param<double>("/sampling_x_max", sampling_x_max, 0.0);
          pnh.param<int>("/sampling_x_cnt", sampling_x_cnt, 0);
          pnh.param<double>("/sampling_y_min", sampling_y_min, 0.0);
          pnh.param<double>("/sampling_y_max", sampling_y_max, 0.0);
          pnh.param<int>("/sampling_y_cnt", sampling_y_cnt, 0);
          pnh.param<double>("/sampling_z_min", sampling_z_min, 0.0);
          pnh.param<double>("/sampling_z_max", sampling_z_max, 0.0);
          pnh.param<int>("/sampling_z_cnt", sampling_z_cnt, 0);
          pnh.param<double>("/sampling_roll_min", sampling_roll_min, 0.0);
          pnh.param<double>("/sampling_roll_max", sampling_roll_max, 0.0);
          pnh.param<int>("/sampling_roll_cnt", sampling_roll_cnt, 0);
          pnh.param<double>("/sampling_pitch_min", sampling_pitch_min, 0.0);
          pnh.param<double>("/sampling_pitch_max", sampling_pitch_max, 0.0);
          pnh.param<int>("/sampling_pitch_cnt", sampling_pitch_cnt, 0);
          pnh.param<double>("/sampling_yaw_min", sampling_yaw_min, 0.0);
          pnh.param<double>("/sampling_yaw_max", sampling_yaw_max, 0.0);
          pnh.param<int>("/sampling_yaw_cnt", sampling_yaw_cnt, 0);
          pnh.param<int>("/sample_start_index", sample_start_index, 0);
          
          cout << "[trajectory_sampling_server::main] sampling_x_min: " << sampling_x_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_x_max: " << sampling_x_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_x_cnt: " << sampling_x_cnt << endl;
          cout << "[trajectory_sampling_server::main] sampling_y_min: " << sampling_y_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_y_max: " << sampling_y_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_y_cnt: " << sampling_y_cnt << endl;
          cout << "[trajectory_sampling_server::main] sampling_z_min: " << sampling_z_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_z_max: " << sampling_z_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_z_cnt: " << sampling_z_cnt << endl;
          cout << "[trajectory_sampling_server::main] sampling_roll_min: " << sampling_roll_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_roll_max: " << sampling_roll_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_roll_cnt: " << sampling_roll_cnt << endl;
          cout << "[trajectory_sampling_server::main] sampling_pitch_min: " << sampling_pitch_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_pitch_max: " << sampling_pitch_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_pitch_cnt: " << sampling_pitch_cnt << endl;
          cout << "[trajectory_sampling_server::main] sampling_yaw_min: " << sampling_yaw_min << endl;
          cout << "[trajectory_sampling_server::main] sampling_yaw_max: " << sampling_yaw_max << endl;
          cout << "[trajectory_sampling_server::main] sampling_yaw_cnt: " << sampling_yaw_cnt << endl;

          tsu.set_sampling_x_min(sampling_x_min);
          tsu.set_sampling_x_max(sampling_x_max);
          tsu.set_sampling_x_cnt(sampling_x_cnt);
          tsu.set_sampling_y_min(sampling_y_min);
          tsu.set_sampling_y_max(sampling_y_max);
          tsu.set_sampling_y_cnt(sampling_y_cnt);
          tsu.set_sampling_z_min(sampling_z_min);
          tsu.set_sampling_z_max(sampling_z_max);
          tsu.set_sampling_z_cnt(sampling_z_cnt);
          tsu.set_sampling_roll_min(sampling_roll_min);
          tsu.set_sampling_roll_max(sampling_roll_max);
          tsu.set_sampling_roll_cnt(sampling_roll_cnt);
          tsu.set_sampling_pitch_min(sampling_pitch_min);
          tsu.set_sampling_pitch_max(sampling_pitch_max);
          tsu.set_sampling_pitch_cnt(sampling_pitch_cnt);
          tsu.set_sampling_yaw_min(sampling_yaw_min);
          tsu.set_sampling_yaw_max(sampling_yaw_max);
          tsu.set_sampling_yaw_cnt(sampling_yaw_cnt);

          tsu.construct_trajectory_data_by_geometry_cube(sample_start_index);
        }
        else
        {
          cout << "[trajectory_sampling_server::main] ERROR: Invalid geo_type!" << endl;
        }
      }
      else if (trajectory_gen_type == "kinematic")
      {
        pnh.param<int>("/tsamp_cnt", tsamp_cnt, 0);
        pnh.param<double>("/dt", dt, 0);
        pnh.param<double>("/ttime", ttime, 0);
        pnh.param<int>("/lat_velo_samp_cnt", lat_velo_samp_cnt, 0);
        pnh.param<int>("/ang_velo_samp_cnt", ang_velo_samp_cnt, 0);
        pnh.param<double>("/min_lat_velo", min_lat_velo, 0.0);
        pnh.param<double>("/max_lat_velo", max_lat_velo, 0.0);
        pnh.param<double>("/max_yaw_velo", max_yaw_velo, 0.0);
        pnh.param<int>("/sample_start_index", sample_start_index, 0);

        cout << "[trajectory_sampling_server::main] tsamp_cnt: " << tsamp_cnt << endl;
        cout << "[trajectory_sampling_server::main] dt: " << dt << endl;
        cout << "[trajectory_sampling_server::main] ttime: " << ttime << endl;
        cout << "[trajectory_sampling_server::main] lat_velo_samp_cnt: " << lat_velo_samp_cnt << endl;
        cout << "[trajectory_sampling_server::main] ang_velo_samp_cnt: " << ang_velo_samp_cnt << endl;
        cout << "[trajectory_sampling_server::main] min_lat_velo: " << min_lat_velo << endl;
        cout << "[trajectory_sampling_server::main] max_lat_velo: " << max_lat_velo << endl;
        cout << "[trajectory_sampling_server::main] max_yaw_velo: " << max_yaw_velo << endl;
        cout << "[trajectory_sampling_server::main] sample_start_index: " << sample_start_index << endl;

        tsu.set_trajectory_sampling_count(tsamp_cnt);
        tsu.set_trajectory_time(ttime);
        tsu.set_lateral_velocity_sampling_count(lat_velo_samp_cnt);
        tsu.set_angular_velocity_sampling_count(ang_velo_samp_cnt);

        tsu.construct_trajectory_data_by_simple_car_model(min_lat_velo, max_lat_velo, max_yaw_velo, dt, sample_start_index);

        tsu.update_sampling_data_from_trajectory_data();
      }
      else
      {
          cout << "[trajectory_sampling_server::main] ERROR: trajectory_gen_type is not defined!" << endl;
      }
      
      tsu.fill_trajectory_sampling_visu();

      if (flag_save)
      {
        tsu.save_trajectory_data();
      }
    }
  }

  //ros::Rate r(100);
  while(ros::ok)
  {
    tsu.publish_trajectory_sampling();
  }

  ros::spin();
}
