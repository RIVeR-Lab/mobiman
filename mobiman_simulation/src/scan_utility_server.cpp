// LAST UPDATE: 2023.01.12
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:
// 1) Map Service is not working with main callback. Asynchronous spin is working but then map gets messy!

// --CUSTOM LIBRARIES--
#include "scan_utility.h"

int main(int argc, char** argv)
{
  cout << "scan_utility_server::main -> Map Utility Server is processing..." << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "scan_utility_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener* listener = new tf::TransformListener;

  // INITIALIZE AND SET PARAMETERS
  string world_frame_name, pc2_msg_name_sensor1, pc2_msg_name_sensor2, pc2_msg_name_sensor3, pc2_msg_name_sensor4;
  double bbx_x_min, bbx_x_max, bbx_y_min, bbx_y_max, bbx_z_min, bbx_z_max, oct_resolution;

  pnh.param<string>("/world_frame_name", world_frame_name, "");
  pnh.param<string>("/pc2_msg_name_sensor1", pc2_msg_name_sensor1, "");
  pnh.param<string>("/pc2_msg_name_sensor2", pc2_msg_name_sensor2, "");
  pnh.param<string>("/pc2_msg_name_sensor3", pc2_msg_name_sensor3, "");
  pnh.param<string>("/pc2_msg_name_sensor4", pc2_msg_name_sensor4, "");

  pnh.param("/bbx_x_min", bbx_x_min, 0.0);
  pnh.param("/bbx_x_max", bbx_x_max, 0.0);
  pnh.param("/bbx_y_min", bbx_y_min, 0.0);
  pnh.param("/bbx_y_max", bbx_y_max, 0.0);
  pnh.param("/bbx_z_min", bbx_z_min, 0.0);
  pnh.param("/bbx_z_max", bbx_z_max, 0.0);
  pnh.param("/oct_resolution", oct_resolution, 0.0);
  
  cout << "[scan_utility_server::main] world_frame_name: " << world_frame_name << endl;
  cout << "[scan_utility_server::main] pc2_msg_name_sensor1: " << pc2_msg_name_sensor1 << endl;
  cout << "[scan_utility_server::main] pc2_msg_name_sensor2: " << pc2_msg_name_sensor2 << endl;
  cout << "[scan_utility_server::main] pc2_msg_name_sensor3: " << pc2_msg_name_sensor3 << endl;
  cout << "[scan_utility_server::main] pc2_msg_name_sensor4: " << pc2_msg_name_sensor4 << endl;
  cout << "[scan_utility_server::main] bbx_x_min: " << bbx_x_min << endl;
  cout << "[scan_utility_server::main] bbx_x_max: " << bbx_x_max << endl;
  cout << "[scan_utility_server::main] bbx_y_min: " << bbx_y_min << endl;
  cout << "[scan_utility_server::main] bbx_y_max: " << bbx_y_max << endl;
  cout << "[scan_utility_server::main] bbx_z_min: " << bbx_z_min << endl;
  cout << "[scan_utility_server::main] bbx_z_max: " << bbx_z_max << endl;
  cout << "[scan_utility_server::main] oct_resolution: " << oct_resolution << endl;

  vector<string> pc2_msg_name_vec;
  if (pc2_msg_name_sensor1 != "")
  {
    pc2_msg_name_vec.push_back(pc2_msg_name_sensor1);
  }
  if (pc2_msg_name_sensor2 != "")
  {
    pc2_msg_name_vec.push_back(pc2_msg_name_sensor2);
  }
  if (pc2_msg_name_sensor3 != "")
  {
    pc2_msg_name_vec.push_back(pc2_msg_name_sensor3);
  }
  if (pc2_msg_name_sensor4 != "")
  {
    pc2_msg_name_vec.push_back(pc2_msg_name_sensor4);
  }

  // INITIALIZE AND SET MAP PARAMETERS
  ScanUtility su(nh,
                 world_frame_name,
                 pc2_msg_name_vec,
                 bbx_x_min,
                 bbx_x_max,
                 bbx_y_min,
                 bbx_y_max,
                 bbx_z_min,
                 bbx_z_max,
                 oct_resolution);

  ros::Duration(1.0).sleep();

  // MAP SERVER LOOP
  double scan_server_dt = 0.1;
  ros::Timer timer = nh.createTimer(ros::Duration(scan_server_dt), &ScanUtility::mainCallback, &su);

  ros::spin();

  return 0;
}