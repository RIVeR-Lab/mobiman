// LAST UPDATE: 2023.02.03
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
#include "mobiman_simulation/scan_utility.h"

int main(int argc, char** argv)
{
  cout << "[scanner::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "scanner");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener* listener = new tf::TransformListener;

  // INITIALIZE AND SET PARAMETERS
  string obj_name, data_dir, world_frame_name, pc2_msg_name_sensor1, pc2_msg_name_sensor2, pc2_msg_name_sensor3, pc2_msg_name_sensor4;
  double bbx_x_min, bbx_x_max, bbx_y_min, bbx_y_max, bbx_z_min, bbx_z_max, oct_resolution;

  pnh.param<string>("/obj_name", obj_name, "");
  pnh.param<string>("/data_dir", data_dir, "");
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
  
  cout << "[scanner::main] obj_name: " << obj_name << endl;
  cout << "[scanner::main] data_dir: " << data_dir << endl;
  cout << "[scanner::main] world_frame_name: " << world_frame_name << endl;
  cout << "[scanner::main] pc2_msg_name_sensor1: " << pc2_msg_name_sensor1 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor2: " << pc2_msg_name_sensor2 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor3: " << pc2_msg_name_sensor3 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor4: " << pc2_msg_name_sensor4 << endl;
  cout << "[scanner::main] bbx_x_min: " << bbx_x_min << endl;
  cout << "[scanner::main] bbx_x_max: " << bbx_x_max << endl;
  cout << "[scanner::main] bbx_y_min: " << bbx_y_min << endl;
  cout << "[scanner::main] bbx_y_max: " << bbx_y_max << endl;
  cout << "[scanner::main] bbx_z_min: " << bbx_z_min << endl;
  cout << "[scanner::main] bbx_z_max: " << bbx_z_max << endl;
  cout << "[scanner::main] oct_resolution: " << oct_resolution << endl << endl;

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
                 obj_name,
                 data_dir,
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

  su.scanner();
  su.writePointcloud2Data();

  // MAP SERVER LOOP
  //double scan_server_dt = 0.1;
  //ros::Timer timer = nh.createTimer(ros::Duration(scan_server_dt), &ScanUtility::mainCallback, &su);

  //ros::Duration(1.0).sleep();

  /*
  ScanUtility su(nh, "/home/akmandor/mobiman_ws/src/mobiman/mobiman_simulation/models/point_cloud/longwide_pkg.json");

  while(ros::ok())
  {
    su.publishPC2Msg();
    ros::spinOnce();
  }
  */

  cout << "[scanner::main] END" << endl;

  ros::waitForShutdown();

  return 0;
}