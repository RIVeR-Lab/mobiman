// LAST UPDATE: 2023.05.17
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

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
  double scan_bbx_x_min, scan_bbx_x_max, scan_bbx_y_min, scan_bbx_y_max, scan_bbx_z_min, scan_bbx_z_max, oct_resolution;

  pnh.param<string>("/obj_name", obj_name, "");
  pnh.param<string>("/data_dir", data_dir, "");
  pnh.param<string>("/world_frame_name", world_frame_name, "");
  pnh.param<string>("/pc2_msg_name_sensor1", pc2_msg_name_sensor1, "");
  pnh.param<string>("/pc2_msg_name_sensor2", pc2_msg_name_sensor2, "");
  pnh.param<string>("/pc2_msg_name_sensor3", pc2_msg_name_sensor3, "");
  pnh.param<string>("/pc2_msg_name_sensor4", pc2_msg_name_sensor4, "");
  pnh.param("/oct_resolution", oct_resolution, 0.0);
  pnh.param("/scan_bbx_x_min", scan_bbx_x_min, 0.0);
  pnh.param("/scan_bbx_x_max", scan_bbx_x_max, 0.0);
  pnh.param("/scan_bbx_y_min", scan_bbx_y_min, 0.0);
  pnh.param("/scan_bbx_y_max", scan_bbx_y_max, 0.0);
  pnh.param("/scan_bbx_z_min", scan_bbx_z_min, 0.0);
  pnh.param("/scan_bbx_z_max", scan_bbx_z_max, 0.0);
  
  cout << "[scanner::main] obj_name: " << obj_name << endl;
  cout << "[scanner::main] data_dir: " << data_dir << endl;
  cout << "[scanner::main] world_frame_name: " << world_frame_name << endl;
  cout << "[scanner::main] pc2_msg_name_sensor1: " << pc2_msg_name_sensor1 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor2: " << pc2_msg_name_sensor2 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor3: " << pc2_msg_name_sensor3 << endl;
  cout << "[scanner::main] pc2_msg_name_sensor4: " << pc2_msg_name_sensor4 << endl;
  cout << "[scanner::main] scan_bbx_x_min: " << scan_bbx_x_min << endl;
  cout << "[scanner::main] scan_bbx_x_max: " << scan_bbx_x_max << endl;
  cout << "[scanner::main] scan_bbx_y_min: " << scan_bbx_y_min << endl;
  cout << "[scanner::main] scan_bbx_y_max: " << scan_bbx_y_max << endl;
  cout << "[scanner::main] scan_bbx_z_min: " << scan_bbx_z_min << endl;
  cout << "[scanner::main] scan_bbx_z_max: " << scan_bbx_z_max << endl;
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
                 scan_bbx_x_min,
                 scan_bbx_x_max,
                 scan_bbx_y_min,
                 scan_bbx_y_max,
                 scan_bbx_z_min,
                 scan_bbx_z_max,
                 oct_resolution);

  ros::Duration(1.0).sleep();

  su.scanner();
  su.writePointcloud2Data();

  cout << "[scanner::main] END" << endl;

  ros::waitForShutdown();

  return 0;
}