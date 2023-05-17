// LAST UPDATE: 2023.02.01
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --CUSTOM LIBRARIES--
#include <mobiman_simulation/scan_utility.h>
#include <mobiman_simulation/map_utility.h>
//#include <voxblox_ros/esdf_server.h>

int main(int argc, char** argv)
{
  cout << "[map_server::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "map_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener* listener = new tf::TransformListener;

  // INITIALIZE AND SET PARAMETERS
  string world_frame_name, gz_model_msg_name;
  std::vector<string> name_pkgs_ign, name_pkgs_man, scan_data_path_pkgs_ign, scan_data_path_pkgs_man;
  double map_resolution;

  pnh.param<string>("/world_frame_name", world_frame_name, "");
  pnh.param<string>("/gz_model_msg_name", gz_model_msg_name, "");
  if (!pnh.getParam("/name_pkgs_ign", name_pkgs_ign))
  {
    ROS_ERROR("Failed to get parameter from server.");
  }
  if (!pnh.getParam("/name_pkgs_man", name_pkgs_man))
  {
    ROS_ERROR("Failed to get parameter from server.");
  }
  if (!pnh.getParam("/scan_data_path_pkgs_ign", scan_data_path_pkgs_ign))
  {
    ROS_ERROR("Failed to get parameter from server.");
  }
  if (!pnh.getParam("/scan_data_path_pkgs_man", scan_data_path_pkgs_man))
  {
    ROS_ERROR("Failed to get parameter from server.");
  }
  pnh.param<double>("/map_resolution", map_resolution, 0.1);

  cout << "[map_server::main] world_frame_name: " << world_frame_name << endl;
  cout << "[map_server::main] gz_model_msg_name: " << gz_model_msg_name << endl;
  cout << "[map_server::main] name_pkgs_ign: " << endl;
  for (size_t i = 0; i < name_pkgs_ign.size(); i++)
  {
    cout << i << " -> " << name_pkgs_ign[i] << endl;
  }
  cout << "[map_server::main] name_pkgs_man: " << endl;
  for (size_t i = 0; i < name_pkgs_man.size(); i++)
  {
    cout << i << " -> " << name_pkgs_man[i] << endl;
  }
  cout << "[map_server::main] map_resolution: " << map_resolution << endl;

  // Initialize Scan Utility
  vector<sensor_msgs::PointCloud2> pc2_msg_pkgs_ign(scan_data_path_pkgs_ign.size());
  vector<sensor_msgs::PointCloud2> pc2_msg_pkgs_man(scan_data_path_pkgs_man.size());
  ScanUtility su(nh);
  for (size_t i = 0; i < scan_data_path_pkgs_ign.size(); i++)
  {
    su.getScanPointcloud2(scan_data_path_pkgs_ign[i], pc2_msg_pkgs_ign[i]);
  }
  for (size_t i = 0; i < scan_data_path_pkgs_man.size(); i++)
  {
    su.getScanPointcloud2(scan_data_path_pkgs_man[i], pc2_msg_pkgs_man[i]);
  }

  //voxblox::EsdfServer esdf_node(nh, pnh);

  // Initialize Map Utility
  MapUtility mu(nh, 
                pnh, 
                world_frame_name, 
                gz_model_msg_name, 
                name_pkgs_ign, 
                name_pkgs_man, 
                pc2_msg_pkgs_ign, 
                pc2_msg_pkgs_man, 
                map_resolution);

  // Initialize Moveit collision objects
  mu.initializeMoveitCollisionObjects();

  // Add moveit collision objects
  mu.addMoveitCollisionObjects();

  // Service
  //ros::ServiceServer service = nh.advertiseService("get_nearest_occ_dist", &MapUtility::getNearestOccupancyDistSrv, &mu);

  //ros::Duration(1.0).sleep();

  //double map_server_dt = 0.1;
  //ros::Timer timer = nh.createTimer(ros::Duration(scan_server_dt), &MapUtility::mainCallback, &mu);

  ros::spin();

  cout << "[map_server::main] END" << endl;

  return 0;
}