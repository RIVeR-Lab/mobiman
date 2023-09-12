// LAST UPDATE: 2023.07.24
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
  string world_frame_name, gz_model_msg_name, egrid_frame_name;
  std::vector<string> name_pkgs_ign, name_pkgs_man, scan_data_path_pkgs_ign, scan_data_path_pkgs_man;
  double map_resolution, egrid_resolution, egrid_occ_threshold;
  geometry_msgs::Point egrid_bbx_min, egrid_bbx_max;

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
  pnh.param<double>("/map_resolution", map_resolution, 0.0);

  pnh.param<string>("/egrid_frame_name", egrid_frame_name, "");
  pnh.param<double>("/egrid_resolution", egrid_resolution, 0.0);
  pnh.param<double>("/egrid_bbx_min_x", egrid_bbx_min.x, 0.0);
  pnh.param<double>("/egrid_bbx_min_y", egrid_bbx_min.y, 0.0);
  pnh.param<double>("/egrid_bbx_min_z", egrid_bbx_min.z, 0.0);
  pnh.param<double>("/egrid_bbx_max_x", egrid_bbx_max.x, 0.0);
  pnh.param<double>("/egrid_bbx_max_y", egrid_bbx_max.y, 0.0);
  pnh.param<double>("/egrid_bbx_max_z", egrid_bbx_max.z, 0.0);
  pnh.param<double>("/egrid_occ_threshold", egrid_occ_threshold, 0.0);
  
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
  cout << "[map_server::main] egrid_frame_name: " << egrid_frame_name << endl;
  cout << "[map_server::main] egrid_resolution: " << egrid_resolution << endl;
  cout << "[map_server::main] egrid_bbx_min_x: " << egrid_bbx_min.x << endl;
  cout << "[map_server::main] egrid_bbx_min_y: " << egrid_bbx_min.y << endl;
  cout << "[map_server::main] egrid_bbx_min_z: " << egrid_bbx_min.z << endl;
  cout << "[map_server::main] egrid_bbx_max_x: " << egrid_bbx_max.x << endl;
  cout << "[map_server::main] egrid_bbx_max_y: " << egrid_bbx_max.y << endl;
  cout << "[map_server::main] egrid_bbx_max_z: " << egrid_bbx_max.z << endl;
  cout << "[map_server::main] egrid_occ_threshold: " << egrid_occ_threshold << endl;

  // Initialize Scan Utility
  ScanUtility su(nh);

  // Get PointCloud2 and bbx-dim data
  vector<sensor_msgs::PointCloud2> pc2_msg_pkgs_ign(scan_data_path_pkgs_ign.size());
  vector<geometry_msgs::Point> obj_bbx_min_pkgs_ign;
  vector<geometry_msgs::Point> obj_bbx_max_pkgs_ign;
  vector<geometry_msgs::Point> obj_dim_pkgs_ign;

  for (size_t i = 0; i < scan_data_path_pkgs_ign.size(); i++)
  {
    su.getScanPointcloud2(scan_data_path_pkgs_ign[i], pc2_msg_pkgs_ign[i]);
    
    su.readObjBbxDim(scan_data_path_pkgs_ign[i]);
    obj_bbx_min_pkgs_ign.push_back(su.getObjBbxMin());
    obj_bbx_max_pkgs_ign.push_back(su.getObjBbxMax());
    obj_dim_pkgs_ign.push_back(su.getObjDim());
  }

  vector<sensor_msgs::PointCloud2> pc2_msg_pkgs_man(scan_data_path_pkgs_man.size());
  vector<geometry_msgs::Point> obj_bbx_min_pkgs_man;
  vector<geometry_msgs::Point> obj_bbx_max_pkgs_man;
  vector<geometry_msgs::Point> obj_dim_pkgs_man;

  for (size_t i = 0; i < scan_data_path_pkgs_man.size(); i++)
  {
    su.getScanPointcloud2(scan_data_path_pkgs_man[i], pc2_msg_pkgs_man[i]);
    
    su.readObjBbxDim(scan_data_path_pkgs_man[i]);
    obj_bbx_min_pkgs_man.push_back(su.getObjBbxMin());
    obj_bbx_max_pkgs_man.push_back(su.getObjBbxMax());
    obj_dim_pkgs_man.push_back(su.getObjDim());
  }

  // Initialize Map Utility
  MapUtility mu(nh, 
                pnh, 
                world_frame_name, 
                gz_model_msg_name, 
                name_pkgs_ign, 
                pc2_msg_pkgs_ign,
                obj_bbx_min_pkgs_ign, 
                obj_bbx_max_pkgs_ign, 
                obj_dim_pkgs_ign, 
                name_pkgs_man, 
                pc2_msg_pkgs_man, 
                obj_bbx_min_pkgs_man, 
                obj_bbx_max_pkgs_man, 
                obj_dim_pkgs_man,
                map_resolution);

  /*
  mu.initializeEgoGrid(egrid_frame_name, 
                       egrid_resolution, 
                       egrid_bbx_min, 
                       egrid_bbx_max,
                       egrid_occ_threshold);

  mu.fillEgoTargetPC();

  // Initialize Moveit collision objects
  mu.initializeMoveitCollisionObjects();
  */

  // Add moveit collision objects
  //mu.addMoveitCollisionObjects();

  ros::spinOnce();

  ros::Duration(2.0).sleep();

  ros::Rate r(100);
  while(ros::ok)
  {
    mu.updateModelPc2Scan();

    //cout << "[map_server::main] BEFORE updateOct" << endl;
    // Update octomap with the recent transformed pc2 data
    mu.updateOct();
    mu.updateObjectOct();

    //mu.updateEgoGrid();

    //mu.updateOccGrid();

    //mu.updateMoveitCollisionObjects();

    /// Publishers
    //cout << "[map_server::main] BEFORE publishPC2MsgGzScan" << endl;
    // Publish pc2 data of gazebo models
    mu.publishPC2MsgGzScan();

    //cout << "[map_server::main] BEFORE publishOctMsg" << endl;
    // Publish Octomap message
    mu.publishOctMsg();
    mu.publishObjectOctMsg();

    //mu.publishEgoGridPcMsg();
    //mu.publishEgoTargetPcMsg();
    //mu.publishEgoGridOccPcMsg();
    //mu.publishOccGridMsg();

    //cout << "[map_server::main] BEFORE publishMoveitCollisionObjects" << endl;
    // Publish moveit collision objects
    //mu.publishMoveitCollisionObjects();

    //cout << "[map_server::main] BEFORE spinOnce" << endl;
    ros::spinOnce();

    r.sleep();
  }

  // Service
  //ros::ServiceServer service = nh.advertiseService("get_nearest_occ_dist", &MapUtility::getNearestOccupancyDistSrv, &mu);

  //double map_server_dt = 0.1;
  //ros::Timer timer = nh.createTimer(ros::Duration(scan_server_dt), &MapUtility::mainCallback, &mu);

  cout << "[map_server::main] END" << endl;

  return 0;
}