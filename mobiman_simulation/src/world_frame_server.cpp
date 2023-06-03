// LAST UPDATE: 2023.05.23
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --CUSTOM LIBRARIES--
#include <mobiman_simulation/map_utility.h>

int main(int argc, char** argv)
{
  cout << "[world_frame_server::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "world_frame_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE AND SET PARAMETERS
  string world_frame_name = "world";
  string origin_frame_name = "base_link";
  string gz_model_msg = "/gazebo/model_states";
  
  // Initialize Map Utility
  MapUtility mu;
  mu.setWorldFrameName(world_frame_name);
  mu.initializeGazeboModelCallback(nh, gz_model_msg);

  ros::Rate r(10000);
  while(ros::ok)
  {
    mu.publishWorldFrame(world_frame_name, origin_frame_name);

    ros::spinOnce();

    r.sleep();
  }

  cout << "[world_frame_server::main] END" << endl;

  return 0;
}