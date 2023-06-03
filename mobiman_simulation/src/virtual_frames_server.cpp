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

using namespace std;

int main(int argc, char** argv)
{
  cout << "[virtual_frames_server::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "virtual_frames_server");
  
  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE AND SET PARAMETERS
  string world_frame_name = "world";
  string origin_frame_name = "base_link";
  string gz_model_msg = "/gazebo/model_states";

  std::vector<string> virtual_frame_names = {"virtual_world_link", "x_axis_link", "y_axis_link", "theta_link"};

  // Initialize Map Utility
  MapUtility mu;
  mu.setWorldFrameName(world_frame_name);

  ros::Rate r(100);
  while(ros::ok)
  {
    mu.publishVirtualFrames(virtual_frame_names, origin_frame_name);

    ros::spinOnce();

    r.sleep();
  }

  cout << "[virtual_frames_server::main] END" << endl;

  return 0;
}