// LAST UPDATE: 2023.02.03
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:

// --EXTERNAL LIBRARIES--
#include <chrono>

// --CUSTOM LIBRARIES--
#include <mobiman_simulation/map_utility.h>
#include <mobiman_simulation/getNearestOccDist.h>

int main(int argc, char** argv)
{
  cout << "[test_map_utility_getNearestOccupancyDist2::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "test_map_utility_getNearestOccupancyDist2");

  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");  

  // INITIALIZE AND SET PARAMETERS
  string world_frame_name, oct_msg_name, pub_name_oct_msg, pub_name_oct_dist_visu;
  double qp_x, qp_y, qp_z;

  pnh.param<string>("/world_frame_name", world_frame_name, "");
  pnh.param<string>("/oct_msg_name", oct_msg_name, "");
  pnh.param<string>("/pub_name_oct_msg", pub_name_oct_msg, "");
  pnh.param<string>("/pub_name_oct_dist_visu", pub_name_oct_dist_visu, "");
  pnh.param<double>("/qp_x", qp_x, 0.0);
  pnh.param<double>("/qp_y", qp_y, 0.0);
  pnh.param<double>("/qp_z", qp_z, 0.0);

  double distance = -1;
  geometry_msgs::Point qp;
  qp.x = qp_x;
  qp.y = qp_y;
  qp.z = qp_z;
  
  MapUtility mu;
  mu.setWorldFrameName(world_frame_name);
  mu.setPubOctMsg(pub_name_oct_msg);
  mu.setPubOctDistVisu(pub_name_oct_dist_visu);
  mu.updateOct(oct_msg_name);

  double duration, frequency;
  std::chrono::steady_clock::time_point timer_start, timer_end;

  ros::Rate r(100);
  while(ros::ok && !ros::isShuttingDown())
  {
    mu.publishOctMsg();

    timer_start = std::chrono::steady_clock::now();
    distance = mu.getNearestOccupancyDist2(qp.x, qp.y, qp.z);
    timer_end = std::chrono::steady_clock::now();

    duration = std::chrono::duration_cast<std::chrono::microseconds>(timer_end - timer_start).count();
    frequency = pow(10,6) / duration;

    std::cout << "[test_map_utility_getNearestOccupancyDist2::main] " << ": (" << qp.x << ", " << qp.y << ", " << qp.z << ") -> " << distance << std::endl;
    std::cout << "[test_map_utility_getNearestOccupancyDist2::main] Processing duration: " << duration << " [µs]" << std::endl;
    std::cout << "[test_map_utility_getNearestOccupancyDist2::main] Processing frequency: " << frequency << " [Hz]" << std::endl;
    std::cout << "" << std::endl;

    r.sleep();
    ros::spinOnce();
  }

  //ros::spin();

  cout << "[test_map_utility_getNearestOccupancyDist2::main] END" << endl;
  return 0;
}