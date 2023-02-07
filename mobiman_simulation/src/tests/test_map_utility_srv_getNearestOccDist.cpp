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
  cout << "[test_srv_getNearestOccDist::main] START" << endl;

  // INITIALIZE ROS
  ros::init(argc, argv, "test_srv_getNearestOccDist");

  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");  

  // INITIALIZE AND SET PARAMETERS
  double qp_x, qp_y, qp_z;

  pnh.param<double>("/qp_x", qp_x, 0.0);
  pnh.param<double>("/qp_y", qp_y, 0.0);
  pnh.param<double>("/qp_z", qp_z, 0.0);

  ros::ServiceClient occ_distance_client = nh.serviceClient<mobiman_simulation::getNearestOccDist>("get_nearest_occ_dist");;
  mobiman_simulation::getNearestOccDist occ_distance_srv; 

  double distance = -1;
  geometry_msgs::Point qp;
  qp.x = qp_x;
  qp.y = qp_y;
  qp.z = qp_z;

  double duration, frequency;
  std::chrono::steady_clock::time_point timer_start, timer_end;

  ros::Rate r(100);
  while(ros::ok && !ros::isShuttingDown())
  {
    occ_distance_srv.request.x = qp.x;
    occ_distance_srv.request.y = qp.y;
    occ_distance_srv.request.z = qp.z;

    timer_start = std::chrono::steady_clock::now();
    if (const_cast<ros::ServiceClient*>(&occ_distance_client)->call(occ_distance_srv))
    //if (occ_distance_client.call(occ_distance_srv))
    {
      
      distance = occ_distance_srv.response.distance;
    }
    else
    {
      ROS_ERROR("[test_srv_getNearestOccDist::main] Failed to call service get_nearest_occ_dist!");
      distance = -2;
    }
    timer_end = std::chrono::steady_clock::now();

    duration = std::chrono::duration_cast<std::chrono::microseconds>(timer_end - timer_start).count();
    frequency = pow(10,6) / duration;

    std::cout << "[test_srv_getNearestOccDist::main] " << ": (" << qp.x << ", " << qp.y << ", " << qp.z << ") -> " << distance << std::endl;
    std::cout << "[test_srv_getNearestOccDist::main] Processing duration: " << duration << " [µs]" << std::endl;
    std::cout << "[test_srv_getNearestOccDist::main] Processing frequency: " << frequency << " [Hz]" << std::endl;
    std::cout << "" << std::endl;
    
    r.sleep();
    ros::spinOnce();
  }

  //ros::spin();

  cout << "[test_srv_getNearestOccDist::main] END" << endl;
  return 0;
}