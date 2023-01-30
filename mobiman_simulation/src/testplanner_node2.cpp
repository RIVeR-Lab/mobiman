#include "testplanner2.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testplanner2");

  PlannerVoxblox planer;

  while (ros::ok())
  {
    for (double i = -5; i < 5; i = i + 1)
    {
      for (double j = -5; j < 5; j = j + 1)
      {
        for (double k = -10; k < 10; k = k + 1)
        {
          Eigen::Vector3d p(i, j, k);
          Eigen::Vector3d v(0,0,0);
          double d;

          d = planer.getMapDistanceAndGradientAtPosition(p, v);
          
          //std::cout << "[testplanner2_node::main] d: " << d << std::endl;

          if (d > 0)
          {
            ROS_INFO("Point = %0.2f , %0.2f , %0.2f ", p[0], p[1], p[2]);
            ROS_INFO("Distance = %0.2f ", d);
            ROS_INFO("Gradient = %0.7f , %0.7f , %0.7f ", v[0], v[1], v[2]);
          }
        }
      }
    }
    ros::spinOnce();
  }
}


