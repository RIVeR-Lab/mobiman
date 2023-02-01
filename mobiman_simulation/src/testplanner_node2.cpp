#include "testplanner2.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testplanner2");

  PlannerVoxblox planer;

  Eigen::Vector3d p0(0, 0, 0);
  Eigen::Vector3d p1(1, 0, 0);
  Eigen::Vector3d p2(1, 1, 0);
  Eigen::Vector3d p3(0, 1, 0);

  Eigen::Vector3d p4(0, 0, 1);
  Eigen::Vector3d p5(1, 0, 1);
  Eigen::Vector3d p6(1, 1, 1);
  Eigen::Vector3d p7(0, 1, 1);

  Eigen::Vector3d p8(0, -1, 0);
  Eigen::Vector3d p9(1, -1, 0);
  Eigen::Vector3d p10(0, -1, 1);
  Eigen::Vector3d p11(1, -1, 1);

  Eigen::Vector3d p12(2, 0, 0);
  Eigen::Vector3d p13(-1, 0, 0);
  Eigen::Vector3d p14(-2, 0, 0);

  std::vector<Eigen::Vector3d> pv;

  /*
  pv.push_back(p12);
  pv.push_back(p1);
  pv.push_back(p0);
  pv.push_back(p13);
  pv.push_back(p14);
  */

  pv.push_back(p0);
  pv.push_back(p1);
  pv.push_back(p2);
  pv.push_back(p3);
  pv.push_back(p4);
  pv.push_back(p5);
  pv.push_back(p6);
  pv.push_back(p7);
  pv.push_back(p8);
  pv.push_back(p9);
  pv.push_back(p10);
  pv.push_back(p11);
  pv.push_back(p12);

  double d;

  ros::Rate r(10);

  while (ros::ok())
  {
    std::cout << "=============================" << std::endl;

    for (size_t i = 0; i < pv.size(); i++)
    {
      std::cout << "[testplanner2_node::main] p" << i << ": " << pv[i][0] << ", " << pv[i][1] << ", " << pv[i][2] << std::endl;
      d = planer.getMapDistanceAtPosition(pv[i]);
      std::cout << "[testplanner2_node::main] d: " << d << std::endl << std::endl;
    }

    std::cout << "=============================" << std::endl << std::endl;
    ros::spinOnce();
    r.sleep();
  }
}


