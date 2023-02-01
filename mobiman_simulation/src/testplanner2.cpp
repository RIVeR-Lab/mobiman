#include "testplanner2.h"

PlannerVoxblox::PlannerVoxblox()
    : nh_("~"),
      nh_private_("~"),
      voxblox_server_(nh_, nh_private_) 
{
    voxblox_server_.getServerConfigFromRosParam(nh_private_);  

    //double robot_radius = 0.2;
    //voxblox_server_.setTraversabilityRadius(robot_radius);
    //voxblox_server_.publishTraversable();  
}

double PlannerVoxblox::getMapDistanceAndGradientAtPosition(const Eigen::Vector3d position, Eigen::Vector3d gradient) const 
{  
  if (!voxblox_server_.getEsdfMapPtr()) 
  {
    return -1.0;
  }

  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(position,
                                                                         &distance, 
                                                                         &gradient)) 
  {
    return -2.0;
  }
  return distance;
}

double PlannerVoxblox::getMapDistanceAtPosition(const Eigen::Vector3d position) const 
{  
  if (!voxblox_server_.getEsdfMapPtr()) 
  {
    return -1.0;
  }

  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance))
  {
    return -2.0;
  }
  return distance;
}

