#include "testplanner2.h"

PlannerVoxblox::PlannerVoxblox()
    : nh_("~"),
      nh_private_("~"),
      voxblox_server_(nh_, nh_private_) 
{
    voxblox_server_.getServerConfigFromRosParam(nh_private_);    
}

double PlannerVoxblox::getMapDistanceAndGradientAtPosition(const Eigen::Vector3d position, Eigen::Vector3d gradient) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return -1.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(position,
                                                              &distance, &gradient)) {
    return -2.0;
  }
  return distance;
}

