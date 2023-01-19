#include  <ros/ros.h>
#include  <voxblox_ros/esdf_server.h>

/* 
 * File:   testplanner.h
 * Author: schnph
 *
 * Created on 22. August 2018, 14:57
 */

#ifndef TESTPLANNER_H
#define TESTPLANNER_H

class PlannerVoxblox {
 public:
  PlannerVoxblox();
  virtual ~PlannerVoxblox() {}
  double getMapDistanceAndGradientAtPosition(const Eigen::Vector3d position, Eigen::Vector3d gradient) const;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
};





#endif /* TESTPLANNER_H */

