#include <ros/ros.h>
#include <kinova_msgs/JointVelocity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <vector>

// Global variable current position
sensor_msgs::JointState jaco_position;
// Global variable target position
trajectory_msgs::JointTrajectory jaco_trajectory;
//PID propotions
float p = 8000.0f;
float i = 0.0f;
float d = 0.0f;
float dt = 0.0f;
float cur_time;
float prev_time;
std::vector<float> error(6, 0.0f);
std::vector<float> pid(6, 0.0f);
// Kinova_velocity msg
kinova_msgs::JointVelocity jaco_velocity;

ros::Publisher velocity_publisher;
void position_listener(trajectory_msgs::JointTrajectory trajectory);
void jaco_feedback(sensor_msgs::JointState joint_state);
void shutdown_handler(int sig);
void pid_callback(const ros::TimerEvent &);