#include "mobiman_simulation/jaco_utility.h"

void positionListener(trajectory_msgs::JointTrajectory trajectory);


int main(int argc, char **argv) {
    ros::init(argc, argv, "jaco_position_utility");
    ros::NodeHandle nh;
    ros::Subscriber trajectory_subscriber = nh.subscribe("/arm_controller/command", 100, positionListener);
    ros::Publisher velocity_publisher = nh.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity", 100);
    ros::spin();

    return 0;
}

void positionListener(trajectory_msgs::JointTrajectory trajectory) {
    std::cout << trajectory.joint_names[0] << std::endl;
}