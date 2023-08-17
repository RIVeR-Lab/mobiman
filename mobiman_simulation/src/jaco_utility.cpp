#include "mobiman_simulation/jaco_utility.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "jaco_position_utility");
    std::cout << "[+] Node started: jaco_position_utility!" << std::endl;
    ros::NodeHandle nh;
    // cur_time = (float)ros::Time::now().toSec() + (float)ros::Time::now().toNSec()*1e-9;
    prev_time = (float)ros::Time::now().toSec() + (float)ros::Time::now().toNSec()*1e-9;
    ros::Timer timer = nh.createTimer(ros::Duration(1/100), pid_callback);
    ros::Subscriber state_subscriber = nh.subscribe("/j2n6s300_driver/out/joint_state", 100, jaco_feedback);
    ros::Subscriber trajectory_subscriber = nh.subscribe("/arm_controller/command", 100, position_listener);
    velocity_publisher = nh.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity", 100);
    
    signal(SIGINT, shutdown_handler);
    
    ros::spin();
    return 0;
}

void position_listener(trajectory_msgs::JointTrajectory trajectory) {
    jaco_trajectory = trajectory;
    
}

void jaco_feedback(sensor_msgs::JointState joint_state) {
    jaco_position = joint_state;
    // std::cout << "[Joint State] " << std::endl;
    // for(int i = 0; i < joint_state.position.size(); i++) {
    //     std::cout << joint_state.position[i] << " ";
    // }
    // std::cout << std::endl;
}

void shutdown_handler(int sig) {
    jaco_velocity.joint1 = 0;
    jaco_velocity.joint2 = 0;
    jaco_velocity.joint3 = 0;
    jaco_velocity.joint4 = 0;
    jaco_velocity.joint5 = 0;
    jaco_velocity.joint6 = 0;
    velocity_publisher.publish(jaco_velocity);
}

void pid_callback(const ros::TimerEvent &){ 
    for(int i = 0; i < jaco_trajectory.points.size(); i++) {
            std::cout << "[TRAJECTORY] " << std::endl;
            cur_time = (float)ros::Time::now().toSec() + (float)ros::Time::now().toNSec()*1e-9;
            dt = cur_time - prev_time;
            prev_time = cur_time;
            for(int j = 0; j < jaco_trajectory.points[i].positions.size(); j++) {
                error[j] = (jaco_trajectory.points[i].positions[j] - jaco_position.position[j]) / 100;
                pid[j] = error[j]*p;
                std::cout << " (error) " << error[j] << " (Prop) " << error[j]*p << " ";
            }
            jaco_velocity.joint1 = pid[0];
            jaco_velocity.joint2 = pid[1];
            jaco_velocity.joint3 = pid[2];
            jaco_velocity.joint4 = pid[3];
            jaco_velocity.joint5 = pid[4];
            jaco_velocity.joint6 = pid[5];
            velocity_publisher.publish(jaco_velocity);
            std::cout << std::endl;
        }

}