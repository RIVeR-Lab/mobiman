#include "mobiman_simulation/jaco_utility.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "jaco_position_utility");
    std::cout << "[+] Node started: jaco_position_utility!" << std::endl;
    ros::NodeHandle nh;
    
    ros::Timer timer = nh.createTimer(ros::Duration(1/((int)dt)), pid_callback);
    ros::Subscriber state_subscriber = nh.subscribe("/j2n6s300_driver/out/joint_state", 5, jaco_feedback);
    ros::Subscriber trajectory_subscriber = nh.subscribe("/arm_controller/command", 5, position_listener);
    velocity_publisher = nh.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity", 100);
    
    // signal(SIGINT, shutdown_handler);
    ros::spin();
    std::cout << "Shutdown" << std::endl;
    for(int i = 0; i < 100000; i++) {
        jaco_velocity.joint1 = 0;
        jaco_velocity.joint2 = 0;
        jaco_velocity.joint3 = 0;
        jaco_velocity.joint4 = 0;
        jaco_velocity.joint5 = 0;
        jaco_velocity.joint6 = 0;
        velocity_publisher.publish(jaco_velocity);
    }
    return 0;
}

void position_listener(trajectory_msgs::JointTrajectory trajectory) {
    // jaco_trajectory = trajectory;
    mrt_target = Eigen::Map<Eigen::Matrix<double, 6, 1>>(trajectory.points[0].positions.data());
    start_pid = true;
    time_ = ros::Time::now();
}

void jaco_feedback(sensor_msgs::JointState joint_state) {
    jaco_state = Eigen::Map<Eigen::Matrix<double, 6, 1>>(joint_state.position.data());
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
    if(!start_pid) {
        return;
    }
    std::cout << ros::Time::now() - time_ << std::endl;
    if((ros::Time::now() - time_) > ros::Duration(3.0)) {
        jaco_velocity.joint1 = 0;
        jaco_velocity.joint2 = 0;
        jaco_velocity.joint3 = 0;
        jaco_velocity.joint4 = 0;
        jaco_velocity.joint5 = 0;
        jaco_velocity.joint6 = 0;
        velocity_publisher.publish(jaco_velocity);
        return;
    } 
    try {
        error_ = mrt_target - jaco_state;
        // error_ = mrt_target - jaco_state;
        propotional = error_;
        derivative = (error_ - prev_error_) / dt;
        prev_error_ = error_;
        integral_ += error_;
        output_ = propotional * p + integral * i + derivative * d;
        // output_.max(10).min(-10);
        jaco_velocity.joint1 = (output_[0] > max_ ? max_ : output_[0]) < min_ ? min_ : (output_[0] > max_ ? max_ : output_[0]);
        jaco_velocity.joint2 = (output_[1] > max_ ? max_ : output_[1]) < min_ ? min_ : (output_[1] > max_ ? max_ : output_[1]);
        jaco_velocity.joint3 = (output_[2] > max_ ? max_ : output_[2]) < min_ ? min_ : (output_[2] > max_ ? max_ : output_[2]);
        jaco_velocity.joint4 = (output_[3] > max_ ? max_ : output_[3]) < min_ ? min_ : (output_[3] > max_ ? max_ : output_[3]);
        jaco_velocity.joint5 = (output_[4] > max_ ? max_ : output_[4]) < min_ ? min_ : (output_[4] > max_ ? max_ : output_[4]);
        jaco_velocity.joint6 = (output_[5] > max_ ? max_ : output_[5]) < min_ ? min_ : (output_[5] > max_ ? max_ : output_[5]);
        std::cout << output_[0] << " " << jaco_velocity.joint1 << "\n"
                << output_[1] << " " << jaco_velocity.joint2 << "\n"
                << output_[2] << " " << jaco_velocity.joint3 << "\n"
                << output_[3] << " " << jaco_velocity.joint4 << "\n"
                << output_[4] << " " << jaco_velocity.joint5 << "\n"
                << output_[5] << " " << jaco_velocity.joint6 << "----------\n" << std::endl;
        velocity_publisher.publish(jaco_velocity);

    } catch (...) {

    }

}