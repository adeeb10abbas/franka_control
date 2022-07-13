#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <thread> 
#include <mutex>
#include <stdio.h>
#include <signal.h>    

#include <Eigen/Dense>
#include <chrono>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "franka_control/PTIPacket.h"

std::mutex mtx;

class Panda {
    public:

    // robot state
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> q0;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau;

    // robot end-effector state in Cartesian space wrt base frame
    Eigen::Vector3d position;
    Eigen::Vector3d position_0;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation;
    Eigen::Quaterniond orientation_0;
    Eigen::Quaterniond orientation_d;
    Eigen::Matrix<double, 6, 1> twist;
    Eigen::Matrix<double, 6, 1> twist_d;
    Eigen::Matrix<double, 6, 1> force;
    Eigen::Affine3d transform;

    // wave variable
    Eigen::Matrix<double, 3, 1> wave_in;
    Eigen::Matrix<double, 3, 1> wave_out;
    Eigen::Vector3d position_in;
    Eigen::Quaterniond orientation_in;
    Eigen::Matrix<double, 6, 1> twist_in;

    // master wave variable controller
    // only force feedback in translational direction
    // inline void mTeleController(void) {
    //     double wave_damping = 0.1;
    //     force.setZero();
    //     force.head(3) << -1.0 * (wave_damping * twist.head(3) - std::sqrt(2.0 * wave_damping) * wave_in);
    //     wave_out << std::sqrt(2.0 * wave_damping) * twist.head(3) - wave_in;
    // }

    // // slave wave variable controller
    // inline void sTeleController(void) {
    //     double sample_time = 1e-3;
    //     double wave_damping = 0.1;
    //     double translation_stiffness = 20.0;
    //     double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
    //     double rotation_stiffness = 1.0;
    //     double rotation_damping = 2.0 * 1.0 * std::sqrt(rotation_stiffness * 1.0);
    //     Eigen::Vector3d position_relative;
    //     Eigen::Quaterniond orientation_relative;
    //     Eigen::Vector3d orientation_angle_relative;

    //     position = transform.translation();
    //     orientation = transform.linear();
    //     position_relative << position - position_0;
    //     orientation_relative << orientation - orientation_0;

    //     // translation part with wave variable
    //     position_d += twist_d * sample_time;
    //     twist_d.head(3) << (std::sqrt(2.0 * wave_damping) * wave_in + translation_damping * twist.head(3) + translation_stiffness * (position_relative - position_d)) / (wave_damping + damping);
    //     force.head(3) << tranlation_stiffness * (position_d - position_relative) + translation_damping * (twist_d.head(3) - twist.head(3));
    //     wave_out << wave_in - std::sqrt(2.0 / wave_damping) * force.head(3);

    //     // open loop rotation control
    //     orientation_relative = orientation.inverse() * orientation_0;
    //     orientation_angle_relative << orientation_relative.x(), orientation_relative.y(), orientation_relative.z();
    //     orientation_angle_relative << transform.linear() * orientation_angle_relative;
    //     force.tail(3) << rotation_stiffness * (orientation_in - orientation_angle_relative) + rotation_damping * (twist_d.tail(3) - twist.tail(3));
    // }

};

class PTINode {
    public:
    explicit PTINode(ros::NodeHandle& node, std::string type);
    ~PTINode();
    void run();
    
    Panda *panda;
    ros::NodeHandle nh_;
    ros::Subscriber pti_packet_sub;
    ros::Publisher pti_packet_pub;
    std::string node_type;
    void publish_ptipacket();
    void ptipacket_callback(const franka_control::PTIPacket::ConstPtr &msg);
};

/* Panda teleop interface */
PTINode::PTINode(ros::NodeHandle& node, std::string type): node_type(type) {
    nh_ = node;

    if (node_type == "master") {
        ROS_INFO("Launch ros interface as master");
        pti_packet_sub = nh_.subscribe("/pti_slave_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
        pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_master_output", 1);
    }
    else if (node_type == "slave") {
        ROS_INFO("Launch ros interface as slave");
        pti_packet_sub = nh_.subscribe("/pti_master_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
        pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_slave_output", 1);
    }

    ROS_INFO("Node initialized");
}

PTINode::~PTINode() = default;

/* Publisher */
void PTINode::publish_ptipacket() {
    franka_control::PTIPacket packet_msg;
    packet_msg.wave.resize(3);

    mtx.lock();
    for (int i = 0; i < 3; i ++) {
        packet_msg.wave[i] = panda->wave_out[i];
    }
    packet_msg.pose.position.x = panda->position[0];
    packet_msg.pose.position.y = panda->position[1];
    packet_msg.pose.position.z = panda->position[2];
    packet_msg.pose.orientation.w = panda->orientation.w();
    packet_msg.pose.orientation.x = panda->orientation.x();
    packet_msg.pose.orientation.y = panda->orientation.y();
    packet_msg.pose.orientation.z = panda->orientation.z();
    packet_msg.twist.linear.x = panda->twist[0];
    packet_msg.twist.linear.y = panda->twist[1];
    packet_msg.twist.linear.z = panda->twist[2];
    packet_msg.twist.angular.x = panda->twist[3];
    packet_msg.twist.angular.y = panda->twist[4];
    packet_msg.twist.angular.z = panda->twist[5];
    mtx.unlock();

    // if (pti_packet_pub.getNumSubscribers() == 0) {
    //     if (node_type == "master") {
    //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
    //         pti_packet_pub.shutdown();
    //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_master_output", 1);
    //     }
    //     else if (node_type == "slave") {
    //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
    //         pti_packet_pub.shutdown();
    //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_slave_output", 1);
    //     }
        
    // }

    pti_packet_pub.publish(packet_msg);

}

/* Subscriber callback */
void PTINode::ptipacket_callback(const franka_control::PTIPacket::ConstPtr &packet_msg) {

    mtx.lock();
    for (int i = 0; i < 3; i ++) {
        panda->wave_in[i] = packet_msg->wave[i];
    }
    panda->position_in << packet_msg->pose.position.x, packet_msg->pose.position.y, packet_msg->pose.position.z;
    panda->orientation_in.w() = packet_msg->pose.orientation.w;
    panda->orientation_in.x() = packet_msg->pose.orientation.x;
    panda->orientation_in.y() = packet_msg->pose.orientation.y;
    panda->orientation_in.z() = packet_msg->pose.orientation.z;
    panda->twist_in << packet_msg->twist.linear.x, packet_msg->twist.linear.y, packet_msg->twist.linear.z,\
                    packet_msg->twist.angular.x, packet_msg->twist.angular.y, packet_msg->twist.angular.z;
    mtx.unlock();
    ROS_INFO_THROTTLE(1, "Write into pti memory");
}

/* Run loop */
void PTINode::run() {
    ros::Rate loop_rate(500);
    while (ros::ok()) {
        publish_ptipacket();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv) {
    // Check whether the required arguments were passed
    if (argc != 3) {
        std::cout << argc << std::endl;
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> & <master/slave>" << std::endl;
        return -1;
    }

    if (std::string(argv[2]) == "master") {
        ros::init(argc, argv, "pti_interface_master");
    }
    else if (std::string(argv[2]) == "slave") {
        ros::init(argc, argv, "pti_interface_slave");
    }
    else {
        std::cerr << "Mode: " << argv[2] << ", need to specify as master or slave" << std::endl;
    }

    ros::NodeHandle node("~");
    Panda *panda;

    PTINode pti(node, std::string(argv[2]));
    ROS_INFO("Node starts running");
    
    pti.run();


    // return 0;
}
