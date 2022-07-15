#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <thread> 
#include <mutex>
#include <stdio.h>
#include <signal.h>
#include <math.h> 

#include <Eigen/Dense>
#include <chrono>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "franka_control/PTIPacket.h"

std::mutex mtx;

class Panda {
    public:
    // robot model
    // franka::Robot robot;
    // franka::Model model;

    /* robot joint state */
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> q0;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau;

    /* robot initial state */
    franka::RobotState initial_state;
    Eigen::Affine3d initial_transform;
    Eigen::Vector3d position_0;
    Eigen::Quaterniond orientation_0;

    /* robot end-effector state */
    Eigen::Affine3d transform;
    Eigen::Vector3d position;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position_relative;
    Eigen::Vector3d angle_relative;
    Eigen::Matrix<double, 6, 1> twist;
    Eigen::Matrix<double, 6, 1> twist_d;
    Eigen::Matrix<double, 6, 1> force;

    Eigen::Matrix<double, 7, 1> coriolis;
    Eigen::Matrix<double, 6, 7> jacobian;

    /* wave variable */
    Eigen::Vector3d wave_in;
    Eigen::Vector3d wave_out;
    Eigen::Vector3d wave_integral;
    Eigen::Vector3d position_in;
    Eigen::Vector3d angle_in;
    Eigen::Matrix<double, 6, 1> twist_in;

    int delay_current_index;
    int delay_cycle_previous;
    int max_buff_size = 2000;
    double wave_history[3][2000];

    /* panda initialization */
    void teleInit(franka::Model& model) {

        initial_transform = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
        position_0 = initial_transform.translation();
        orientation_0 = initial_transform.linear();

        q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());

        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(model.zeroJacobian(franka::Frame::kEndEffector, initial_state).data());

        position_d.setZero();
        twist_d.setZero();
        wave_in.setZero();
        wave_integral.setZero();
        position_in.setZero();
        angle_in.setZero();
        twist_in.setZero();

        delay_current_index = 0;
        delay_cycle_previous = 3;
        // max_buff_size = 2000;
        for (int i = 0; i < 3; i ++) {
            for (int j = 0; j < max_buff_size; j ++) {
                wave_history[i][j] = 0.0;
            }
        }

    }

    /* update robot endeffector state */
    void robotStateUpdate(franka::Model& model, const franka::RobotState& robot_state) {

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

        coriolis = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
        q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
        dq = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

        transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        position = transform.translation();
        orientation = transform.linear();
        position_relative = position - position_0;

        Eigen::Quaterniond orientation_relative;
        Eigen::Vector3d angle_relative_local;
        orientation_relative = orientation.inverse() * orientation_0;
        angle_relative_local << orientation_relative.x(), orientation_relative.y(), orientation_relative.z();
        angle_relative = transform.linear() * angle_relative_local;

        twist = jacobian * dq;
    }

    /* master wave variable controller */
    void mTeleController(void) {

        double wave_damping = 0.1;

        force.setZero();
        force.head(3) = -1.0 * (wave_damping * twist.head(3) - std::sqrt(2.0 * wave_damping) * wave_in);
        wave_out = std::sqrt(2.0 * wave_damping) * twist.head(3) - wave_in;

        tau = jacobian.transpose() * force + coriolis;
    }

    /* slave wave variable controller */
    void sTeleController(void) {

        int num = 3;
        double sample_time = 1e-3;
        double wave_damping = 0.1;
        double lambda = 10.0;
        double translation_stiffness = 300.0;
        double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
        double rotation_stiffness = 10.0;
        double rotation_damping = 2.0 * 1.0 * std::sqrt(rotation_stiffness * 1.0);

        Eigen::Vector3d actual_position_error;
        Eigen::Vector3d predict_position_error;
        Eigen::Vector3d error_difference;
        Eigen::Vector3d wave_correction;

        int delay_index;
        int delay_difference;
        int delay_cycle_current = 3;

        // translation part with wave variable
        position_d += twist_d.head(3) * sample_time;
        twist_d.head(3) = (std::sqrt(2.0 * wave_damping) * wave_in + translation_damping * twist.head(3) + \
                        translation_stiffness * (position_relative - position_d)) / (wave_damping + translation_damping);
        force.head(3) = translation_stiffness * (position_d - position_relative) + translation_damping * (twist_d.head(3) - twist.head(3));
        wave_out = wave_in - std::sqrt(2.0 / wave_damping) * force.head(3);

        delay_index = delay_current_index - delay_cycle_current;
        delay_difference = delay_cycle_current - delay_cycle_previous;
        if (delay_index < 0) {
            delay_index += max_buff_size;
        }

        // position drift correction
        actual_position_error = position_in - position_d;
        predict_position_error = -1.0 / std::sqrt(2.0 * wave_damping) * wave_integral;
        error_difference = predict_position_error - actual_position_error;
        wave_correction = 1.0 * std::sqrt(2.0 * wave_damping) * (2.0 * M_PI * lambda) * error_difference;
        for (int i = 0; i < num; i ++) {
            if (error_difference[i] * wave_out[i] <= 0) {
                if (std::fabs(wave_correction[i]) < std::fabs(wave_out[i])) {
                    wave_out[i] += wave_correction[i];
                }
                else {
                    wave_out[i] = 0.0;
                }
            }

            /* check time-varying current delay cycle */
            if (delay_difference == 0) {
                wave_integral[i] -= wave_history[i][delay_index] * sample_time;
            }
            else if (delay_difference > 0) {
                for (int j = 1; j <= delay_difference; j ++) {
                    if (delay_index + j >= max_buff_size) wave_integral[i] += wave_history[i][delay_index + j - max_buff_size] * sample_time;
                    else wave_integral[i] += wave_history[i][delay_index + j] * sample_time;
                }
            }
            else {
                for (int j = 0; j < -delay_difference; j ++) {
                    if (delay_index - j < 0) wave_integral[i] -= wave_history[i][delay_index - j + max_buff_size] * sample_time;
                    else wave_integral[i] -= wave_history[i][delay_index - j] * sample_time;
                }
            }
            wave_integral[i] += wave_out[i] * sample_time;
            wave_history[i][delay_current_index] = wave_out[i];
        }
        delay_cycle_previous = delay_cycle_current;
        delay_current_index++;
        if (delay_current_index >= max_buff_size) {
            delay_current_index = 0;
        }


        // open loop rotation control
        force.tail(3) = rotation_stiffness * (angle_in - angle_relative) + rotation_damping * (twist_d.tail(3) - twist.tail(3));

        tau = jacobian.transpose() * force + coriolis;
    }

};

class PTINode {
    public:
    explicit PTINode(ros::NodeHandle& node, std::string type);
    // ~PTINode();
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

// PTINode::~PTINode() = default;

/* Publisher */
void PTINode::publish_ptipacket() {
    franka_control::PTIPacket packet_msg;
    packet_msg.wave.resize(3);

    mtx.lock();
    for (int i = 0; i < 3; i ++) {
        packet_msg.wave[i] = panda->wave_out[i];
    }
    packet_msg.position.x = panda->position_relative[0];
    packet_msg.position.y = panda->position_relative[1];
    packet_msg.position.z = panda->position_relative[2];
    packet_msg.angle.x = panda->angle_relative[0];
    packet_msg.angle.y = panda->angle_relative[1];
    packet_msg.angle.z = panda->angle_relative[2];
    packet_msg.twist.linear.x = panda->twist[0];
    packet_msg.twist.linear.y = panda->twist[1];
    packet_msg.twist.linear.z = panda->twist[2];
    packet_msg.twist.angular.x = panda->twist[3];
    packet_msg.twist.angular.y = panda->twist[4];
    packet_msg.twist.angular.z = panda->twist[5];
    mtx.unlock();

    if (pti_packet_pub.getNumSubscribers() == 0) {
        if (node_type == "master") {
            ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
            pti_packet_pub.shutdown();
            pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_master_output", 1);
        }
        else if (node_type == "slave") {
            ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
            pti_packet_pub.shutdown();
            pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_slave_output", 1);
        }
    }

    pti_packet_pub.publish(packet_msg);

}

/* Subscriber callback */
void PTINode::ptipacket_callback(const franka_control::PTIPacket::ConstPtr &packet_msg) {

    mtx.lock();
    for (int i = 0; i < 3; i ++) {
        panda->wave_in[i] = packet_msg->wave[i];
    }
    panda->position_in << packet_msg->position.x, packet_msg->position.y, packet_msg->position.z;
    panda->angle_in << packet_msg->angle.x, packet_msg->angle.y, packet_msg->angle.z;
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
        return -1;
    }

    ros::NodeHandle node("~");
    Panda panda;

    PTINode pti(node, std::string(argv[2]));
    ROS_INFO("Node starts running");

    // pti.run();

    try {
        // connect to robot
        franka::Robot robot(argv[1]);
        robot.automaticErrorRecovery();
        setDefaultBehavior(robot);
        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();
        panda.initial_state = robot.readOnce();
        panda.teleInit(model);

        // // set collision behavior
        // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        //                         {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        //                         {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        //                         {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        //     impedance_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
        // {
            
        //     return tau_d_array;
        // };

        size_t count = 0;
        robot.read([&count, &panda, &model, &argv](const franka::RobotState& robot_state) {
        // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        // std::cout << robot_state << std::endl;
        panda.robotStateUpdate(model, robot_state);
        if (std::string(argv[2]) == "master") {
            panda.mTeleController();
        }
        else if (std::string(argv[2]) == "slave") {
            panda.sTeleController();
        }
        std::cout << panda.tau << std::endl;
        return ++count < 1;
        });

        std::cout << "Done." << std::endl;
    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }


    return 0;
}
