#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <thread> 
#include <mutex>
#include <cstdlib>
#include <csignal>
#include <math.h> 
#include <signal.h>
#include <string.h>

// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <Eigen/LU>
// #include <Eigen/SVD>
// #include <chrono>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

// #include "ros/ros.h"
#include "avatar_panda.hpp"

// #include "sensor_msgs/JointState.h"
// #include "geometry_msgs/Point.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/Quaternion.h"
// #include "std_msgs/Bool.h"
// #include "franka_control/PTIPacket.h"
// #include "franka_control/PInfo.h"

// volatile sig_atomic_t done = 0;

// void signal_callback_handler(int signum) {
//     std::cout << "CTRL+C interrupted. " << std::endl;
//     // Terminate program
//     if (signum == SIGINT) {
//         done = 1;
//     }
// }


// /* Panda teleop interface */
// PTINode::PTINode(ros::NodeHandle& node, std::string type): node_type(type) {
//     nh_ = node;
//     ROS_INFO_STREAM("Launch ros interface as" << node_type << "panda");
//     joint_names.insert(joint_names.end(),{node_type+"_panda_joint1", 
//                                           node_type+"_panda_joint2", 
//                                           node_type+"_panda_joint3", 
//                                           node_type+"_panda_joint4", 
//                                           node_type+"_panda_joint5", 
//                                           node_type+"_panda_joint6", 
//                                           node_type+"_panda_joint7"});
//     pti_packet_sub = nh_.subscribe("/" + node_type + "_smarty_arm_output", 1, &PTINode::ptipacket_callback, this);
//     pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("pti_output", 1);
//     pinfo_pub = nh_.advertise<franka_control::PInfo>("panda_info", 1);
//     robot_joint_state_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

//     ROS_INFO("Node initialized");
// }

// // PTINode::~PTINode() = default;

// /* Publisher */
// void PTINode::publish_ptipacket() {
//     franka_control::PTIPacket packet_msg;
//     packet_msg.wave.resize(3);
//     packet_msg.est_ext_force.resize(6);

//     // mtx.lock();
//     packet_msg.robot_translation_mass = robot_translation_mass;
//     for (int i = 0; i < 3; i ++) {
//         packet_msg.wave[i] = wave_out[i];
//     }
//     for (int i = 0; i < 3; i ++) {
//         packet_msg.est_ext_force[i] = est_ext_force[i];
//     }
//     packet_msg.position.x = position_relative[0];
//     packet_msg.position.y = position_relative[1];
//     packet_msg.position.z = position_relative[2];
//     packet_msg.angle.x = -quat_error[0];
//     packet_msg.angle.y = -quat_error[1];
//     packet_msg.angle.z = -quat_error[2];
//     packet_msg.twist.linear.x = twist[0];
//     packet_msg.twist.linear.y = twist[1];
//     packet_msg.twist.linear.z = twist[2];
//     packet_msg.twist.angular.x = twist[3];
//     packet_msg.twist.angular.y = twist[4];
//     packet_msg.twist.angular.z = twist[5];
//     // mtx.unlock();
//     packet_msg.local_stamp = ros::Time::now().toSec();
//     packet_msg.remote_stamp = remote_time;
//     packet_msg.position_d.x = position_d[0];
//     packet_msg.position_d.y = position_d[1];
//     packet_msg.position_d.z = position_d[2];

//     // if (pti_packet_pub.getNumSubscribers() == 0) {
//     //     if (node_type == "Right") {
//     //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
//     //         pti_packet_pub.shutdown();
//     //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_right_output", 1);
//     //         // pti_packet_sub = nh_.subscribe("/right_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
//     //     }
//     //     else if (node_type == "Left") {
//     //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
//     //         pti_packet_pub.shutdown();
//     //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_left_output", 1);
//     //         // pti_packet_sub = nh_.subscribe("/left_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
//     //     }
//     // }

//     pti_packet_pub.publish(packet_msg);

// }

// void PTINode::publish_pinfo() {
//     franka_control::PInfo info_msg;
//     info_msg.external_load_mass = std::max((est_ext_force[2] - 9.0) / 10.0, 0.0);
//     info_msg.slow_catching_index = slow_catching_flag;
//     pinfo_pub.publish(info_msg);
// }

// void PTINode::publish_robot_joint_state() {
//     sensor_msgs::JointState states;
//     states.effort.resize(joint_names.size());
//     states.name.resize(joint_names.size());
//     states.position.resize(joint_names.size());
//     states.velocity.resize(joint_names.size());
//     states.header.stamp = ros::Time::now();
//     for (size_t i = 0; i < joint_names.size(); i++) {
//         states.name[i] = joint_names[i];
//         states.position[i] = q[i];
//         states.velocity[i] = dq[i];
//         states.effort[i] = tau[i];
//     }
//     robot_joint_state_pub.publish(states);
// }

// /* Subscriber callback */
// void PTINode::ptipacket_callback(const franka_control::PTIPacket::ConstPtr &packet_msg) {

//     // mtx.lock();
//     Eigen::Vector3d wave_in_unfiltered;
//     for (int i = 0; i < 3; i ++) {
//         wave_in_unfiltered[i] = packet_msg->wave[i];
//     }
//     wave_in = (wave_filter_freq * sample_time * wave_in_unfiltered + wave_in_prev) / (wave_filter_freq * sample_time + 1.0);
//     wave_in_prev = wave_in;
//     quat_in_prev = quat_in;
//     filtered_quat_in_prev = filtered_quat_in;
//     twist_in_prev = twist_in;
//     filtered_twist_in_prev = filtered_twist_in;

//     position_in << packet_msg->position.x, packet_msg->position.y, packet_msg->position.z;
//     quat_in << packet_msg->quat.x, packet_msg->quat.y, packet_msg->quat.z;
//     twist_in << packet_msg->twist.linear.x, packet_msg->twist.linear.y, packet_msg->twist.linear.z,\
//                     packet_msg->twist.angular.x, packet_msg->twist.angular.y, packet_msg->twist.angular.z;
//     // mtx.unlock();

//     for (int i = 0; i < 3; i ++) {
//         filtered_quat_in[i] = firstOrderIIRFilter(quat_in[i], quat_in_prev[i], filtered_quat_in_prev[i]);
//         filtered_twist_in[i + 3] = firstOrderIIRFilter(twist_in[i + 3], twist_in_prev[i + 3], filtered_twist_in_prev[i + 3]);
//     }
//     delay_cycle = (int)((ros::Time::now().toSec() - packet_msg->remote_stamp) / 1e-3 / 2);
//     remote_time = packet_msg->local_stamp;
//     // ROS_INFO_THROTTLE(1, "Write into pti memory");
// }

// /* Run loop */
// void PTINode::ros_run(int* status) {
//     th_ros_running = true;
//     int publish_ratio = 10;
//     int publish_index = 1;
//     ros::Rate loop_rate(1000);
//     while (!done && *status == 0) {
//         // signal(SIGINT, signal_callback_handler);
//         // publish_ptipacket();
//         if (publish_index < publish_ratio) publish_index++;
//         else {
//             publish_pinfo();
//             publish_robot_joint_state();
//             publish_index = 1;
//         }
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     wave_out.setZero();
//     publish_ptipacket();
// }

// /* Slow catching */
// void PTINode::slow_catching(void) {
//     int num = 3;
//     double lambda = 10.0;
//     double translation_stiffness = 3000.0;
//     double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
//     double rotation_stiffness = 300.0;
//     double rotation_damping = 2.0 * 1.0 * std::sqrt(rotation_stiffness * 0.01);
//     double torque_ratio = 0.9;
//     double force_ratio = 0.2;

//     double regulate_translation_velocity = 5.0;
//     double regulate_rotation_velocity = 8.0;

//     Eigen::Vector3d position_target;
//     Eigen::Vector3d quat_target;

//     for (int i = 0; i < 3; i ++) {
//         if (position_in[i] > position_relative[i]) {
//             position_target[i] = std::min(position_in[i], position_relative[i] + regulate_translation_velocity * sample_time);
//         }
//         else {
//             position_target[i] = std::max(position_in[i], position_relative[i] - regulate_translation_velocity * sample_time);
//         }

//         if (quat_in[i] > -quat_error[i]) {
//             quat_target[i] = std::min(quat_in[i], -quat_error[i] + regulate_rotation_velocity * sample_time); 
//         }
//         else {
//             quat_target[i] = std::max(quat_in[i], -quat_error[i] - regulate_rotation_velocity * sample_time);
//         }
//     }

//     // std::cout << quat_in.transpose() + quat_error.transpose() << std::endl;

//     force.head(3) = translation_stiffness * (position_target - position_relative) + translation_damping * (-twist.head(3));

//     force.tail(3) = rotation_stiffness * (quat_target + quat_error) + rotation_damping * (-twist.tail(3));
//     force = force_regulation(force, force_ratio);
    
//     tau = jacobian.transpose() * force + coriolis + tau_hose + tau_wall;
//     tau = torque_regulation(tau, last_tau, torque_ratio);
//     last_tau = tau;

// }

// /* panda control */
// void panda_control(PTINode& pti, std::string type, std::string ip, int* status) {

//     std::thread th_nullspaceControl;
//     std::thread th_ros;

//     std::cout << type << " node starts running" << std::endl;

//     try {
//         // connect to robot
//         franka::Robot robot(ip);
//         robot.automaticErrorRecovery();
//         setDefaultBehavior(robot);

//         // set collision behavior
//         robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                 {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                 {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                 {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        
//         // set external load
//         if (type == "right") {
//             const double load_mass = 0.9; // 2.5 fully filled
//             // const std::array< double, 3 > F_x_Cload = {{0.03, -0.01, -0.06}};
//             // const std::array< double, 9 > load_inertia = {{0.01395, 0.0, 0.0, 0.0, 0.01395, 0.0, 0.0, 0.0, 0.00125}};
//             const std::array< double, 3 > F_x_Cload = {{0.0, 0.0, 0.0}};
//             const std::array< double, 9 > load_inertia = {{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
//             robot.setLoad(load_mass, F_x_Cload, load_inertia);
//         }
//         else if (type == "left") {
//             const double load_mass = 0.9;
//             // const std::array< double, 3 > F_x_Cload = {{0.03, -0.01, -0.06}};
//             // const std::array< double, 9 > load_inertia = {{0.01395, 0.0, 0.0, 0.0, 0.01395, 0.0, 0.0, 0.0, 0.00125}};
//             const std::array< double, 3 > F_x_Cload = {{0.0, 0.0, 0.0}};
//             const std::array< double, 9 > load_inertia = {{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
//             robot.setLoad(load_mass, F_x_Cload, load_inertia);
//         }
        
//         // First move the robot to a suitable joint configuration
//         std::array<double, 7> q_goal;
//         if (type == "right") {
//             q_goal = std::array<double, 7>{{0.0, 0.0, 0.0, -2.7, 1.5708, 1.5708, -2.0}};
//         }
//         else if (type == "left") {
//             q_goal = std::array<double, 7>{{0.0, 0.0, 0.0, -2.7, -1.5708, 1.5708, 0.4}};
//         }
//         MotionGenerator motion_generator(0.3, q_goal);
//         std::cout << "WARNING: " << type << " arm starts moving."
//                 << "Please make sure to have the user stop button at hand!" << std::endl
//                 << "Press Enter to continue..." << std::endl;
//         // std::cin.ignore();
//         robot.control(motion_generator);
//         std::cout << "Finished moving to initial joint configuration." << std::endl;

//         if (type == "right") {
//             std::array<double, 16> ee = {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0}};
//             robot.setEE(ee);
//         }
//         else if (type == "left") {
//             std::array<double, 16> ee = {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0}};
//             robot.setEE(ee);
//         }

//         // load the kinematics and dynamics model
//         franka::Model model = robot.loadModel();
//         pti.initial_state = robot.readOnce();

//         pti.teleInit(model);
//         std::cout << type << " PTI class initialized" << std::endl;

//         if (type == "right") {
//             pti.hose_gravity[2] = 10.0;
//         }
//         else if (type == "left") {
//             pti.hose_gravity[2] = 10.0;
//         }

//         franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

//         // slow catching smarty arm
//         std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
//             impedance_control_callback = [&pti, &model, zero_torques, type](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
//         {
//             /* ctrl+c finish running */
//             if (done) {
//                 std::cout << std::endl << "Ctrl + c entered, shutting down slow catching" << std::endl;
//                 return franka::MotionFinished(zero_torques);
//             }
//             pti.robotStateUpdate(model, robot_state);
//             pti.jointLimit(type);
//             pti.slow_catching();

//             std::array<double, 7> tau_d_array{};
//             Eigen::VectorXd::Map(&tau_d_array[0], 7) = pti.tau;

//             double position_error = std::sqrt((pti.position_in[0] - pti.position_relative[0]) * (pti.position_in[0] - pti.position_relative[0])
//                                             + (pti.position_in[1] - pti.position_relative[1]) * (pti.position_in[1] - pti.position_relative[1])
//                                             + (pti.position_in[2] - pti.position_relative[2]) * (pti.position_in[2] - pti.position_relative[2]));
//             double quaternion_error = std::sqrt((pti.quat_in[0] + pti.quat_error[0]) * (pti.quat_in[0] + pti.quat_error[0])
//                                         + (pti.quat_in[1] + pti.quat_error[1]) * (pti.quat_in[1] + pti.quat_error[1])
//                                         + (pti.quat_in[2] + pti.quat_error[2]) * (pti.quat_in[2] + pti.quat_error[2]));

//             double position_threshold = 0.02;
//             double quaternion_threshold = 0.05;

//             std::cout << "position error: " << position_error << " , quaternion error: " << quaternion_error << std::endl;
//             if ((position_error < position_threshold) && (quaternion_error < quaternion_threshold)) {
//                 std::cout << std::endl << "Homing position reached, starting teleop" << std::endl;
//                 pti.tau = pti.coriolis;
//                 franka::Torques homing_stop_torque{{pti.tau[0], pti.tau[1], pti.tau[2], pti.tau[3], pti.tau[4], pti.tau[5], pti.tau[6]}};
//                 return franka::MotionFinished(homing_stop_torque);
//             }

//             return tau_d_array;
//         };

//         th_nullspaceControl = std::thread([&pti, status](){pti.nullHandling(status);});
//         th_ros = std::thread([&pti, status](){pti.ros_run(status);});

//         pti.slow_catching_flag = 1;
//         robot.control(impedance_control_callback);
//         pti.slow_catching_flag = 0;

//         pti.position_d = pti.position_in;
//         pti.last_tau.setZero();
//         pti.filtered_quat_in = pti.quat_in;
//         pti.filtered_quat_in_prev = pti.quat_in;

//         std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
//             tele_control_callback = [&pti, &model, zero_torques, type](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
//         {
//             /* ctrl+c finish running */
//             if (done) {
//                 std::cout << std::endl << "Finished task, shutting down operation" << std::endl;
//                 return franka::MotionFinished(zero_torques);
//             }

//             pti.robotStateUpdate(model, robot_state);
//             pti.jointLimit(type);
//             pti.sTeleController();
//             pti.publish_ptipacket();

//             std::array<double, 7> tau_d_array{};
//             Eigen::VectorXd::Map(&tau_d_array[0], 7) = pti.tau;
//             return tau_d_array;
//         };

//         std::cout << type << " panda teleop controller starts running" << std::endl;
//         robot.control(tele_control_callback);

//         th_ros.join(); pti.th_ros_running = false;
//         th_nullspaceControl.join(); pti.th_nullspace_running = false;
//         *status = 0;

//     } catch (franka::Exception const& e) {
//         std::cout << e.what() << std::endl;
//         *status = -1;
//         std::cout << type << " arm error, waiting for recovery..." << std::endl;

//         // std::cout << *status << std::endl;

//         if (pti.th_nullspace_running) {
//             th_nullspaceControl.join();
//             pti.th_nullspace_running = false;
//         }
//         if (pti.th_ros_running) {
//             th_ros.join();
//             pti.th_ros_running = false;
//         }
//         std::cout << "All thread shut down" << std::endl;
        
//     }

// }

// /* auto-recovery */
// void arm_run(PTINode& pti, std::string type, std::string ip, int* status) {

//     // PTINode pti(node, type);

//     while(!done) {
//         panda_control(pti, type, ip, status);
//         if (*status == 0) {
//             std::cout << type << " panda control stop" << std::endl;
//             break;
//         }
//         else if (*status == -1) {
//             std::cout << type << " panda restarts. Pressure enter to continue" << std::endl;
//             // std::cin.ignore();
//         }
//         *status = 0;
//     }
//     std::cout << type << " panda program stops." << std::endl;
// }


int main(int argc, char** argv) {

    int status = 0;

    // signal(SIGINT, signal_callback_handler);

    // // Check whether the required arguments were passed
    // if (argc != 2) {
    //     std::cout << argc << std::endl;
    //     std::cerr << "Usage: avatar_panda && left or right" << std::endl;
    //     return -1;
    // }

    // // ros::init(argc, argv, "pti_interface", ros::init_options::NoSigintHandler);

    // if (std::string(argv[1]) == "left") {
    //     ros::init(argc, argv, "pti_interface_left", ros::init_options::NoSigintHandler);
    //     ros::NodeHandle node("~");
    //     std::string ip = "192.168.1.101";
    //     PTINode pti(node, "left");
    //     arm_run(pti, "left", ip, &status);
    // }
    // else if (std::string(argv[1]) == "right") {
    //     ros::init(argc, argv, "pti_interface_right", ros::init_options::NoSigintHandler);
    //     ros::NodeHandle node("~");
    //     std::string ip = "192.168.1.100";
    //     PTINode pti(node, "right");
    //     arm_run(pti, "right", ip, &status);
    // }
    // else {
    //     std::cout << "Error: left or right arm not configured." << std::endl;
    //     return -1;
    // }

    // // std::thread left_arm_run([&pti_left, ip_left, &left_status](){arm_run(pti_left, "Left", ip_left, &left_status);});
    // // arm_run(pti_right, "Right", ip_right, &right_status);
    // // arm_run(left_node, "Left", ip_left, &left_status);
    // // left_arm_run.join();

    // ros::shutdown();

    // std::cout << "Done." << std::endl;
    
    return 0;
}
