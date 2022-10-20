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

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
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

volatile sig_atomic_t done = 0;

void signal_callback_handler(int signum) {
    std::cout << "CTRL+C interrupted. " << std::endl;
    // Terminate program
    if (signum == SIGINT) {
        done = 1;
    }
}

class PTINode {
    public:

    explicit PTINode(ros::NodeHandle& node, std::string type);
    void ros_run(int* status);
    void slow_catching(void);
    ros::NodeHandle nh_;
    ros::Subscriber pti_packet_sub;
    ros::Publisher pti_packet_pub;
    std::string node_type;
    void publish_ptipacket();
    void ptipacket_callback(const franka_control::PTIPacket::ConstPtr &msg);
    std::mutex mtx;

    bool th_nullspace_running = false;
    bool th_ros_running = false;

    /* robot joint state */
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> q0;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau;
    Eigen::Matrix<double, 7, 1> last_tau;

    Eigen::Matrix<double, 7, 1> tau_nullspace;
    Eigen::Matrix<double, 7, 1> tau_wall;
    Eigen::Matrix<double, 7, 1> tau_hose;
    Eigen::Matrix<double, 6, 1> hose_gravity;

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
    Eigen::Matrix<double, 6, 1> est_ext_force;

    Eigen::Matrix<double, 7, 1> coriolis;
    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 7> joint5_jacobian;

    /* wave variable */
    Eigen::Vector3d wave_in;
    Eigen::Vector3d wave_out;
    Eigen::Vector3d wave_integral;
    Eigen::Vector3d position_in;
    Eigen::Vector3d angle_in;
    Eigen::Matrix<double, 6, 1> twist_in;
    double wave_damping;

    int delay_current_index;
    int delay_cycle_previous;
    int max_buff_size = 2000;
    double wave_history[3][2000];
    int delay_cycle;
    double sample_time;

    /* panda initialization */
    void teleInit(franka::Model& model) {

        initial_transform = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
        position_0 = initial_transform.translation();
        orientation_0 = initial_transform.linear();

        q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());

        std::cout << q0.transpose() << std::endl;

        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(model.zeroJacobian(franka::Frame::kEndEffector, initial_state).data());
        joint5_jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(model.zeroJacobian(franka::Frame::kJoint5, initial_state).data());

        position_d.setZero();
        twist_d.setZero();
        wave_in.setZero();
        wave_integral.setZero();
        position_in.setZero();
        angle_in.setZero();
        twist_in.setZero();
        tau_nullspace.setZero();
        tau.setZero();
        last_tau.setZero();
        tau_wall.setZero();

        wave_damping = 10.0;

        delay_current_index = 0;
        delay_cycle_previous = 2;
        delay_cycle = 2;
        // max_buff_size = 2000;
        for (int i = 0; i < 3; i ++) {
            for (int j = 0; j < max_buff_size; j ++) {
                wave_history[i][j] = 0.0;
            }
        }
        sample_time = 1e-3;

        hose_gravity.setZero();
    }

    /* update robot endeffector state */
    void robotStateUpdate(franka::Model& model, const franka::RobotState& robot_state) {

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        std::array<double, 42> joint5_jacobian_array = model.zeroJacobian(franka::Frame::kJoint5, robot_state);

        coriolis = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
        joint5_jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(joint5_jacobian_array.data());
        q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
        dq = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

        est_ext_force = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());

        // std::cout << q.transpose()<< std::endl;

        transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        position = transform.translation();
        orientation = transform.linear();
        position_relative = position - position_0;

        Eigen::Quaterniond orientation_relative;
        Eigen::Vector3d angle_relative_local;
        if (orientation_0.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        orientation_relative = orientation.inverse() * orientation_0;
        angle_relative_local << orientation_relative.x(), orientation_relative.y(), orientation_relative.z();
        angle_relative = transform.linear() * angle_relative_local;

        // std::cout << angle_relative.transpose() << std::endl;

        twist = jacobian * dq;
        tau_hose = joint5_jacobian.transpose() * hose_gravity;
    }

    /* slave wave variable controller */
    void sTeleController(void) {

        int num = 3;
        double lambda = 10.0;
        double translation_stiffness = 1200.0;
        double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
        double rotation_stiffness = 20.0;
        double rotation_damping = 2.0 * 1.0 * std::sqrt(rotation_stiffness * 0.01);
        double torque_ratio = 0.8;
        double force_ratio = 0.5;

        Eigen::Vector3d actual_position_error;
        Eigen::Vector3d predict_position_error;
        Eigen::Vector3d error_difference;
        Eigen::Vector3d wave_correction;

        int delay_index;
        int delay_difference;
        int delay_cycle_current;
        delay_cycle_current = 4;

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
        force.tail(3) = -rotation_stiffness * (-angle_in - angle_relative) + rotation_damping * (twist_in.tail(3) - twist.tail(3));
        force = force_regulation(force, force_ratio);
        
        tau = jacobian.transpose() * force + coriolis + tau_nullspace + tau_wall + tau_hose;
        // tau = jacobian.transpose() * force + coriolis + tau_nullspace + tau_wall;
        tau = torque_regulation(tau, last_tau, torque_ratio);
        last_tau = tau;

    }

    /* joint virtual wall limit*/
    void jointLimit(std::string type) {
        std::array<double, 7> q_min_degree = {{-160.0, -90.0, -160.0, -160.0, -160.0, 5.0, -160.0}};
        std::array<double, 7> q_max_degree = {{160.0, 90.0, 160.0, -15.0, 160.0, 209.0, 160.0}};

        const std::array<double, 7> k_gains = {{100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 20.0}};
        const std::array<double, 7> d_gains = {{15.0, 15.0, 15.0, 15.0, 10.0, 10.0, 5.0}};
        double d2r = 180.0 / M_PI;
        std::array<double, 7> q_min_radian;
        std::array<double, 7> q_max_radian;

        for (int i = 0; i < 7; i ++) {
            q_min_radian[i] = q_min_degree[i] / d2r;
            q_max_radian[i] = q_max_degree[i] / d2r;
            if (q[i] < q_min_radian[i]) {
                tau_wall[i] = (k_gains[i] * (q_min_radian[i] - q[i]) - d_gains[i] * dq[i]);
                std::cout << "Joint [" << i << "] reach lower limit at time: " << ros::Time::now() << std::endl;
            }
            else if (q[i] > q_max_radian[i]) {
                tau_wall[i] = (k_gains[i] * (q_max_radian[i] - q[i]) - d_gains[i] * dq[i]);
                std::cout << "Joint [" << i << "] reach upper limit at time: " << ros::Time::now() << std::endl;
            }
            else {
                tau_wall[i] = 0.0;
                // std::cout << "All joints are within range limitation." << std::endl;
            }
        }

    }

    /* null space handling */
    void nullHandling(int* status) {

        Eigen::MatrixXd jacobian_transpose_pinv;
        double nullspace_stiffness_ = 1.0;

        th_nullspace_running = true;

        ros::Rate loop_rate(500);
        while (!done && *status == 0) {
            pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv, true);
            // nullspace PD control with damping ratio = 1
            tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) * 
                        (nullspace_stiffness_ * (q0 - q) - (2.0 * std::sqrt(nullspace_stiffness_)) * dq);
            
            loop_rate.sleep();
            // std::cout << tau_nullspace.transpose() << std::endl;
        }

    }

    private:
    
    /* damped pseudo inverse function */
    void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {

        double lambda_ = damped ? 0.2 : 0.0;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
        Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
        S_.setZero();

        for (int i = 0; i < sing_vals_.size(); i++)
            S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

        M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
    }

    Eigen::Matrix<double, 7, 1> torque_regulation(Eigen::Matrix<double, 7, 1> val, Eigen::Matrix<double, 7, 1> last_val, double ratio) {
        Eigen::Matrix<double, 7, 1> result;
        std::array<double, 7> limited_val{};
        std::array<double, 7> val_derivatives{};
        double max_derivatives = 1000.0 * ratio;
        double max = 100.0 * ratio;
        double min = -100.0 * ratio;
        for (int i = 0; i < 7; i ++) {
            val_derivatives[i] = (val[i] - last_val[i]) / sample_time;
            limited_val[i] = last_val[i] + std::max(std::min(val_derivatives[i], max_derivatives), -max_derivatives) * sample_time;
            result[i] = std::min(std::max(limited_val[i], min), max);
        }
        return result;
    }

    Eigen::Matrix<double, 6, 1> force_regulation(Eigen::Matrix<double, 6, 1> val, double ratio) {
        Eigen::Matrix<double, 6, 1> result;
        std::array<double, 6> min = {{-125.0, -100.0, -50.0, -10.0, -10.0, -10.0}};
        std::array<double, 6> max = {{95.0, 100.0, 150.0, 10.0, 10.0, 10.0}};

        for (int i = 0; i < 6; i ++) {
            result[i] = std::min(std::max(val[i], min[i] * ratio), max[i] * ratio);
        }
        return result;
    }
};

/* Panda teleop interface */
PTINode::PTINode(ros::NodeHandle& node, std::string type): node_type(type) {
    nh_ = node;

    if (node_type == "Right") {
        ROS_INFO("Launch ros interface as right panda");
        pti_packet_sub = nh_.subscribe("/right_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
        pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_right_output", 1);
    }
    else if (node_type == "Left") {
        ROS_INFO("Launch ros interface as left panda");
        pti_packet_sub = nh_.subscribe("/left_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
        pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_left_output", 1);
    }

    ROS_INFO("Node initialized");
}

// PTINode::~PTINode() = default;

/* Publisher */
void PTINode::publish_ptipacket() {
    franka_control::PTIPacket packet_msg;
    packet_msg.wave.resize(3);
    packet_msg.test.resize(7);

    // mtx.lock();
    for (int i = 0; i < 3; i ++) {
        packet_msg.wave[i] = wave_out[i];
    }
    for (int i = 0; i < 7; i ++) {
        packet_msg.test[i] = tau[i];
    }
    packet_msg.position.x = position_relative[0];
    packet_msg.position.y = position_relative[1];
    packet_msg.position.z = position_relative[2];
    packet_msg.angle.x = -angle_relative[0];
    packet_msg.angle.y = -angle_relative[1];
    packet_msg.angle.z = -angle_relative[2];
    packet_msg.twist.linear.x = twist[0];
    packet_msg.twist.linear.y = twist[1];
    packet_msg.twist.linear.z = twist[2];
    packet_msg.twist.angular.x = twist[3];
    packet_msg.twist.angular.y = twist[4];
    packet_msg.twist.angular.z = twist[5];
    // mtx.unlock();
    packet_msg.timestamp = ros::Time::now().toSec();

    // if (pti_packet_pub.getNumSubscribers() == 0) {
    //     if (node_type == "Right") {
    //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
    //         pti_packet_pub.shutdown();
    //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_right_output", 1);
    //         // pti_packet_sub = nh_.subscribe("/right_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
    //     }
    //     else if (node_type == "Left") {
    //         // ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
    //         pti_packet_pub.shutdown();
    //         pti_packet_pub = nh_.advertise<franka_control::PTIPacket>("/pti_left_output", 1);
    //         // pti_packet_sub = nh_.subscribe("/left_smarty_arm_output", 1, &PTINode::ptipacket_callback, this, ros::TransportHints().udp());
    //     }
    // }

    pti_packet_pub.publish(packet_msg);

}

/* Subscriber callback */
void PTINode::ptipacket_callback(const franka_control::PTIPacket::ConstPtr &packet_msg) {

    // mtx.lock();
    for (int i = 0; i < 3; i ++) {
        wave_in[i] = packet_msg->wave[i];
    }
    position_in << packet_msg->position.x, packet_msg->position.y, packet_msg->position.z;
    angle_in << packet_msg->angle.x / 2.0, packet_msg->angle.y / 2.0, packet_msg->angle.z / 2.0;
    twist_in << packet_msg->twist.linear.x, packet_msg->twist.linear.y, packet_msg->twist.linear.z,\
                    packet_msg->twist.angular.x / 2.0, packet_msg->twist.angular.y / 2.0, packet_msg->twist.angular.z / 2.0;
    // mtx.unlock();
    delay_cycle = (int)((ros::Time::now().toSec() - packet_msg->timestamp) / 1e-3 / 2);
    // ROS_INFO_THROTTLE(1, "Write into pti memory");
}

/* Run loop */
void PTINode::ros_run(int* status) {
    th_ros_running = true;
    ros::Rate loop_rate(1000);
    while (!done && *status == 0) {
        // signal(SIGINT, signal_callback_handler);
        publish_ptipacket();
        ros::spinOnce();
        loop_rate.sleep();
    }
    wave_out.setZero();
    publish_ptipacket();
}

/* Slow catching */
void PTINode::slow_catching(void) {
    int num = 3;
    double lambda = 10.0;
    double translation_stiffness = 3000.0;
    double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
    double rotation_stiffness = 300.0;
    double rotation_damping = 2.0 * 1.0 * std::sqrt(rotation_stiffness * 0.01);
    double torque_ratio = 0.9;
    double force_ratio = 0.2;

    double regulate_translation_velocity = 5.0;
    double regulate_rotation_velocity = 8.0;

    Eigen::Vector3d position_target;
    Eigen::Vector3d angle_target;

    for (int i = 0; i < 3; i ++) {
        if (position_in[i] > position_relative[i]) {
            position_target[i] = std::min(position_in[i], position_relative[i] + regulate_translation_velocity * sample_time);
        }
        else {
            position_target[i] = std::max(position_in[i], position_relative[i] - regulate_translation_velocity * sample_time);
        }

        if (angle_in[i] > -angle_relative[i]) {
            angle_target[i] = std::min(angle_in[i], -angle_relative[i] + regulate_rotation_velocity * sample_time); 
        }
        else {
            angle_target[i] = std::max(angle_in[i], -angle_relative[i] - regulate_rotation_velocity * sample_time);
        }
    }

    // std::cout << angle_in.transpose() + angle_relative.transpose() << std::endl;

    force.head(3) = translation_stiffness * (position_target - position_relative) + translation_damping * (-twist.head(3));

    force.tail(3) = rotation_stiffness * (angle_target + angle_relative) + rotation_damping * (-twist.tail(3));
    force = force_regulation(force, force_ratio);
    
    tau = jacobian.transpose() * force + coriolis + tau_hose + tau_wall;
    tau = torque_regulation(tau, last_tau, torque_ratio);
    last_tau = tau;

}

/* panda control */
void panda_control(PTINode& pti, std::string type, std::string ip, int* status) {

    std::thread th_nullspaceControl;
    std::thread th_ros;

    std::cout << type << " node starts running" << std::endl;

    try {
        // connect to robot
        franka::Robot robot(ip);
        robot.automaticErrorRecovery();
        setDefaultBehavior(robot);

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        
        // set external load
        if (type == "Right") {
            const double load_mass = 0.9; // 2.5 fully filled
            // const std::array< double, 3 > F_x_Cload = {{0.03, -0.01, -0.06}};
            // const std::array< double, 9 > load_inertia = {{0.01395, 0.0, 0.0, 0.0, 0.01395, 0.0, 0.0, 0.0, 0.00125}};
            const std::array< double, 3 > F_x_Cload = {{0.0, 0.0, 0.0}};
            const std::array< double, 9 > load_inertia = {{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
            robot.setLoad(load_mass, F_x_Cload, load_inertia);
        }
        else if (type == "Left") {
            const double load_mass = 0.9;
            // const std::array< double, 3 > F_x_Cload = {{0.03, -0.01, -0.06}};
            // const std::array< double, 9 > load_inertia = {{0.01395, 0.0, 0.0, 0.0, 0.01395, 0.0, 0.0, 0.0, 0.00125}};
            const std::array< double, 3 > F_x_Cload = {{0.0, 0.0, 0.0}};
            const std::array< double, 9 > load_inertia = {{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
            robot.setLoad(load_mass, F_x_Cload, load_inertia);
        }
        
        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal;
        if (type == "Right") {
            q_goal = std::array<double, 7>{{0.0, 0.0, 0.0, -2.7, 1.5708, 1.5708, -2.0}};
        }
        else if (type == "Left") {
            q_goal = std::array<double, 7>{{0.0, 0.0, 0.0, -2.7, -1.5708, 1.5708, 0.4}};
        }
        MotionGenerator motion_generator(0.3, q_goal);
        std::cout << "WARNING: " << type << " arm starts moving."
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        if (type == "Right") {
            std::array<double, 16> ee = {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0}};
            robot.setEE(ee);
        }
        else if (type == "Left") {
            std::array<double, 16> ee = {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0}};
            robot.setEE(ee);
        }

        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();
        pti.initial_state = robot.readOnce();

        pti.teleInit(model);
        std::cout << type << " PTI class initialized" << std::endl;

        if (type == "Right") {
            pti.hose_gravity[2] = 8.0;
        }
        else if (type == "Left") {
            pti.hose_gravity[2] = 10.0;
        }

        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

        // slow catching smarty arm
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [&pti, &model, zero_torques, type](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
        {
            /* ctrl+c finish running */
            if (done) {
                std::cout << std::endl << "Ctrl + c entered, shutting down slow catching" << std::endl;
                return franka::MotionFinished(zero_torques);
            }
            pti.robotStateUpdate(model, robot_state);
            pti.jointLimit(type);
            pti.slow_catching();

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = pti.tau;

            double position_error = std::sqrt((pti.position_in[0] - pti.position_relative[0]) * (pti.position_in[0] - pti.position_relative[0])
                                            + (pti.position_in[1] - pti.position_relative[1]) * (pti.position_in[1] - pti.position_relative[1])
                                            + (pti.position_in[2] - pti.position_relative[2]) * (pti.position_in[2] - pti.position_relative[2]));
            double angle_error = std::sqrt((pti.angle_in[0] + pti.angle_relative[0]) * (pti.angle_in[0] + pti.angle_relative[0])
                                        + (pti.angle_in[1] + pti.angle_relative[1]) * (pti.angle_in[1] + pti.angle_relative[1])
                                        + (pti.angle_in[2] + pti.angle_relative[2]) * (pti.angle_in[2] + pti.angle_relative[2]));

            double position_threshold = 0.02;
            double angle_threshold = 0.05;

            std::cout << "position error: " << position_error << " , angle error: " << angle_error << std::endl;
            if ((position_error < position_threshold) && (angle_error < angle_threshold)) {
                std::cout << std::endl << "Homing position reached, starting teleop" << std::endl;
                pti.tau = pti.coriolis;
                franka::Torques homing_stop_torque{{pti.tau[0], pti.tau[1], pti.tau[2], pti.tau[3], pti.tau[4], pti.tau[5], pti.tau[6]}};
                return franka::MotionFinished(homing_stop_torque);
            }

            return tau_d_array;
        };

        th_nullspaceControl = std::thread([&pti, status](){pti.nullHandling(status);});
        th_ros = std::thread([&pti, status](){pti.ros_run(status);});
        robot.control(impedance_control_callback);

        pti.position_d = pti.position_in;
        pti.last_tau.setZero();

        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            tele_control_callback = [&pti, &model, zero_torques, type](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
        {
            /* ctrl+c finish running */
            if (done) {
                std::cout << std::endl << "Finished task, shutting down operation" << std::endl;
                return franka::MotionFinished(zero_torques);
            }

            pti.robotStateUpdate(model, robot_state);
            pti.jointLimit(type);
            pti.sTeleController();

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = pti.tau;
            return tau_d_array;
        };

        std::cout << type << " panda teleop controller starts running" << std::endl;
        robot.control(tele_control_callback);

        th_ros.join(); pti.th_ros_running = false;
        th_nullspaceControl.join(); pti.th_nullspace_running = false;
        *status = 0;

    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        *status = -1;
        std::cout << type << " arm error, waiting for recovery..." << std::endl;

        // std::cout << *status << std::endl;

        if (pti.th_nullspace_running) {
            th_nullspaceControl.join();
            pti.th_nullspace_running = false;
        }
        if (pti.th_ros_running) {
            th_ros.join();
            pti.th_ros_running = false;
        }
        std::cout << "All thread shut down" << std::endl;
        
    }

}

/* auto-recovery */
void arm_run(PTINode& pti, std::string type, std::string ip, int* status) {

    // PTINode pti(node, type);

    while(!done) {
        panda_control(pti, type, ip, status);
        if (*status == 0) {
            std::cout << type << " panda control stop" << std::endl;
            break;
        }
        else if (*status == -1) {
            std::cout << type << " panda restarts. Pressure enter to continue" << std::endl;
            // std::cin.ignore();
        }
        *status = 0;
    }
    std::cout << type << " panda program stops." << std::endl;
}


int main(int argc, char** argv) {

    int status = 0;

    signal(SIGINT, signal_callback_handler);

    // Check whether the required arguments were passed
    if (argc != 2) {
        std::cout << argc << std::endl;
        std::cerr << "Usage: avatar_panda && left or right" << std::endl;
        return -1;
    }

    // ros::init(argc, argv, "pti_interface", ros::init_options::NoSigintHandler);

    if (std::string(argv[1]) == "left") {
        ros::init(argc, argv, "pti_interface_left", ros::init_options::NoSigintHandler);
        ros::NodeHandle node("~");
        std::string ip = "10.180.1.100";
        PTINode pti(node, "Left");
        arm_run(pti, "Left", ip, &status);
    }
    else if (std::string(argv[1]) == "right") {
        ros::init(argc, argv, "pti_interface_right", ros::init_options::NoSigintHandler);
        ros::NodeHandle node("~");
        std::string ip = "10.180.1.101";
        PTINode pti(node, "Right");
        arm_run(pti, "Right", ip, &status);
    }
    else {
        std::cout << "Error: left or right arm not configured." << std::endl;
        return -1;
    }

    // std::thread left_arm_run([&pti_left, ip_left, &left_status](){arm_run(pti_left, "Left", ip_left, &left_status);});
    // arm_run(pti_right, "Right", ip_right, &right_status);
    // arm_run(left_node, "Left", ip_left, &left_status);
    // left_arm_run.join();

    ros::shutdown();

    std::cout << "Done." << std::endl;
    
    return 0;
}
