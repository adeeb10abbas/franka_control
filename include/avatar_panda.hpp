#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <csignal>
#include <cstdlib>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <chrono>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include "franka_control/msg/pti_packet.hpp"
#include "franka_control/msg/p_info.hpp"
volatile sig_atomic_t done = 0;

void signal_callback_handler(int signum) {
    std::cout << "CTRL+C interrupted. " << std::endl;
    if (signum == SIGINT) {
        done = 1;
    }
}

class PTINode : public rclcpp::Node {
public:
    explicit PTINode(std::string type);

    void ros_run(int* status);
    void slow_catching(void);

    rclcpp::Subscription<franka_control::msg::PTIPacket>::SharedPtr pti_packet_sub;
    rclcpp::Publisher<franka_control::msg::PTIPacket>::SharedPtr pti_packet_pub;
    rclcpp::Publisher<franka_control::msg::PInfo>::SharedPtr pinfo_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_joint_state_pub;

    std::string node_type;

    void publish_ptipacket();
    void publish_pinfo();
    void publish_robot_joint_state();
    void ptipacket_callback(const franka_control::msg::PTIPacket::SharedPtr msg);

    std::mutex mtx;

    bool th_nullspace_running = false;
    bool th_ros_running = false;
    int slow_catching_flag = 0;

    /* robot joint state */
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> q0;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> tau;
    Eigen::Matrix<double, 7, 1> last_tau;
    Eigen::Matrix<double, 7, 1> tau_measured;
    Eigen::Matrix<double, 7, 1> tau_ext;
    Eigen::Matrix<double, 7, 1> gravity;
    std::vector<std::string> joint_names; 
    Eigen::Matrix<double, 7, 1> tau_nullspace;
    Eigen::Matrix<double, 7, 1> tau_wall;
    Eigen::Matrix<double, 7, 1> tau_hose;
    Eigen::Matrix<double, 6, 1> hose_gravity;

    /* robot initial state */
    franka::RobotState initial_state;
    Eigen::Affine3d initial_transform;
    Eigen::Vector3d position_0;
    Eigen::Quaterniond orientation_0;
    Eigen::Matrix<double, 7, 1> initial_tau_ext;

    /* robot end-effector state */
    Eigen::Affine3d transform;
    Eigen::Vector3d position;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position_relative;
    Eigen::Vector3d quat_error;
    Eigen::Matrix<double, 6, 1> twist;
    Eigen::Matrix<double, 6, 1> twist_d;
    Eigen::Matrix<double, 6, 1> force;
    Eigen::Matrix<double, 6, 1> est_ext_force;
    Eigen::Matrix<double, 7, 7> inertia;
    double robot_translation_mass;

    Eigen::Matrix<double, 7, 1> coriolis;
    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 7> joint5_jacobian;

    /* wave variable */
    Eigen::Vector3d wave_in;
    Eigen::Vector3d wave_in_prev;
    Eigen::Vector3d wave_out;
    Eigen::Vector3d wave_integral;
    Eigen::Vector3d position_in;
    Eigen::Vector3d quat_in;
    Eigen::Vector3d quat_in_prev;
    Eigen::Vector3d filtered_quat_in;
    Eigen::Vector3d filtered_quat_in_prev;
    Eigen::Matrix<double, 6, 1> twist_in;
    Eigen::Matrix<double, 6, 1> twist_in_prev;
    Eigen::Matrix<double, 6, 1> filtered_twist_in;
    Eigen::Matrix<double, 6, 1> filtered_twist_in_prev;
    double wave_damping;
    double wave_filter_freq;

    int delay_current_index;
    int delay_cycle_previous;
    int max_buff_size = 2000;
    double wave_history[3][2000];
    int delay_cycle;
    double remote_time;
    double sample_time;
    
    // void ptipacket_callback(const franka_control::msg::PTIPacket::SharedPtr packet_msg);
    /* panda initialization */
    void teleInit(franka::Model& model) {

        initial_transform = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
        position_0 = initial_transform.translation();
        orientation_0 = initial_transform.linear();

        q0 = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());

        std::cout << q0.transpose() << std::endl;

        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(model.zeroJacobian(franka::Frame::kEndEffector, initial_state).data());
        joint5_jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(model.zeroJacobian(franka::Frame::kJoint5, initial_state).data());

        std::array<double, 49> initial_inertia_array = model.mass(initial_state);
        inertia = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(initial_inertia_array.data());
        robot_translation_mass = 5.0;

        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(model.gravity(initial_state).data());
        initial_tau_ext = initial_tau_measured - initial_gravity;

        position_d.setZero();
        twist_d.setZero();
        wave_in.setZero();
        wave_in_prev.setZero();
        wave_integral.setZero();
        position_in.setZero();
        quat_in.setZero();
        quat_in_prev.setZero();
        filtered_quat_in.setZero();
        filtered_quat_in_prev.setZero();
        twist_in.setZero();
        twist_in_prev.setZero();
        filtered_twist_in.setZero();
        filtered_twist_in_prev.setZero();
        tau_nullspace.setZero();
        tau.setZero();
        last_tau.setZero();
        tau_wall.setZero();

        wave_damping = 10.0;
        wave_filter_freq = 2.0 * M_PI * 20.0;

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
        remote_time = this->get_clock()->now().seconds();

        hose_gravity.setZero();
    }

    /* update robot endeffector state */
    void robotStateUpdate(franka::Model& model, const franka::RobotState& robot_state) {

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        std::array<double, 42> joint5_jacobian_array = model.zeroJacobian(franka::Frame::kJoint5, robot_state);
        std::array<double, 49> inertia_array = model.mass(robot_state);

        coriolis = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
        jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
        joint5_jacobian = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(joint5_jacobian_array.data());
        q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
        dq = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

        est_ext_force = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());
        inertia = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(inertia_array.data());

        tau_measured = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
        gravity = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(model.gravity(robot_state).data());
        tau_ext = tau_measured - gravity - initial_tau_ext;

        // std::cout << q.transpose()<< std::endl;

        transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        position = transform.translation();
        orientation = transform.linear();
        position_relative = position - position_0;

        Eigen::Quaterniond orientation_relative;
        Eigen::Vector3d quat_error_local;
        if (orientation_0.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        orientation_relative = orientation.inverse() * orientation_0;
        quat_error_local << orientation_relative.x(), orientation_relative.y(), orientation_relative.z();
        quat_error = transform.linear() * quat_error_local;

        // std::cout << quat_error.transpose() << std::endl;

        twist = jacobian * dq;
        tau_hose = joint5_jacobian.transpose() * hose_gravity;
    }

    /* slave wave variable controller */
    void sTeleController(void) {

        int num = 3;
        double lambda = 1.0;
        double translation_stiffness = 1200.0;
        double translation_damping = 2.0 * 1.0 * std::sqrt(translation_stiffness * 1.0);
        double rotation_stiffness = 100.0;
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
        delay_cycle_current = 1;

        // translation part with wave variable
        position_d += twist_d.head(3) * sample_time;

        // position drift correction
        actual_position_error = position_in - position_d;
        predict_position_error = -1.0 / std::sqrt(2.0 * wave_damping) * wave_integral;
        error_difference = predict_position_error - actual_position_error;
        wave_correction = -1.0 * std::sqrt(2.0 * wave_damping) * (2.0 * M_PI * lambda) * error_difference;
        for (int i = 0; i < num; i ++) {
            if (wave_correction[i] * wave_in[i] < 0) {
                if (std::fabs(wave_correction[i]) < std::fabs(wave_in[i])) {
                    wave_in[i] += wave_correction[i];
                }
                else {
                    wave_in[i] = 0.0;
                }
            }


        twist_d.head(3) = (std::sqrt(2.0 * wave_damping) * wave_in + translation_damping * twist.head(3) + \
                        translation_stiffness * (position_relative - position_d)) / (wave_damping + translation_damping);
        force.head(3) = translation_stiffness * (position_d - position_relative) + translation_damping * (twist_d.head(3) - twist.head(3));
        wave_out = wave_in - std::sqrt(2.0 / wave_damping) * force.head(3);

        delay_index = delay_current_index - delay_cycle_current;
        delay_difference = delay_cycle_current - delay_cycle_previous;
        if (delay_index < 0) {
            delay_index += max_buff_size;
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
        force.tail(3) = rotation_stiffness * (filtered_quat_in + quat_error) + rotation_damping * (filtered_twist_in.tail(3) - twist.tail(3));
        force = force_regulation(force, force_ratio);
        
        tau = jacobian.transpose() * force + coriolis + tau_nullspace + tau_wall + tau_hose;
        // tau = jacobian.transpose() * force + coriolis + tau_wall + tau_hose;
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
                std::cout << "Joint [" << i << "] reach lower limit at time: " << this->get_clock()->now().seconds() << std::endl;
            }
            else if (q[i] > q_max_radian[i]) {
                tau_wall[i] = (k_gains[i] * (q_max_radian[i] - q[i]) - d_gains[i] * dq[i]);
                std::cout << "Joint [" << i << "] reach upper limit at time: " << this->get_clock()->now().seconds() << std::endl;
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
        double nullspace_stiffness_ = 10.0;
        Eigen::MatrixXd M_ee;

        rclcpp::Rate loop_rate(1000); // 1000 Hz
        while (!done && *status == 0 && rclcpp::ok()) {
            // Assuming dynamicallyConsistentGeneralizedInverse() is a function defined elsewhere
            dynamicallyConsistentGeneralizedInverse(jacobian, inertia, jacobian_transpose_pinv, M_ee);
            
            // nullspace PD control with damping ratio = 1
            tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                            (nullspace_stiffness_ * (q0 - q) - (2.0 * std::sqrt(nullspace_stiffness_)) * dq);
            
            robot_translation_mass = (M_ee(0,0) + M_ee(0,1) + M_ee(0,2) + M_ee(1,0) + M_ee(1,1) + M_ee(1,2) + M_ee(2,0) + M_ee(2,1) + M_ee(2,2)) / 3;
            
            loop_rate.sleep();
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

    /* dynamically_consistent_generalized_inverse */
    void dynamicallyConsistentGeneralizedInverse(const Eigen::MatrixXd& J_, const Eigen::MatrixXd& M_, Eigen::MatrixXd& J_dcginv_, Eigen::MatrixXd& M_ee_) {
        Eigen::MatrixXd M_inv_ = M_.inverse();
        Eigen::MatrixXd M_ee_inv_ = J_ * M_inv_ * J_.transpose();
        
        M_ee_ = M_ee_inv_.inverse();
        J_dcginv_ = M_ee_inv_.inverse() * J_ * M_inv_;
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

    double firstOrderIIRFilter(double input, double input_prev, double output_prev) {
        double output;
        double b0 = 2.0 * M_PI * 2.0 * 5e-3 / (2.0 + 2.0 * M_PI * 2.0 * 5e-3);
        double b1 = b0;
        double a1 = (2.0 - 2.0 * M_PI * 2.0 * 5e-3) / (2.0 + 2.0 * M_PI * 2.0 * 5e-3);
        output = b0 * input + b1 * input_prev + a1 * output_prev;
        return output;
    }
};
