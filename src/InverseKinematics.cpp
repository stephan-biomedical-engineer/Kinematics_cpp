#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR> // Add this line to include the QR decomposition header
#include <cmath>
#include "InverseKinematics.h"

InverseKinematics::InverseKinematics(const ForwardKinematicsDH& fk, int max_iterations, double tolerance)
    : fk_(fk), max_iterations_(max_iterations), tolerance_(tolerance) {}

void InverseKinematics::error_counter() const {
    std::cout << "Number of iterations: " << error_count_ << std::endl;
}

Eigen::VectorXd InverseKinematics::compute_ik(const Eigen::Vector3d& desired_position) {
    Eigen::VectorXd current_joint_angles(fk_.joints_.size());
    for (int i = 0; i < current_joint_angles.size(); ++i) {
        current_joint_angles[i] = fk_.joints_[i].theta;
    }

    error_count_ = 0;  // Reset error count for each call
    for (int i = 0; i < max_iterations_; ++i) {
        Eigen::Matrix4d current_end_effector = fk_.compute_end_effector();
        Eigen::Vector3d current_position = current_end_effector.block<3, 1>(0, 3);

        Eigen::Vector3d error = desired_position - current_position;

        if (error.norm() < tolerance_) {
            return current_joint_angles;  // Solution found
        }

        error_count_++;  // Increment error count

        Eigen::MatrixXd J = fk_.compute_jacobian();
        // Use Eigen QR decomposition for efficient pseudoinverse calculation
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J.block<3, Eigen::Dynamic>(0, 0, 3, fk_.joints_.size()));
        Eigen::VectorXd delta_theta = qr.solve(error);

        current_joint_angles += delta_theta;

        // Enforce joint angle limits (optional):
        // for (int i = 0; i < current_joint_angles.size(); ++i) {
        //   // Implement logic to enforce joint limits here
        // }
    }

    throw std::runtime_error("Inverse kinematics failed to converge.");
}
