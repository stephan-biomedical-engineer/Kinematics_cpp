#include "ForwardKinematicsDH.h"
#include "InverseKinematics.h"
#include <iostream>
#include <chrono>

int main() {
    // Tempo de início
    auto start = std::chrono::high_resolution_clock::now();

    // Joint(d(cm),a(cm),theta(deg),alpha(deg))
    Joint Base(1, 0, 0, -90);
    Joint Shoulder(0, 19.5, 180, 0);
    Joint Elbow(0, 24.3, 25, 0);
    Joint Wrist1(0, 0, 10, 90);
    Joint Wrist2(35, 0, 0, 0);

    std::vector<Joint> joints = {Base, Shoulder, Elbow, Wrist1, Wrist2};
    ForwardKinematicsDH fk(joints);

    Eigen::Matrix4d end_effector = fk.compute_end_effector();
    for (int i = 0; i < end_effector.rows(); ++i) {
        for (int j = 0; j < end_effector.cols(); ++j) {
            end_effector(i, j) = std::round(end_effector(i, j) * 1000) / 1000;
        }
    }

    std::cout << "End effector matrix: " << std::endl << end_effector << std::endl;

    Eigen::MatrixXd jacobian = fk.compute_jacobian();
    for (int i = 0; i < jacobian.rows(); ++i) {
        for (int j = 0; j < jacobian.cols(); ++j) {
            jacobian(i, j) = std::round(jacobian(i, j) * 1000) / 1000;
        }
    }
    
    // std::cout << "Jacobian: " << std::endl << jacobian << std::endl;

    InverseKinematics ik(fk);
    Eigen::Vector3d desired_position(35.0, 30.0, 10.0);  // Ajuste para uma posição razoável
    try {
        Eigen::VectorXd joint_angles = ik.compute_ik(desired_position);
        std::cout << "Joint angles: " << std::endl << joint_angles << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
    }

    ik.error_counter();

    // Tempo de fim
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    return 0;
}
