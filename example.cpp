#include "ForwardKinematicsDH.h"
#include "InverseKinematics.h"
#include <iostream>

int main() {
    // Joint(d(cm),a(cm),theta(deg),alpha(deg))
    Joint Base(1, 0, 0, -90);
    Joint Shoulder(0, 19.5, 180, 0);
    Joint Elbow(0, 24.3, 25, 0);
    Joint Wrist1(0, 0, 10, 90);
    Joint Wrist2(35, 0, 0, 0);

    std::vector<Joint> joints = {Base, Shoulder, Elbow, Wrist1, Wrist2};
    ForwardKinematicsDH fk(joints);

    Eigen::Matrix4d end_effector = fk.compute_end_effector();
    std::cout << "End effector matrix: " << std::endl << end_effector << std::endl;

    Eigen::MatrixXd jacobian = fk.compute_jacobian();
    std::cout << "Jacobian: " << std::endl << jacobian << std::endl;

    InverseKinematics ik(fk);
    Eigen::Vector3d desired_position(10.0, 10.0, 10.0);  // Ajuste para uma posição razoável
    try {
        Eigen::VectorXd joint_angles = ik.compute_ik(desired_position);
        std::cout << "Joint angles: " << std::endl << joint_angles << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
