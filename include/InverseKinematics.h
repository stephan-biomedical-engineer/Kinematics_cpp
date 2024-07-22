#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "ForwardKinematicsDH.h"
#include <Eigen/Dense>

class InverseKinematics {
public:
    InverseKinematics(const ForwardKinematicsDH& fk, int max_iterations = 100, double tolerance = 1e-6);

    void error_counter() const;
    Eigen::VectorXd compute_ik(const Eigen::Vector3d& desired_position);

private:
    const ForwardKinematicsDH& fk_;
    int max_iterations_;
    double tolerance_;
    mutable int error_count_;  // Tracks iterations for each call (mutable for modification)
};

#endif // INVERSEKINEMATICS_H
