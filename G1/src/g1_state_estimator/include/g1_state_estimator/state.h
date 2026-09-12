#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct State {
    double timestamp = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();         // Position in global frame
    Eigen::Vector3d v = Eigen::Vector3d::Zero();         // Velocity in global frame
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // Orientation in global frame
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();        // Gyroscope bias
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();        // Accelerometer bias

    // Boxplus operator for manifold addition: x_new = x ⊕ dx
    void boxplus(const Eigen::Matrix<double, 15, 1>& dx) {
        p += dx.segment<3>(0);
        v += dx.segment<3>(3);
        
        Eigen::Vector3d dtheta = dx.segment<3>(6);
        if (dtheta.norm() > 1e-12) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
            q = (q * dq).normalized();
        }
        
        bg += dx.segment<3>(9);
        ba += dx.segment<3>(12);
    }
};

struct IMUData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct LIOMeasurement {
    double timestamp;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

struct LegMeasurement {
    double timestamp;
    Eigen::Vector3d v_body; // Linear velocity in body-frame
};
