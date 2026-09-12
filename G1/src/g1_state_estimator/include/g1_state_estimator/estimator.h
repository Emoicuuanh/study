#pragma once
#include "state.h"
#include <map>
#include <vector>
#include <mutex>
#include <memory>

class Estimator {
public:
    Estimator();
    ~Estimator() = default;

    // Initialize state and covariance
    void Initialize(const State& initial_state, const Eigen::Matrix<double, 15, 15>& initial_cov);

    // Set noise parameters
    void SetNoiseParameters(double gyro_noise, double acc_noise, 
                            double gyro_bias_noise, double acc_bias_noise,
                            const Eigen::Vector3d& lio_pos_noise,
                            const Eigen::Vector3d& lio_ori_noise,
                            const Eigen::Vector3d& leg_vel_noise);

    // Set maximum history buffer length in seconds
    void SetMaxHistoryDuration(double duration) { max_history_duration_ = duration; }

    // Predict state using IMU measurement
    void Predict(const IMUData& imu, double dt);

    // Update state using Leg Odometry (runs on current state/low latency)
    void UpdateLeg(const LegMeasurement& leg);

    // Update state using delayed LIO measurement (runs rollback)
    bool UpdateLIO(const LIOMeasurement& lio);

    // Accessors
    State GetState() const;
    Eigen::Matrix<double, 15, 15> GetCovariance() const;
    bool IsInitialized() const { return is_initialized_; }

private:
    bool is_initialized_ = false;
    State state_;
    Eigen::Matrix<double, 15, 15> P_; // 15x15 Error-state covariance matrix

    // Noise parameters (continuous-time PSD or discrete variance)
    double q_gyro_ = 1e-4;
    double q_acc_ = 1e-3;
    double q_bg_ = 1e-6;
    double q_ba_ = 1e-5;

    // Measurement noise parameters (variance)
    Eigen::Vector3d r_lio_pos_ = Eigen::Vector3d::Constant(1e-4);
    Eigen::Vector3d r_lio_ori_ = Eigen::Vector3d::Constant(1e-4);
    Eigen::Vector3d r_leg_vel_ = Eigen::Vector3d::Constant(1e-3);

    // History buffers
    std::map<double, State> state_history_;
    std::map<double, Eigen::Matrix<double, 15, 15>> cov_history_;
    std::map<double, IMUData> imu_history_;

    double max_history_duration_ = 2.0; // Keep 2 seconds of history

    mutable std::mutex mutex_;

    // Helper functions
    void PruneHistory(double latest_time);
    bool RollbackAndRePropagate(double t_meas, const LIOMeasurement& lio);

    // Skew-symmetric matrix helper
    Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
             -v.y(), v.x(), 0.0;
        return m;
    }
};
