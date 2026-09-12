#include "g1_state_estimator/estimator.h"
#include <iostream>
#include <algorithm>

Estimator::Estimator() {
    state_ = State();
    P_ = Eigen::Matrix<double, 15, 15>::Identity() * 1e-2;
}

void Estimator::Initialize(const State& initial_state, const Eigen::Matrix<double, 15, 15>& initial_cov) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = initial_state;
    P_ = initial_cov;
    
    state_history_.clear();
    cov_history_.clear();
    imu_history_.clear();

    state_history_[state_.timestamp] = state_;
    cov_history_[state_.timestamp] = P_;
    is_initialized_ = true;
}

void Estimator::SetNoiseParameters(double gyro_noise, double acc_noise, 
                                    double gyro_bias_noise, double acc_bias_noise,
                                    const Eigen::Vector3d& lio_pos_noise,
                                    const Eigen::Vector3d& lio_ori_noise,
                                    const Eigen::Vector3d& leg_vel_noise) {
    std::lock_guard<std::mutex> lock(mutex_);
    q_gyro_ = gyro_noise;
    q_acc_ = acc_noise;
    q_bg_ = gyro_bias_noise;
    q_ba_ = acc_bias_noise;
    r_lio_pos_ = lio_pos_noise;
    r_lio_ori_ = lio_ori_noise;
    r_leg_vel_ = leg_vel_noise;
}

void Estimator::Predict(const IMUData& imu, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;

    if (dt <= 0.0 || dt > 0.1) return; // Guard against abnormal time steps

    // 1. Nominal state integration
    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    Eigen::Vector3d unbiased_acc = imu.acc - state_.ba;
    Eigen::Vector3d unbiased_gyro = imu.gyro - state_.bg;
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    // Position integration
    state_.p += state_.v * dt + 0.5 * (R * unbiased_acc + gravity) * dt * dt;

    // Velocity integration
    state_.v += (R * unbiased_acc + gravity) * dt;

    // Orientation integration (using exponential map / quaternion multiplication)
    Eigen::Vector3d angle_axis = unbiased_gyro * dt;
    if (angle_axis.norm() > 1e-12) {
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized()));
        state_.q = (state_.q * dq).normalized();
    }
    
    // Bias integration (random walk)
    // Nominal state assumes bias is constant, error state models their random walk

    // Update timestamp
    state_.timestamp = imu.timestamp;

    // 2. Covariance propagation (P_new = F * P * F^T + Q)
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * SkewSymmetric(unbiased_acc) * dt;
    F.block<3, 3>(3, 12) = -R * dt;
    
    // exp(-w*dt) Jacobian approximation
    F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - SkewSymmetric(unbiased_gyro) * dt;
    F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_acc_ * dt;
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * q_gyro_ * dt;
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * q_bg_ * dt;
    Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_ba_ * dt;

    P_ = F * P_ * F.transpose() + Q;

    // 3. Save to history
    state_history_[imu.timestamp] = state_;
    cov_history_[imu.timestamp] = P_;
    imu_history_[imu.timestamp] = imu;

    // Prune history buffer to prevent memory leakage
    PruneHistory(imu.timestamp);
}

void Estimator::UpdateLeg(const LegMeasurement& leg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) return;

    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    
    // V_body_est = R^T * V_global
    Eigen::Vector3d v_body_est = R.transpose() * state_.v;
    
    // Measurement Residual: r = z - h(x)
    Eigen::Vector3d r = leg.v_body - v_body_est;

    // Measurement Jacobian H (3x15)
    // H = [ 0_3x3  R^T  [R^T * V_global]_x  0_3x3  0_3x3 ]
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = R.transpose();
    H.block<3, 3>(0, 6) = SkewSymmetric(R.transpose() * state_.v);

    // Measurement Covariance
    Eigen::Matrix3d R_meas = r_leg_vel_.asDiagonal();

    // Kalman gain
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_meas;
    Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

    // Update state and covariance
    Eigen::Matrix<double, 15, 1> dx = K * r;
    state_.boxplus(dx);
    
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K * H) * P_;

    // Update history for the current time
    state_history_[state_.timestamp] = state_;
    cov_history_[state_.timestamp] = P_;
}

bool Estimator::UpdateLIO(const LIOMeasurement& lio) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) {
        // Initialize position and orientation from LIO measurement directly
        state_.timestamp = lio.timestamp;
        state_.p = lio.p;
        state_.q = lio.q;
        state_.v.setZero();
        state_.ba.setZero();
        state_.bg.setZero();
        
        P_ = Eigen::Matrix<double, 15, 15>::Identity() * 1e-2;
        
        state_history_[lio.timestamp] = state_;
        cov_history_[lio.timestamp] = P_;
        is_initialized_ = true;
        return true;
    }

    return RollbackAndRePropagate(lio.timestamp, lio);
}

bool Estimator::RollbackAndRePropagate(double t_meas, const LIOMeasurement& lio) {
    if (state_history_.empty()) return false;

    // 1. Find the closest historical record prior to or equal to measurement timestamp
    auto it_state = state_history_.lower_bound(t_meas);
    if (it_state == state_history_.end()) {
        // If t_meas is newer than all states in history, use the latest
        it_state = std::prev(state_history_.end());
    } else if (it_state != state_history_.begin() && it_state->first > t_meas) {
        // Pick the closest element
        auto prev_it = std::prev(it_state);
        if (std::abs(prev_it->first - t_meas) < std::abs(it_state->first - t_meas)) {
            it_state = prev_it;
        }
    }

    double t_match = it_state->first;
    
    // Check if the time discrepancy is within reasonable bounds (e.g., 0.5s)
    if (std::abs(t_match - t_meas) > 0.5) {
        return false;
    }

    State x_hist = it_state->second;
    Eigen::Matrix<double, 15, 15> P_hist = cov_history_[t_match];

    // 2. Perform EKF update on the historical state x_hist and P_hist using LIO pose
    // Residual for position: r_pos = p_meas - p_est
    Eigen::Vector3d r_pos = lio.p - x_hist.p;

    // Residual for orientation: r_ori = Log(q_est.inverse() * q_meas)
    Eigen::Quaterniond dq = (x_hist.q.inverse() * lio.q).normalized();
    if (dq.w() < 0.0) {
        dq.coeffs() = -dq.coeffs();
    }
    Eigen::AngleAxisd angle_axis(dq);
    Eigen::Vector3d r_ori = angle_axis.axis() * angle_axis.angle();
    
    // Outlier Gating: reject sudden position (>2.0m) or orientation (>1.0 rad) jumps from unstable LIO
    if (r_pos.norm() > 2.0 || r_ori.norm() > 1.0) {
        std::cerr << "[StateEstimator] Rejected LIO measurement outlier! Pos jump: " 
                  << r_pos.norm() << "m, Ori jump: " << r_ori.norm() << " rad" << std::endl;
        return false;
    }

    Eigen::Matrix<double, 6, 1> r;
    r << r_pos, r_ori;

    // Measurement Jacobian H (6x15)
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // H_pos w.r.t p
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(); // H_ori w.r.t theta

    // Measurement Noise Covariance R_lio (6x6)
    Eigen::Matrix<double, 6, 6> R_meas = Eigen::Matrix<double, 6, 6>::Zero();
    R_meas.block<3, 3>(0, 0) = r_lio_pos_.asDiagonal();
    R_meas.block<3, 3>(3, 3) = r_lio_ori_.asDiagonal();

    // Kalman gain
    Eigen::Matrix<double, 6, 6> S = H * P_hist * H.transpose() + R_meas;
    Eigen::Matrix<double, 15, 6> K = P_hist * H.transpose() * S.inverse();

    // Update historical state
    Eigen::Matrix<double, 15, 1> dx = K * r;
    x_hist.boxplus(dx);
    
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_hist = (I - K * H) * P_hist;

    // Save updated historical state back to buffer
    state_history_[t_match] = x_hist;
    cov_history_[t_match] = P_hist;

    // 3. Re-propagate from t_match to the current time using the IMU data history
    auto it_imu = imu_history_.upper_bound(t_match);
    State x_curr = x_hist;
    Eigen::Matrix<double, 15, 15> P_curr = P_hist;
    double t_last = t_match;

    while (it_imu != imu_history_.end()) {
        double t_curr = it_imu->first;
        double dt = t_curr - t_last;
        IMUData imu = it_imu->second;

        if (dt > 0.0 && dt < 0.1) {
            Eigen::Matrix3d R = x_curr.q.toRotationMatrix();
            Eigen::Vector3d unbiased_acc = imu.acc - x_curr.ba;
            Eigen::Vector3d unbiased_gyro = imu.gyro - x_curr.bg;
            Eigen::Vector3d gravity(0.0, 0.0, -9.81);

            // Predict position, velocity, and orientation
            x_curr.p += x_curr.v * dt + 0.5 * (R * unbiased_acc + gravity) * dt * dt;
            x_curr.v += (R * unbiased_acc + gravity) * dt;
            
            Eigen::Vector3d angle_axis_prop = unbiased_gyro * dt;
            if (angle_axis_prop.norm() > 1e-12) {
                Eigen::Quaterniond dq_prop(Eigen::AngleAxisd(angle_axis_prop.norm(), angle_axis_prop.normalized()));
                x_curr.q = (x_curr.q * dq_prop).normalized();
            }
            x_curr.timestamp = t_curr;

            // Predict covariance
            Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(3, 6) = -R * SkewSymmetric(unbiased_acc) * dt;
            F.block<3, 3>(3, 12) = -R * dt;
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - SkewSymmetric(unbiased_gyro) * dt;
            F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

            Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
            Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_acc_ * dt;
            Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * q_gyro_ * dt;
            Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * q_bg_ * dt;
            Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_ba_ * dt;

            P_curr = F * P_curr * F.transpose() + Q;

            // Overwrite updated states in history
            state_history_[t_curr] = x_curr;
            cov_history_[t_curr] = P_curr;
        }

        t_last = t_curr;
        it_imu++;
    }

    // 4. Update the latest estimator state and covariance
    state_ = x_curr;
    P_ = P_curr;

    return true;
}

void Estimator::PruneHistory(double latest_time) {
    double threshold_time = latest_time - max_history_duration_;
    
    // Erase entries older than the threshold
    auto it_state = state_history_.lower_bound(threshold_time);
    if (it_state != state_history_.begin()) {
        state_history_.erase(state_history_.begin(), std::prev(it_state));
    }

    auto it_cov = cov_history_.lower_bound(threshold_time);
    if (it_cov != cov_history_.begin()) {
        cov_history_.erase(cov_history_.begin(), std::prev(it_cov));
    }

    auto it_imu = imu_history_.lower_bound(threshold_time);
    if (it_imu != imu_history_.begin()) {
        imu_history_.erase(imu_history_.begin(), std::prev(it_imu));
    }
}

State Estimator::GetState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

Eigen::Matrix<double, 15, 15> Estimator::GetCovariance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return P_;
}
