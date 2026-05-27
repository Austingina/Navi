// chassis_ekf_node.cpp
// =============================================================================
// 4-state EKF chassis odometry node, ported from the original Python
// implementation (now retired) to C++ to drop the Python CPU footprint
// (~100% of one core on Jetson) down to noise. The /cmd_vel ->
// /motion_target/target_speed_chassis bridge that used to live alongside
// the EKF is now in fast_lio/scripts/cmd_vel_bridge.py.
//
// Inputs:
//   /motion_control/chassis_speed (geometry_msgs/TwistStamped) - body velocity
//   /livox/imu                    (sensor_msgs/Imu)            - yaw rate
//   /Odometry                     (nav_msgs/Odometry)          - laser pose
// Output:
//   /odom (nav_msgs/Odometry)
//
// State vector:
//     x = [px, py, theta, b_g]
// Prediction is driven by chassis_speed (typical ~50 Hz) using the latest
// bias-corrected IMU yaw rate. The first /Odometry message snaps the EKF
// origin so /odom shares the same frame definition as the laser odometry
// from that point on; subsequent messages are fused as a linear-H Kalman
// update so position drift is corrected and the gyro yaw bias becomes
// observable.
// =============================================================================

#include <cmath>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace {

// ----------------------- math helpers -------------------------------------

inline double stamp_to_sec(const builtin_interfaces::msg::Time &stamp) {
    return stamp.sec + stamp.nanosec * 1e-9;
}

inline double quat_to_yaw(double qx, double qy, double qz, double qw) {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

inline double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// ----------------------- EKF ----------------------------------------------

struct ChassisEKFConfig {
    // Per-sqrt-second noise on the velocity / gyro inputs used in predict.
    double sigma_vx = 0.05;       // m/s
    double sigma_vy = 0.05;       // m/s
    double sigma_omega = 0.01;    // rad/s
    double sigma_bias = 1e-4;     // rad/s/sqrt(s)
    // /Odometry measurement noise (FAST_LIO doesn't always populate cov).
    double sigma_lio_xy = 0.05;   // m
    double sigma_lio_yaw = 0.02;  // rad
    // Initial covariance prior.
    double init_sigma_pos = 1.0;
    double init_sigma_yaw = 0.5;
    double init_sigma_bias = 0.05;
};

class ChassisEKF {
public:
    ChassisEKFConfig cfg;
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
    bool has_state = false;
    bool lio_anchored = false;

    void reset_prior() {
        x.setZero();
        P.setZero();
        P(0, 0) = cfg.init_sigma_pos * cfg.init_sigma_pos;
        P(1, 1) = cfg.init_sigma_pos * cfg.init_sigma_pos;
        P(2, 2) = cfg.init_sigma_yaw * cfg.init_sigma_yaw;
        P(3, 3) = cfg.init_sigma_bias * cfg.init_sigma_bias;
        has_state = false;
        lio_anchored = false;
    }

    // Integrate one step using measured body velocity + IMU gyro. Uses a
    // midpoint yaw for the position integration to reduce truncation error
    // during sharp turns and long /Odometry outages.
    void predict(double vx, double vy, double omega_imu, double dt) {
        if (dt <= 0.0) return;
        if (dt > 0.5) dt = 0.5;  // cap so a long stall doesn't blow up cov

        const double omega_corr = omega_imu - x(3);
        const double theta = x(2);
        const double theta_mid = theta + 0.5 * omega_corr * dt;
        const double cos_m = std::cos(theta_mid), sin_m = std::sin(theta_mid);

        x(0) += (vx * cos_m - vy * sin_m) * dt;
        x(1) += (vx * sin_m + vy * cos_m) * dt;
        x(2) = wrap_angle(theta + omega_corr * dt);
        // x(3) bias evolves as a random walk in Q only.

        const double cos_t = std::cos(theta), sin_t = std::sin(theta);
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = -(vx * sin_t + vy * cos_t) * dt;
        F(1, 2) =  (vx * cos_t - vy * sin_t) * dt;
        F(2, 3) = -dt;

        const double svxdt2 = (cfg.sigma_vx * dt) * (cfg.sigma_vx * dt);
        const double svydt2 = (cfg.sigma_vy * dt) * (cfg.sigma_vy * dt);
        const double sx2 = svxdt2 * cos_t * cos_t + svydt2 * sin_t * sin_t;
        const double sy2 = svxdt2 * sin_t * sin_t + svydt2 * cos_t * cos_t;
        const double sxy = ((cfg.sigma_vx * cfg.sigma_vx -
                             cfg.sigma_vy * cfg.sigma_vy) * dt * dt)
                           * sin_t * cos_t;
        const double sth2 = (cfg.sigma_omega * dt) * (cfg.sigma_omega * dt);
        const double sb2  = (cfg.sigma_bias * cfg.sigma_bias) * dt;

        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
        Q(0, 0) = sx2; Q(0, 1) = sxy;
        Q(1, 0) = sxy; Q(1, 1) = sy2;
        Q(2, 2) = sth2;
        Q(3, 3) = sb2;

        P = F * P * F.transpose() + Q;
        has_state = true;
    }

    // Linear-H Kalman update with z = [px, py, theta].
    void update_lio(double px, double py, double yaw) {
        Eigen::Matrix<double, 3, 4> H = Eigen::Matrix<double, 3, 4>::Zero();
        H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0;

        Eigen::Vector3d z(px, py, yaw);
        Eigen::Vector3d innov = z - H * x;
        innov(2) = wrap_angle(innov(2));

        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R(0, 0) = cfg.sigma_lio_xy * cfg.sigma_lio_xy;
        R(1, 1) = cfg.sigma_lio_xy * cfg.sigma_lio_xy;
        R(2, 2) = cfg.sigma_lio_yaw * cfg.sigma_lio_yaw;

        Eigen::Matrix3d S = H * P * H.transpose() + R;
        Eigen::FullPivLU<Eigen::Matrix3d> lu(S);
        if (!lu.isInvertible()) return;
        Eigen::Matrix<double, 4, 3> K = P * H.transpose() * lu.inverse();

        x = x + K * innov;
        x(2) = wrap_angle(x(2));
        Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
        P = (I4 - K * H) * P;
        P = 0.5 * (P + P.transpose());
        lio_anchored = true;
        has_state = true;
    }

    // Bootstrap pose to the first /Odometry without Kalman fusion. Avoids a
    // multi-meter jump in /odom when the very first absolute reference shows
    // up after some free-running drift.
    void snap_to(double px, double py, double yaw,
                 double sigma_pos = 0.02, double sigma_yaw = 0.01) {
        x(0) = px;
        x(1) = py;
        x(2) = wrap_angle(yaw);
        const double bias_var = P(3, 3);
        P.setZero();
        P(0, 0) = sigma_pos * sigma_pos;
        P(1, 1) = sigma_pos * sigma_pos;
        P(2, 2) = sigma_yaw * sigma_yaw;
        P(3, 3) = bias_var > 0.0 ? bias_var
                                 : (cfg.init_sigma_bias * cfg.init_sigma_bias);
        lio_anchored = true;
        has_state = true;
    }
};

constexpr const char *MODE_FREE_RUNNING = "EKF_FREE_RUNNING";
constexpr const char *MODE_LIO_TRACKING = "EKF_LIO_TRACKING";
constexpr const char *MODE_LIO_LOST     = "EKF_DEAD_RECKONING";

}  // namespace

// =============================================================================

class ChassisEKFNode : public rclcpp::Node {
public:
    ChassisEKFNode() : Node("chassis_ekf") {
        // -- topics & frames --
        this->declare_parameter<std::string>("chassis_speed_topic",
                                             "/motion_control/chassis_speed");
        this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("lio_topic", "/Odometry");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");

        // -- timeouts --
        this->declare_parameter<double>("lio_timeout_sec", 0.5);
        this->declare_parameter<double>("chassis_speed_timeout_sec", 1.0);

        // -- noise --
        this->declare_parameter<double>("sigma_vx", 0.05);
        this->declare_parameter<double>("sigma_vy", 0.05);
        this->declare_parameter<double>("sigma_omega", 0.01);
        this->declare_parameter<double>("sigma_bias", 1e-4);
        this->declare_parameter<double>("sigma_lio_xy", 0.05);
        this->declare_parameter<double>("sigma_lio_yaw", 0.02);
        this->declare_parameter<double>("init_sigma_pos", 1.0);
        this->declare_parameter<double>("init_sigma_yaw", 0.5);
        this->declare_parameter<double>("init_sigma_bias", 0.05);

        const auto p = [&](const std::string &n) { return this->get_parameter(n); };
        chassis_speed_topic_ = p("chassis_speed_topic").as_string();
        imu_topic_           = p("imu_topic").as_string();
        lio_topic_           = p("lio_topic").as_string();
        odom_topic_          = p("odom_topic").as_string();
        odom_frame_          = p("odom_frame").as_string();
        base_frame_          = p("base_frame").as_string();
        lio_timeout_         = p("lio_timeout_sec").as_double();
        chassis_speed_timeout_ = p("chassis_speed_timeout_sec").as_double();

        ekf_.cfg.sigma_vx        = p("sigma_vx").as_double();
        ekf_.cfg.sigma_vy        = p("sigma_vy").as_double();
        ekf_.cfg.sigma_omega     = p("sigma_omega").as_double();
        ekf_.cfg.sigma_bias      = p("sigma_bias").as_double();
        ekf_.cfg.sigma_lio_xy    = p("sigma_lio_xy").as_double();
        ekf_.cfg.sigma_lio_yaw   = p("sigma_lio_yaw").as_double();
        ekf_.cfg.init_sigma_pos  = p("init_sigma_pos").as_double();
        ekf_.cfg.init_sigma_yaw  = p("init_sigma_yaw").as_double();
        ekf_.cfg.init_sigma_bias = p("init_sigma_bias").as_double();
        ekf_.reset_prior();

        // -- QoS --
        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
        auto sensor_qos   = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort();

        sub_chassis_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            chassis_speed_topic_, reliable_qos,
            std::bind(&ChassisEKFNode::chassis_speed_cb, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, sensor_qos,
            std::bind(&ChassisEKFNode::imu_cb, this, std::placeholders::_1));
        sub_lio_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lio_topic_, reliable_qos,
            std::bind(&ChassisEKFNode::lio_cb, this, std::placeholders::_1));

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_, reliable_qos);

        RCLCPP_INFO(this->get_logger(),
                    "chassis_ekf up. inputs: chassis_speed=%s, imu=%s, lio=%s "
                    "-> odom=%s",
                    chassis_speed_topic_.c_str(), imu_topic_.c_str(),
                    lio_topic_.c_str(), odom_topic_.c_str());
    }

private:
    // ---- IMU: cache latest yaw rate ---------------------------------------
    void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        double ts = stamp_to_sec(msg->header.stamp);
        if (ts <= 0.0) ts = now_sec();
        std::lock_guard<std::mutex> lk(mtx_);
        last_omega_imu_ = msg->angular_velocity.z;
        last_imu_ts_    = ts;
    }

    // ---- chassis_speed: predict + publish /odom ---------------------------
    void chassis_speed_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        double ts = stamp_to_sec(msg->header.stamp);
        if (ts <= 0.0) ts = now_sec();

        const double vx = msg->twist.linear.x;
        const double vy = msg->twist.linear.y;
        const double omega_chassis = msg->twist.angular.z;

        std::lock_guard<std::mutex> lk(mtx_);
        last_vx_ = vx;
        last_vy_ = vy;
        last_omega_chassis_ = omega_chassis;
        last_chassis_ts_ = ts;

        // Prefer IMU yaw rate when fresh (lower noise than chassis feedback).
        const double now = now_sec();
        const bool use_imu = (last_imu_ts_ > 0.0) && ((now - last_imu_ts_) < 1.0);
        double omega_input;
        if (use_imu) {
            omega_input = last_omega_imu_;          // bias subtracted inside predict()
        } else {
            // We're feeding omega *without* bias correction; freeze the bias
            // state for this step by adding the current bias back so predict()
            // subtracts it again to a no-op on the bias.
            omega_input = omega_chassis + ekf_.x(3);
        }

        if (last_predict_ts_ <= 0.0) {
            // First message: just stamp the predict clock.
            last_predict_ts_ = ts;
            ekf_.has_state = true;
        } else {
            const double dt = ts - last_predict_ts_;
            if (dt > 0.0) {
                ekf_.predict(vx, vy, omega_input, dt);
                last_predict_ts_ = ts;
            }
        }

        publish_odom_locked(ts);
        update_mode_locked();
    }

    // ---- /Odometry: snap or Kalman update ---------------------------------
    void lio_cb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        double ts = stamp_to_sec(msg->header.stamp);
        if (ts <= 0.0) ts = now_sec();

        const auto &q = msg->pose.pose.orientation;
        const double yaw = quat_to_yaw(q.x, q.y, q.z, q.w);
        const double px = msg->pose.pose.position.x;
        const double py = msg->pose.pose.position.y;

        std::lock_guard<std::mutex> lk(mtx_);
        if (!ekf_.lio_anchored) {
            ekf_.snap_to(px, py, yaw);
            RCLCPP_INFO(this->get_logger(),
                "EKF anchored to first /Odometry: (%.3f, %.3f, yaw=%.1f deg)",
                px, py, yaw * 180.0 / M_PI);
        } else {
            // Predict up to the LIO timestamp using the latest known inputs
            // so the update is applied at the right instant in time.
            if (last_predict_ts_ > 0.0 && ts > last_predict_ts_) {
                const double dt = ts - last_predict_ts_;
                const double now = now_sec();
                const bool stale = (last_chassis_ts_ <= 0.0) ||
                                   ((now - last_chassis_ts_) > chassis_speed_timeout_);
                const double vx = stale ? 0.0 : last_vx_;
                const double vy = stale ? 0.0 : last_vy_;
                ekf_.predict(vx, vy, last_omega_imu_, dt);
                last_predict_ts_ = ts;
            }
            ekf_.update_lio(px, py, yaw);
        }

        last_lio_ts_ = ts;
        update_mode_locked();
    }

    // ---- helpers ----------------------------------------------------------
    double now_sec() {
        return this->get_clock()->now().seconds();
    }

    void update_mode_locked() {
        std::string new_mode = mode_;
        if (!ekf_.lio_anchored) {
            new_mode = MODE_FREE_RUNNING;
        } else {
            const double lio_age = now_sec() - last_lio_ts_;
            new_mode = (lio_age > lio_timeout_) ? MODE_LIO_LOST
                                                : MODE_LIO_TRACKING;
        }
        if (new_mode != mode_) {
            RCLCPP_WARN(this->get_logger(),
                        "/odom mode: %s -> %s",
                        mode_.c_str(), new_mode.c_str());
            mode_ = new_mode;
        }
    }

    void publish_odom_locked(double ts) {
        nav_msgs::msg::Odometry out;
        const int sec = static_cast<int>(ts);
        out.header.stamp.sec = sec;
        out.header.stamp.nanosec =
            static_cast<uint32_t>(std::round((ts - sec) * 1e9));
        out.header.frame_id = odom_frame_;
        out.child_frame_id  = base_frame_;

        out.pose.pose.position.x = ekf_.x(0);
        out.pose.pose.position.y = ekf_.x(1);
        out.pose.pose.position.z = 0.0;
        const double yaw = ekf_.x(2);
        out.pose.pose.orientation.x = 0.0;
        out.pose.pose.orientation.y = 0.0;
        out.pose.pose.orientation.z = std::sin(yaw * 0.5);
        out.pose.pose.orientation.w = std::cos(yaw * 0.5);

        for (auto &c : out.pose.covariance) c = 0.0;
        out.pose.covariance[0]  = ekf_.P(0, 0);
        out.pose.covariance[1]  = ekf_.P(0, 1);
        out.pose.covariance[6]  = ekf_.P(1, 0);
        out.pose.covariance[7]  = ekf_.P(1, 1);
        out.pose.covariance[14] = 1e-3;
        out.pose.covariance[21] = 1e-3;
        out.pose.covariance[28] = 1e-3;
        out.pose.covariance[35] = ekf_.P(2, 2);

        out.twist.twist.linear.x = last_vx_;
        out.twist.twist.linear.y = last_vy_;
        out.twist.twist.linear.z = 0.0;
        const double now = now_sec();
        if (last_imu_ts_ > 0.0 && (now - last_imu_ts_) < 1.0) {
            out.twist.twist.angular.z = last_omega_imu_ - ekf_.x(3);
        } else {
            out.twist.twist.angular.z = last_omega_chassis_;
        }

        for (auto &c : out.twist.covariance) c = 0.0;
        out.twist.covariance[0]  = ekf_.cfg.sigma_vx * ekf_.cfg.sigma_vx;
        out.twist.covariance[7]  = ekf_.cfg.sigma_vy * ekf_.cfg.sigma_vy;
        out.twist.covariance[14] = 1e-3;
        out.twist.covariance[21] = 1e-3;
        out.twist.covariance[28] = 1e-3;
        out.twist.covariance[35] = ekf_.cfg.sigma_omega * ekf_.cfg.sigma_omega;

        pub_odom_->publish(out);
    }

private:
    // params / topic config
    std::string chassis_speed_topic_, imu_topic_, lio_topic_;
    std::string odom_topic_;
    std::string odom_frame_, base_frame_;
    double      lio_timeout_ = 0.5;
    double      chassis_speed_timeout_ = 1.0;

    // ROS handles
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_chassis_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          sub_lio_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             pub_odom_;

    // shared state — guarded by mtx_ since the default executor still
    // serializes callbacks per-node, but a multi-threaded executor would
    // not. Cheap mutex => harmless.
    std::mutex   mtx_;
    ChassisEKF   ekf_;
    double last_omega_imu_      = 0.0;
    double last_imu_ts_         = -1.0;
    double last_predict_ts_     = -1.0;
    double last_chassis_ts_     = -1.0;
    double last_lio_ts_         = -1.0;
    double last_vx_             = 0.0;
    double last_vy_             = 0.0;
    double last_omega_chassis_  = 0.0;
    std::string mode_           = MODE_FREE_RUNNING;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisEKFNode>());
    rclcpp::shutdown();
    return 0;
}
