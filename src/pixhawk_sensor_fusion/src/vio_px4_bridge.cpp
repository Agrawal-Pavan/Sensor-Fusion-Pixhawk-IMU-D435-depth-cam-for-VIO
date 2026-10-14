#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VioPx4Bridge : public rclcpp::Node
{
public:
  VioPx4Bridge() : Node("vio_px4_bridge")
  {
    // Declare parameters
    this->declare_parameter<std::string>("odom_topic", "/rtabmap/odom");
    this->declare_parameter<std::string>("px4_topic", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<double>("max_allowed_variance", 90.0);
    this->declare_parameter<double>("default_position_variance", 0.01);    // ~10 cm std dev
    this->declare_parameter<double>("default_orientation_variance", 0.01); // ~0.1 rad std dev

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    px4_topic_ = this->get_parameter("px4_topic").as_string();
    max_allowed_variance_ = static_cast<float>(this->get_parameter("max_allowed_variance").as_double());
    default_pos_var_ = static_cast<float>(this->get_parameter("default_position_variance").as_double());
    default_ori_var_ = static_cast<float>(this->get_parameter("default_orientation_variance").as_double());

    // PX4 uXRCE-DDS publisher: Best Effort QoS is standard for sensor streams
    rclcpp::QoS px4_qos(10);
    px4_qos.best_effort();

    pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, px4_qos);
    mavros_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

    // RTAB-Map odom subscriber
    rclcpp::QoS odom_qos(10);
    odom_qos.best_effort();

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(&VioPx4Bridge::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "VIO-PX4 Bridge initialized. Subscribing to: %s, Publishing to: %s",
      odom_topic_.c_str(), px4_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & pos = msg->pose.pose.position;
    const auto & ori = msg->pose.pose.orientation;
    const auto & cov = msg->pose.covariance;

    // Check for NaN or Inf in incoming odometry
    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z) ||
        std::isnan(ori.w) || std::isnan(ori.x) || std::isnan(ori.y) || std::isnan(ori.z))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Received NaN in visual odometry. Skipping message.");
      return;
    }

    // Skip publishing when RTAB-Map reports lost state (variance >= max_allowed_variance)
    if (cov[0] >= max_allowed_variance_ || cov[7] >= max_allowed_variance_ || cov[14] >= max_allowed_variance_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Visual odometry lost (cov[0]=%.2f >= %.2f). Skipping frame to protect PX4 EKF.",
        cov[0], max_allowed_variance_);
      return;
    }

    px4_msgs::msg::VehicleOdometry px4_msg;

    // Timestamps in microseconds
    px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.timestamp_sample = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000;

    // Set Pose Frame to FRD (Forward-Right-Down arbitrary local heading)
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

    // Position Conversion (ROS FLU/ENU to PX4 FRD):
    // X_frd =  X_flu
    // Y_frd = -Y_flu
    // Z_frd = -Z_flu
    px4_msg.position[0] = static_cast<float>(pos.x);
    px4_msg.position[1] = static_cast<float>(-pos.y);
    px4_msg.position[2] = static_cast<float>(-pos.z);

    // Orientation Conversion (B * R * B rotation for FLU to FRD):
    // In Hamiltonian quaternion [w, x, y, z]:
    // w_frd =  w
    // x_frd =  x
    // y_frd = -y
    // z_frd = -z
    px4_msg.q[0] = static_cast<float>(ori.w);
    px4_msg.q[1] = static_cast<float>(ori.x);
    px4_msg.q[2] = static_cast<float>(-ori.y);
    px4_msg.q[3] = static_cast<float>(-ori.z);

    // Variances
    float var_x = (cov[0] > 0.0 && cov[0] < max_allowed_variance_) ? static_cast<float>(cov[0]) : default_pos_var_;
    float var_y = (cov[7] > 0.0 && cov[7] < max_allowed_variance_) ? static_cast<float>(cov[7]) : default_pos_var_;
    float var_z = (cov[14] > 0.0 && cov[14] < max_allowed_variance_) ? static_cast<float>(cov[14]) : default_pos_var_;
    px4_msg.position_variance = {var_x, var_y, var_z};

    float var_roll = (cov[21] > 0.0 && cov[21] < max_allowed_variance_) ? static_cast<float>(cov[21]) : default_ori_var_;
    float var_pitch = (cov[28] > 0.0 && cov[28] < max_allowed_variance_) ? static_cast<float>(cov[28]) : default_ori_var_;
    float var_yaw = (cov[35] > 0.0 && cov[35] < max_allowed_variance_) ? static_cast<float>(cov[35]) : default_ori_var_;
    px4_msg.orientation_variance = {var_roll, var_pitch, var_yaw};

    // Velocity: Set to NaN since VO twist may have high noise; PX4 EKF2 estimates velocity from position
    constexpr float f_nan = std::numeric_limits<float>::quiet_NaN();
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    px4_msg.velocity = {f_nan, f_nan, f_nan};
    px4_msg.angular_velocity = {f_nan, f_nan, f_nan};
    px4_msg.velocity_variance = {f_nan, f_nan, f_nan};

    px4_msg.reset_counter = 0;
    px4_msg.quality = 0;

    pub_->publish(px4_msg);

    // Also publish to MAVROS for Pixhawks running MAVLink
    geometry_msgs::msg::PoseStamped mavros_msg;
    mavros_msg.header = msg->header;
    mavros_msg.pose = msg->pose.pose;
    mavros_pub_->publish(mavros_msg);
  }

  std::string odom_topic_;
  std::string px4_topic_;
  float max_allowed_variance_;
  float default_pos_var_;
  float default_ori_var_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VioPx4Bridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
