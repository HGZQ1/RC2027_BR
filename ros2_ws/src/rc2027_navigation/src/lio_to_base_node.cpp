// lio_to_base —— 把 Point-LIO 的位姿换算成车体位姿，发布 lidar_odom -> base_footprint。
//
// 为什么需要它（docs/decisions/0003-field-localization.md 决定 8）：
// Point-LIO 自己发的 TF 是 lidar_odom -> lidar_link（laserMapping.cpp 写死），
// 但 URDF 里 lidar_link 的父节点是 base_link，一个坐标系两个父节点 TF 树会乱，
// map -> base_link 也接不起来。所以 Point-LIO 的 TF 一律关掉
// （publish.tf_send_en: false），改由这个节点：
//
//   订阅 /aft_mapped_to_init（lidar_odom -> body，body = Point-LIO 的 IMU 系）
//   从 TF 查一次 base_footprint -> <body_frame>（URDF 里的静态外参）
//   发布 lidar_odom -> base_footprint = (lidar_odom -> body) * (base_footprint -> body)^-1
//
// 发 base_footprint 而不是 base_link：URDF 根节点是 base_footprint，base_link 已经
// 有父节点了（跟 GAFA 去年 TF_TREE.md 里 EKF 发 odom -> base_footprint 同一个理由）。
//
// body_frame 参数必须是 URDF 里跟 Point-LIO "body" 重合的那个坐标系：
//   - 仿真：livox_sim（雷达和 IMU 在同一个 link，Point-LIO 仿真配置外参为 0）
//   - 真车：lidar_imu_link（Mid-360 内置 IMU 在雷达系里的位置，见 URDF）
//
// 时间戳直接用 Point-LIO 里程计的时间戳，不用 now()——真车上雷达时间和系统时间
// 不一定是一个时钟，用里程计自己的时间戳才能跟点云对得上。
//
// 看门狗（2026-10-04 加）：仿真里 Point-LIO 发散后停止输出，下游只看到
// "extrapolation into the past"，很难看出是上游断了。现在收到过里程计以后，
// 超过 odom_timeout 秒（墙钟，默认 1 s）没有新的就打 ERROR，恢复了打 INFO。用墙钟不用
// ROS 时钟：仿真暂停 / RTF 很低时 ROS 时钟走得慢，会把真正的断流掩盖掉。

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rc2027_navigation/pose_math.hpp"

namespace rc2027_navigation
{

class LioToBaseNode : public rclcpp::Node
{
public:
  LioToBaseNode()
  : Node("lio_to_base")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "lidar_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    body_frame_ = declare_parameter<std::string>("body_frame", "lidar_imu_link");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    // 1 s：仿真 RTF 低到 0.2 时正常帧间隔（墙钟）也有 0.5 s，再短会误报
    odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LioToBaseNode::on_odom, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&LioToBaseNode::check_watchdog, this));

    RCLCPP_INFO(
      get_logger(), "%s -> %s，外参取 TF 里的 %s -> %s", odom_frame_.c_str(), base_frame_.c_str(),
      base_frame_.c_str(), body_frame_.c_str());
  }

private:
  void check_watchdog()
  {
    if (!got_odom_) {
      return;  // 还没收到过（Point-LIO 在初始化），不算断流
    }
    const double silent = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_odom_wall_).count();
    if (silent > odom_timeout_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "已经 %.1f s 没收到 Point-LIO 里程计了，%s -> %s 停止更新（Point-LIO 发散/卡住？"
        "看它的终端输出）", silent, odom_frame_.c_str(), base_frame_.c_str());
      stalled_ = true;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_wall_ = std::chrono::steady_clock::now();
    got_odom_ = true;
    if (stalled_) {
      RCLCPP_INFO(get_logger(), "Point-LIO 里程计恢复了");
      stalled_ = false;
    }

    if (!have_extrinsic_ && !lookup_extrinsic()) {
      return;
    }

    Transform3D odom_to_body;
    odom_to_body.x = msg->pose.pose.position.x;
    odom_to_body.y = msg->pose.pose.position.y;
    odom_to_body.z = msg->pose.pose.position.z;
    odom_to_body.qx = msg->pose.pose.orientation.x;
    odom_to_body.qy = msg->pose.pose.orientation.y;
    odom_to_body.qz = msg->pose.pose.orientation.z;
    odom_to_body.qw = msg->pose.pose.orientation.w;

    const auto t = odom_to_base(odom_to_body, base_to_body_);

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.transform.translation.x = t.x;
    out.transform.translation.y = t.y;
    out.transform.translation.z = t.z;
    out.transform.rotation.x = t.qx;
    out.transform.rotation.y = t.qy;
    out.transform.rotation.z = t.qz;
    out.transform.rotation.w = t.qw;
    tf_broadcaster_->sendTransform(out);
  }

  // 外参是 URDF 里的固定关节（robot_state_publisher 发在 /tf_static），查到一次就够
  bool lookup_extrinsic()
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, body_frame_, tf2::TimePointZero);
      base_to_body_.x = tf.transform.translation.x;
      base_to_body_.y = tf.transform.translation.y;
      base_to_body_.z = tf.transform.translation.z;
      base_to_body_.qx = tf.transform.rotation.x;
      base_to_body_.qy = tf.transform.rotation.y;
      base_to_body_.qz = tf.transform.rotation.z;
      base_to_body_.qw = tf.transform.rotation.w;
      have_extrinsic_ = true;
      RCLCPP_INFO(
        get_logger(), "外参 %s -> %s: xyz=(%.4f, %.4f, %.4f) yaw=%.4f", base_frame_.c_str(),
        body_frame_.c_str(), base_to_body_.x, base_to_body_.y, base_to_body_.z,
        yaw_of(base_to_body_));
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "查不到外参 %s -> %s（robot_state_publisher 没起？body_frame 写错？）: %s",
        base_frame_.c_str(), body_frame_.c_str(), exc.what());
      return false;
    }
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string body_frame_;

  double odom_timeout_{1.0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // 日志限频用墙钟，仿真暂停时也照常打
  bool got_odom_{false};
  bool stalled_{false};
  std::chrono::steady_clock::time_point last_odom_wall_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool have_extrinsic_{false};
  Transform3D base_to_body_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace rc2027_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc2027_navigation::LioToBaseNode>());
  rclcpp::shutdown();
  return 0;
}
