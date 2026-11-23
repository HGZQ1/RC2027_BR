// Gazebo Classic 全向底盘。ROS 只更新目标；每个物理步按仿真 dt 改变速度。
#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rc2027_sim_bridge
{
class PlanarDrive : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    base_ = model_->GetLink("base_footprint");
    if (!base_) {
      RCLCPP_ERROR(node_->get_logger(), "全向底盘找不到 base_footprint");
      return;
    }
    accel_ = sdf->Get<double>("linear_accel", 8.0).first;
    angular_accel_ = sdf->Get<double>("angular_accel", 6.0).first;
    command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_ = *msg;
      });
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    auto & covariance = odom_.pose.covariance;
    covariance[0] = covariance[7] = 1e-5;
    covariance[14] = covariance[21] = covariance[28] = 1e12;
    covariance[35] = 1e-3;
    odom_.twist.covariance = covariance;
    Reset();
    connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PlanarDrive::update, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "全向底盘按物理步限加速度 %.1f m/s²，%.1f rad/s²",
      accel_, angular_accel_);
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = geometry_msgs::msg::Twist{};
    last_step_ = last_odom_ = model_->GetWorld()->SimTime();
  }

private:
  void update(const gazebo::common::UpdateInfo & info)
  {
    const double dt = (info.simTime - last_step_).Double();
    last_step_ = info.simTime;
    if (dt <= 0.0) {return;}

    geometry_msgs::msg::Twist target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = target_;
    }
    const auto pose = base_->WorldPose();
    const auto velocity = base_->WorldLinearVel();
    const auto angular = base_->WorldAngularVel();
    const double c = std::cos(pose.Rot().Yaw()), s = std::sin(pose.Rot().Yaw());
    const double dx = c * target.linear.x - s * target.linear.y - velocity.X();
    const double dy = s * target.linear.x + c * target.linear.y - velocity.Y();
    const double delta = std::hypot(dx, dy);
    const double fraction = delta > 0.0 ? std::min(1.0, accel_ * dt / delta) : 0.0;
    const ignition::math::Vector3d base_velocity{
      velocity.X() + dx * fraction, velocity.Y() + dy * fraction, velocity.Z()};
    const double dw = std::clamp(
      target.angular.z - angular.Z(), -angular_accel_ * dt, angular_accel_ * dt);
    const ignition::math::Vector3d angular_velocity{0.0, 0.0, angular.Z() + dw};

    // WorldLinearVel 读取 link 原点，ODE SetLinearVel 写的是质心。
    // 用同一刚体运动计算各 link 的质心速度，避免每步把旋转分量算进底盘平移。
    for (const auto & link : model_->GetLinks()) {
      const auto offset = link->WorldCoGPose().Pos() - pose.Pos();
      link->SetEnabled(true);
      link->SetAngularVel(angular_velocity);
      link->SetLinearVel(base_velocity + angular_velocity.Cross(offset));
    }

    if ((info.simTime - last_odom_).Double() >= 0.02) {
      last_odom_ = info.simTime;
      odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);
      odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
      odom_.twist.twist.linear.x = c * velocity.X() + s * velocity.Y();
      odom_.twist.twist.linear.y = -s * velocity.X() + c * velocity.Y();
      odom_.twist.twist.angular.z = angular.Z();
      odom_pub_->publish(odom_);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo_ros::Node::SharedPtr node_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist target_;
  double accel_{}, angular_accel_{};
  gazebo::common::Time last_step_, last_odom_;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  gazebo::event::ConnectionPtr connection_;
};
GZ_REGISTER_MODEL_PLUGIN(PlanarDrive)
}  // namespace rc2027_sim_bridge
