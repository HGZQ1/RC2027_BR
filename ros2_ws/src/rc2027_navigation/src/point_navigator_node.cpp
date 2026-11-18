// point_navigator —— 点到点导航，见 docs/decisions/0005-navigation.md。
//
// 控制律在 point_nav_controller.hpp（纯逻辑，有单测），这里是 ROS 薄壳：
//   - 每个控制周期查 TF map -> robot_frame（默认 base_link，跟 teach_point 录点用的
//     同一个坐标系，录的点直接能当目标），喂给控制器，把输出发到 /cmd_vel
//   - 目标入口：
//       navigate_to_pose（nav2_msgs/action/NavigateToPose）——跟 GAFA 去年一样保留 Nav2 的
//         标准接口，用节点参数的到位门限
//       navigate_to_point（rc2027_navigation_msgs/action/NavigateToPoint）——决策层 Flow 引擎
//         用这个，多了每个目标自己的到位门限（拿 / 放前最后一步用 1 cm 级，2026-10-04 加）。
//         两个入口共用一个控制器，新目标顶掉旧目标，不管从哪个入口来
//       /goal_pose（geometry_msgs/PoseStamped）——RViz 工具栏 "2D Goal Pose" 默认发
//         这个话题，手动测试用；会顶掉正在执行的 action 目标
//   - 订阅 /field_localizer/retry_status：PAUSED / RETRY_FAILED 时立刻停车、中止当前
//     目标。0003 决定 6 原本只要求决策层"自觉"停车，cmd_vel 是从这里发出去的，
//     在这里拦一道成本最低，也不依赖决策层写对
//   - /point_navigator/status（String）：IDLE/TRANSLATE/ROTATE/ARRIVED/TIMEOUT/STUCK/PAUSED
//
// 目标的 frame_id 必须是 map（或者空，当成 map）。不做坐标变换：所有点位都应该是场地
// 坐标（field_geometry.yaml / teach_point 录的点），出现别的坐标系说明调用方搞错了，
// 直接拒绝比悄悄变换更容易发现问题。
//
// 单线程 executor 就够：action 的执行不开线程，在控制定时器里推进、在定时器里
// succeed/abort/canceled，所有状态都在同一个线程里改。

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rc2027_navigation/point_nav_controller.hpp"
#include "rc2027_navigation_msgs/action/navigate_to_point.hpp"

namespace rc2027_navigation
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoint = rc2027_navigation_msgs::action::NavigateToPoint;
using PoseGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using PointGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPoint>;

namespace
{

double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/// 正在执行的 action 目标，把两种 action 类型的 goal handle 包成同一个样子，
/// 后面的控制循环不用管目标是从哪个入口来的
struct ActiveGoal
{
  std::function<bool()> is_active;
  std::function<bool()> is_canceling;
  /// outcome：ARRIVED 算成功，其它（TIMEOUT/STUCK/PAUSED/PREEMPTED）算失败
  std::function<void(const std::string & outcome, double distance)> finish;
  std::function<void(double distance)> canceled;
  std::function<void(const geometry_msgs::msg::PoseStamped & pose, double distance)> feedback;
};

ActiveGoal wrap(const std::shared_ptr<PoseGoalHandle> & h)
{
  ActiveGoal g;
  g.is_active = [h]() {return h->is_active();};
  g.is_canceling = [h]() {return h->is_canceling();};
  g.finish = [h](const std::string & outcome, double) {
      auto result = std::make_shared<NavigateToPose::Result>();
      if (outcome == "ARRIVED") {
        h->succeed(result);
      } else {
        h->abort(result);
      }
    };
  g.canceled = [h](double) {h->canceled(std::make_shared<NavigateToPose::Result>());};
  g.feedback = [h](const geometry_msgs::msg::PoseStamped & pose, double distance) {
      auto fb = std::make_shared<NavigateToPose::Feedback>();
      fb->current_pose = pose;
      fb->distance_remaining = static_cast<float>(distance);
      h->publish_feedback(fb);
    };
  return g;
}

ActiveGoal wrap(const std::shared_ptr<PointGoalHandle> & h)
{
  ActiveGoal g;
  g.is_active = [h]() {return h->is_active();};
  g.is_canceling = [h]() {return h->is_canceling();};
  auto make_result = [](const std::string & outcome, double distance) {
      auto result = std::make_shared<NavigateToPoint::Result>();
      result->outcome = outcome;
      result->distance_remaining = static_cast<float>(distance);
      return result;
    };
  g.finish = [h, make_result](const std::string & outcome, double distance) {
      if (outcome == "ARRIVED") {
        h->succeed(make_result(outcome, distance));
      } else {
        h->abort(make_result(outcome, distance));
      }
    };
  g.canceled = [h, make_result](double distance) {
      h->canceled(make_result("CANCELED", distance));
    };
  g.feedback = [h](const geometry_msgs::msg::PoseStamped & pose, double distance) {
      auto fb = std::make_shared<NavigateToPoint::Feedback>();
      fb->current_pose = pose;
      fb->distance_remaining = static_cast<float>(distance);
      h->publish_feedback(fb);
    };
  return g;
}

}  // namespace

class PointNavigatorNode : public rclcpp::Node
{
public:
  PointNavigatorNode()
  : Node("point_navigator")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    const double rate = declare_parameter<double>("control_rate", 20.0);
    dt_ = 1.0 / rate;

    NavConfig cfg;
    cfg.max_linear_speed = declare_parameter<double>("max_linear_speed", cfg.max_linear_speed);
    cfg.min_linear_speed = declare_parameter<double>("min_linear_speed", cfg.min_linear_speed);
    cfg.max_angular_speed = declare_parameter<double>("max_angular_speed", cfg.max_angular_speed);
    cfg.min_angular_speed = declare_parameter<double>("min_angular_speed", cfg.min_angular_speed);
    cfg.kp_linear = declare_parameter<double>("kp_linear", cfg.kp_linear);
    cfg.kp_angular = declare_parameter<double>("kp_angular", cfg.kp_angular);
    cfg.kp_hold = declare_parameter<double>("kp_hold", cfg.kp_hold);
    cfg.max_hold_speed = declare_parameter<double>("max_hold_speed", cfg.max_hold_speed);
    cfg.linear_decel = declare_parameter<double>("linear_decel", cfg.linear_decel);
    cfg.angular_decel = declare_parameter<double>("angular_decel", cfg.angular_decel);
    cfg.max_linear_accel = declare_parameter<double>("max_linear_accel", cfg.max_linear_accel);
    cfg.max_angular_accel = declare_parameter<double>("max_angular_accel", cfg.max_angular_accel);
    cfg.xy_tolerance = declare_parameter<double>("xy_tolerance", cfg.xy_tolerance);
    cfg.yaw_tolerance = declare_parameter<double>("yaw_tolerance", cfg.yaw_tolerance);
    cfg.goal_timeout = declare_parameter<double>("goal_timeout", cfg.goal_timeout);
    cfg.stuck_timeout = declare_parameter<double>("stuck_timeout", cfg.stuck_timeout);
    cfg.progress_min_delta = declare_parameter<double>("progress_min_delta", cfg.progress_min_delta);
    cfg.max_pushes = declare_parameter<int>("max_pushes", cfg.max_pushes);
    cfg.rotate_timeout = declare_parameter<double>("rotate_timeout", cfg.rotate_timeout);
    controller_ = PointNavController(cfg);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("point_navigator/status", 10);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&PointNavigatorNode::on_goal_topic, this, std::placeholders::_1));
    retry_sub_ = create_subscription<std_msgs::msg::String>(
      "field_localizer/retry_status", rclcpp::QoS(1).transient_local(),
      std::bind(&PointNavigatorNode::on_retry_status, this, std::placeholders::_1));

    pose_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal) {
        return check_goal(goal->pose.header.frame_id);
      },
      [](const std::shared_ptr<PoseGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<PoseGoalHandle> h) {
        accept_goal(wrap(h), to_pose2d(h->get_goal()->pose.pose), 0.0, 0.0, "navigate_to_pose");
      });
    point_server_ = rclcpp_action::create_server<NavigateToPoint>(
      this, "navigate_to_point",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPoint::Goal> goal) {
        return check_goal(goal->pose.header.frame_id);
      },
      [](const std::shared_ptr<PointGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<PointGoalHandle> h) {
        const auto goal = h->get_goal();
        accept_goal(
          wrap(h), to_pose2d(goal->pose.pose), goal->xy_tolerance, goal->yaw_tolerance,
          "navigate_to_point");
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&PointNavigatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "point_navigator: %s -> %s, %.0f Hz, 容差 %.3f m / %.3f rad",
      map_frame_.c_str(), robot_frame_.c_str(), rate, cfg.xy_tolerance, cfg.yaw_tolerance);
  }

private:
  double now_sec() {return get_clock()->now().seconds();}

  bool frame_ok(const std::string & frame_id)
  {
    if (frame_id.empty() || frame_id == map_frame_) {
      return true;
    }
    RCLCPP_ERROR(
      get_logger(), "目标的 frame_id 是 '%s'，只接受 '%s'（所有点位都应该是场地坐标）",
      frame_id.c_str(), map_frame_.c_str());
    return false;
  }

  static Pose2D to_pose2d(const geometry_msgs::msg::Pose & p)
  {
    return {p.position.x, p.position.y,
      yaw_from_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)};
  }

  // ── 目标入口 ──
  void start_goal(
    const Pose2D & goal, double xy_tolerance, double yaw_tolerance, const std::string & source)
  {
    if (paused_) {
      RCLCPP_WARN(get_logger(), "Retry 进行中（%s），不接受新目标", retry_status_.c_str());
      return;
    }
    controller_.set_goal(goal, now_sec(), xy_tolerance, yaw_tolerance);
    RCLCPP_INFO(
      get_logger(), "新目标（%s）: (%.3f, %.3f, yaw %.1f°)，门限 %.3f m / %.1f°", source.c_str(),
      goal.x, goal.y, goal.yaw * 180.0 / M_PI, controller_.xy_tolerance(),
      controller_.yaw_tolerance() * 180.0 / M_PI);
  }

  void on_goal_topic(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!frame_ok(msg->header.frame_id)) {
      return;
    }
    abort_active_goal("PREEMPTED", "被 /goal_pose 的新目标顶掉");
    start_goal(to_pose2d(msg->pose), 0.0, 0.0, "/goal_pose");
  }

  // 两个 action 入口共用。取消请求一律答应，真正的停车和 canceled() 在定时器里做
  // （看 is_canceling()）
  rclcpp_action::GoalResponse check_goal(const std::string & frame_id)
  {
    if (!frame_ok(frame_id)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (paused_) {
      RCLCPP_WARN(get_logger(), "Retry 进行中（%s），拒绝 action 目标", retry_status_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void accept_goal(
    ActiveGoal goal, const Pose2D & pose, double xy_tolerance, double yaw_tolerance,
    const std::string & source)
  {
    abort_active_goal("PREEMPTED", "被新的 action 目标顶掉");
    active_goal_ = std::move(goal);
    start_goal(pose, xy_tolerance, yaw_tolerance, source);
  }

  void abort_active_goal(const std::string & outcome, const std::string & why)
  {
    if (active_goal_ && active_goal_->is_active()) {
      RCLCPP_WARN(get_logger(), "中止当前 action 目标：%s", why.c_str());
      active_goal_->finish(outcome, controller_.distance_remaining());
    }
    active_goal_.reset();
  }

  // ── Retry 联动（0003 决定 6）──
  void on_retry_status(const std_msgs::msg::String::SharedPtr msg)
  {
    retry_status_ = msg->data;
    // 格式 "<状态> [zone]"，例如 "PAUSED l1"、"RESUMED start"（2026-10-04 起带 zone，
    // 决策层要按 zone 选 Retry 后的流程），这里只看第一个词
    const std::string state = msg->data.substr(0, msg->data.find(' '));
    const bool pause = (state == "PAUSED" || state == "RETRY_FAILED");
    if (pause && !paused_) {
      RCLCPP_WARN(get_logger(), "field_localizer: %s，停车并中止当前目标", msg->data.c_str());
      controller_.cancel();
      abort_active_goal("PAUSED", "Retry " + msg->data);
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    } else if (!pause && paused_) {
      RCLCPP_INFO(get_logger(), "field_localizer: %s，可以接受新目标", msg->data.c_str());
    }
    paused_ = pause;
  }

  // ── 控制循环 ──
  void on_timer()
  {
    if (active_goal_ && active_goal_->is_canceling()) {
      controller_.cancel();
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      active_goal_->canceled(controller_.distance_remaining());
      active_goal_.reset();
      RCLCPP_INFO(get_logger(), "action 目标已取消");
    }

    const bool was_active = is_active(controller_.phase());
    Twist2D cmd;
    const double now = now_sec();

    if (paused_) {
      // 已经在 on_retry_status 里停过了，这里只是保险
    } else if (was_active) {
      try {
        const auto tf = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
        const auto & r = tf.transform.rotation;
        const Pose2D current{
          tf.transform.translation.x, tf.transform.translation.y,
          yaw_from_quaternion(r.x, r.y, r.z, r.w)};
        cmd = controller_.update(current, now, dt_);
        publish_feedback(current);
      } catch (const tf2::TransformException & exc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "查不到 %s -> %s，停车等定位: %s",
          map_frame_.c_str(), robot_frame_.c_str(), exc.what());
        cmd = controller_.hold(now);
      }
    }

    // 导航中、或者刚刚结束（发一次 0 把车停住）才发 cmd_vel；空闲时不发，
    // 不跟 teleop 抢话题
    const bool is_now_active = is_active(controller_.phase());
    if (was_active || is_now_active) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = cmd.vx;
      twist.linear.y = cmd.vy;
      twist.angular.z = cmd.wz;
      cmd_pub_->publish(twist);
    }

    if (was_active && !is_now_active) {
      finish_active_goal();
    }

    std_msgs::msg::String status;
    status.data = paused_ ? "PAUSED" : to_string(controller_.phase());
    status_pub_->publish(status);
  }

  void publish_feedback(const Pose2D & current)
  {
    if (!active_goal_ || !active_goal_->is_active()) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = get_clock()->now();
    pose.pose.position.x = current.x;
    pose.pose.position.y = current.y;
    pose.pose.orientation.z = std::sin(current.yaw / 2.0);
    pose.pose.orientation.w = std::cos(current.yaw / 2.0);
    active_goal_->feedback(pose, controller_.distance_remaining());
  }

  void finish_active_goal()
  {
    const auto phase = controller_.phase();
    RCLCPP_INFO(
      get_logger(), "导航结束：%s（剩余距离 %.3f m）", to_string(phase).c_str(),
      controller_.distance_remaining());
    if (!active_goal_ || !active_goal_->is_active()) {
      active_goal_.reset();
      return;
    }
    active_goal_->finish(to_string(phase), controller_.distance_remaining());
    active_goal_.reset();
  }

  std::string map_frame_;
  std::string robot_frame_;
  double dt_{0.05};

  PointNavController controller_;
  bool paused_{false};
  std::string retry_status_{"IDLE"};
  std::optional<ActiveGoal> active_goal_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr retry_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr pose_server_;
  rclcpp_action::Server<NavigateToPoint>::SharedPtr point_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rc2027_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc2027_navigation::PointNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
