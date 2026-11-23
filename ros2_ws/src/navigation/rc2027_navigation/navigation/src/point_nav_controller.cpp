#include "rc2027_navigation/point_nav_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rc2027_navigation
{

std::string to_string(NavPhase phase)
{
  switch (phase) {
    case NavPhase::Idle: return "IDLE";
    case NavPhase::Translate: return "TRANSLATE";
    case NavPhase::Rotate: return "ROTATE";
    case NavPhase::Arrived: return "ARRIVED";
    case NavPhase::Timeout: return "TIMEOUT";
    case NavPhase::Stuck: return "STUCK";
  }
  return "UNKNOWN";
}

bool is_active(NavPhase phase)
{
  return phase == NavPhase::Translate || phase == NavPhase::Rotate;
}

double normalize_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

namespace
{

double clamp_abs(double v, double max_abs)
{
  return std::max(-max_abs, std::min(max_abs, v));
}

double slew(double target, double current, double max_delta)
{
  return current + clamp_abs(target - current, max_delta);
}

// 按距离给速度大小：max、kp*d、sqrt(2*a*d) 取最小
double speed_profile(double d, double max_speed, double kp, double decel,
  double delay = 0.0, double exit_speed = 0.0)
{
  // 以 exit_speed 通过前方点，给超出入点速度的部分留出延迟和制动距离。
  const double offset = exit_speed + decel * delay;
  return std::min({max_speed, kp * d + exit_speed,
    std::sqrt(offset * offset + 2.0 * decel * d) - decel * delay});
}

}  // namespace

PointNavController::PointNavController(const NavConfig & config)
: cfg_(config), xy_tol_(config.xy_tolerance), yaw_tol_(config.yaw_tolerance)
{
}

void PointNavController::set_goal(
  const Pose2D & goal, double now, double xy_tolerance, double yaw_tolerance)
{
  realigning_same_goal_ = phase_ == NavPhase::Arrived &&
    std::hypot(goal.x - goal_.x, goal.y - goal_.y) < 1e-6 &&
    std::abs(normalize_angle(goal.yaw - goal_.yaw)) < 1e-6;
  via_.reset();
  through_approach_ = false;
  final_positioning_ = false;
  goal_ = goal;
  xy_tol_ = xy_tolerance > 0.0 ? xy_tolerance : cfg_.xy_tolerance;
  yaw_tol_ = yaw_tolerance > 0.0 ? yaw_tolerance : cfg_.yaw_tolerance;
  goal_.yaw = normalize_angle(goal.yaw);
  phase_ = NavPhase::Translate;
  goal_start_ = now;
  command_time_ = now;
  short_approach_ = false;
  settled_since_ = -1.0;
  last_progress_ = now;
  pushes_ = 0;
  best_dist_ = std::numeric_limits<double>::infinity();
  last_dist_ = std::numeric_limits<double>::infinity();
  // 不清 last_cmd_：中途换目标时速度从当前值平滑过渡，不是突然归零再加速
}

void PointNavController::cancel()
{
  phase_ = NavPhase::Idle;
  last_cmd_ = Twist2D{};
}

Twist2D PointNavController::finish(NavPhase phase)
{
  phase_ = phase;
  last_cmd_ = Twist2D{};
  return Twist2D{};
}

Twist2D PointNavController::limit(const Twist2D & target, double dt)
{
  Twist2D out;
  const double dx = target.vx - last_cmd_.vx;
  const double dy = target.vy - last_cmd_.vy;
  const double change = std::hypot(dx, dy);
  const double fraction = change > 0 ? std::min(1.0, cfg_.max_linear_accel * dt / change) : 0.0;
  out.vx = last_cmd_.vx + fraction * dx;
  out.vy = last_cmd_.vy + fraction * dy;
  out.wz = slew(target.wz, last_cmd_.wz, cfg_.max_angular_accel * dt);
  last_cmd_ = out;
  return out;
}

void PointNavController::account_approach_motion(
  double now, std::optional<double> chassis_path)
{
  if (short_approach_) {
    if (chassis_path && last_chassis_path_) {
      travel_remaining_ -= std::max(0.0, *chassis_path - *last_chassis_path_);
    } else if (!chassis_path) {
      travel_remaining_ -= std::hypot(last_cmd_.vx, last_cmd_.vy) *
        std::max(0.0, now - command_time_);
    }
  }
  last_chassis_path_ = chassis_path;
  command_time_ = now;
}

Twist2D PointNavController::hold(double now, std::optional<double> chassis_path)
{
  account_approach_motion(now, chassis_path);
  settled_since_ = -1.0;
  if (approach_overrun()) {
    return finish(NavPhase::Stuck);
  }
  if (is_active(phase_) && now - goal_start_ > cfg_.goal_timeout) {
    return finish(NavPhase::Timeout);
  }
  last_cmd_ = Twist2D{};
  return Twist2D{};
}

Twist2D PointNavController::update(const Pose2D & current, double now, double dt,
  const Twist2D & measured, std::optional<double> chassis_path)
{
  if (!is_active(phase_)) {
    last_cmd_ = Twist2D{};
    return Twist2D{};
  }

  if (via_) {
    const double dx = via_->x - current.x;
    const double dy = via_->y - current.y;
    const double remaining = std::hypot(dx, dy);
    const double yaw_error = normalize_angle(goal_.yaw - current.yaw);
    const double entry = cfg_.fine_approach_speed > 0 ? cfg_.fine_approach_speed : 0.25;
    const double c = std::cos(current.yaw), s = std::sin(current.yaw);
    const double approach_x = goal_.x - via_->x;
    const double approach_y = goal_.y - via_->y;
    const double approach_length = std::hypot(approach_x, approach_y);
    const auto aligned_velocity = [&](const Twist2D & twist) {
        if (std::hypot(twist.vx, twist.vy) > entry + 0.05) {return false;}
        if (approach_length < 1e-6) {return true;}
        const double vx = c * twist.vx - s * twist.vy;
        const double vy = s * twist.vx + c * twist.vy;
        const double along = (approach_x * vx + approach_y * vy) / approach_length;
        const double sideways = (approach_x * vy - approach_y * vx) / approach_length;
        return along >= -0.02 && std::abs(sideways) <= 0.05;
      };
    const bool yaw_ready = std::abs(yaw_error) <= 0.06 &&
      std::max(std::abs(last_cmd_.wz), std::abs(measured.wz)) <= 0.2;
    const bool near_via = remaining <= 0.035;
    if (near_via && yaw_ready && aligned_velocity(last_cmd_) && aligned_velocity(measured)) {
      via_.reset();
      through_approach_ = true;
      best_dist_ = last_dist_ = std::numeric_limits<double>::infinity();
      last_progress_ = now;
    } else {
      if (now - goal_start_ > cfg_.goal_timeout) {return finish(NavPhase::Timeout);}
      last_dist_ = remaining + std::hypot(goal_.x - via_->x, goal_.y - via_->y);
      const double error = remaining + 0.15 * std::abs(yaw_error);
      if (error < best_dist_ - cfg_.progress_min_delta) {
        best_dist_ = error;
        last_progress_ = now;
      } else if (now - last_progress_ > cfg_.stuck_timeout) {
        return finish(NavPhase::Stuck);
      }
      // 提前转正，按剩余距离减到入叉速度。角差过大时留在预备点外完成转向。
      const double carry = yaw_ready && !near_via ? entry : 0.0;
      const double speed = speed_profile(
        std::max(0.0, remaining - 0.035), cfg_.max_linear_speed,
        cfg_.kp_linear, cfg_.linear_decel, cfg_.braking_delay, carry);
      // 大角度先转向，避免夹爪在取放区域边转边扫。小角差平滑恢复平移。
      const double alignment = std::clamp((0.35 - std::abs(yaw_error)) / 0.23, 0.0, 1.0);
      const double scale = alignment * speed / std::max(remaining, 1e-6);
      return limit({scale * (c * dx + s * dy), scale * (-s * dx + c * dy),
        std::copysign(speed_profile(std::abs(yaw_error), cfg_.max_angular_speed,
          cfg_.kp_angular, cfg_.angular_decel, cfg_.braking_delay), yaw_error)}, dt);
    }
  }

  const double ex = goal_.x - current.x;
  const double ey = goal_.y - current.y;
  const double dist = std::hypot(ex, ey);
  const double eyaw = normalize_angle(goal_.yaw - current.yaw);
  if (!std::isfinite(last_dist_)) {
    // 最后约 30 cm、车头已对准的精确直进。行程保护独立于地图定位。
    short_approach_ = guarded_precision_goal() &&
      (through_approach_ || (dist <= 0.36 && std::abs(eyaw) <= 0.09));
    travel_remaining_ = dist + 0.04;
    if (realigning_same_goal_) {
      // ICP 后同点微调，累计行程含来回收敛；首帧残差可能只有几毫米。
      travel_remaining_ = std::max(0.12, travel_remaining_);
    }
    last_chassis_path_ = chassis_path;
    command_time_ = now;
  }
  account_approach_motion(now, chassis_path);
  last_dist_ = dist;
  // 已进入最终容差时可以原地停稳。预算用完后禁止再开出修正指令。
  if (approach_overrun() && (dist > xy_tol_ || std::abs(eyaw) > yaw_tol_)) {
    return finish(NavPhase::Stuck);
  }

  if (now - goal_start_ > cfg_.goal_timeout) {
    return finish(NavPhase::Timeout);
  }

  const bool linear_stopped = std::hypot(measured.vx, measured.vy) <= 0.05 &&
    std::hypot(last_cmd_.vx, last_cmd_.vy) <= 0.05;
  const bool angular_stopped = std::abs(measured.wz) <= 0.08 && std::abs(last_cmd_.wz) <= 0.08;
  if (phase_ == NavPhase::Translate && dist <= xy_tol_) {
    final_positioning_ = true;
    if (!linear_stopped) {
      settled_since_ = -1.0;
      return limit({}, dt);
    }
    phase_ = NavPhase::Rotate;
    rotate_start_ = now;
  } else if (phase_ == NavPhase::Rotate &&
    (dist > 2.0 * xy_tol_ || (dist > xy_tol_ && std::abs(eyaw) <= yaw_tol_ && angular_stopped)))
  {
    // 朝向已经到位时，容差边缘的低速微调属于正常收敛。
    if (dist > 2.0 * xy_tol_ && cfg_.max_pushes > 0 && ++pushes_ >= cfg_.max_pushes) {
      return finish(NavPhase::Stuck);
    }
    phase_ = NavPhase::Translate;
    best_dist_ = std::numeric_limits<double>::infinity();
    last_progress_ = now;
  }

  if (phase_ == NavPhase::Rotate && std::abs(eyaw) <= yaw_tol_ &&
    dist <= xy_tol_ && linear_stopped && angular_stopped)
  {
    if (short_approach_) {
      if (settled_since_ < 0.0) {
        settled_since_ = now;
      }
      if (now - settled_since_ < 0.35) {
        last_cmd_ = Twist2D{};
        return last_cmd_;
      }
    }
    return finish(NavPhase::Arrived);
  }
  settled_since_ = -1.0;

  if (phase_ == NavPhase::Rotate && cfg_.rotate_timeout > 0.0 &&
    now - rotate_start_ > cfg_.rotate_timeout)
  {
    return finish(NavPhase::Stuck);
  }

  // ── 卡住检测（TRANSLATE 看有没有靠近；ROTATE 看被推开次数和转身时长，上面）──
  if (phase_ == NavPhase::Translate) {
    if (dist < best_dist_ - cfg_.progress_min_delta) {
      best_dist_ = dist;
      last_progress_ = now;
    } else if (now - last_progress_ > cfg_.stuck_timeout) {
      return finish(NavPhase::Stuck);
    }
  }

  // ── 世界系（map）下想要的速度 ──
  double vx_w = 0.0;
  double vy_w = 0.0;
  double wz = 0.0;
  if (phase_ == NavPhase::Translate) {
    if (final_positioning_ && !angular_stopped) {
      return limit({}, dt);
    }
    double speed = speed_profile(
      dist, cfg_.max_linear_speed, cfg_.kp_linear, cfg_.linear_decel, cfg_.braking_delay);
    speed = std::max(speed, cfg_.min_linear_speed);
    if (final_positioning_) {
      speed = std::min(speed, cfg_.max_hold_speed);
    }
    if (short_approach_) {
      speed = std::min(speed, cfg_.fine_approach_speed);
    }
    vx_w = ex / dist * speed;  // dist > xy_tolerance > 0，不会除零
    vy_w = ey / dist * speed;
    if (through_approach_ && !final_positioning_) {
      wz = clamp_abs(cfg_.kp_angular * eyaw, cfg_.max_angular_speed);
    }
  } else {
    // 原地转向。偏出容差后先刹住角速度，再低速回位。
    if (std::abs(eyaw) > yaw_tol_) {
      double w = speed_profile(
        std::abs(eyaw), cfg_.max_angular_speed, cfg_.kp_angular, cfg_.angular_decel,
        cfg_.braking_delay);
      w = std::max(w, cfg_.min_angular_speed);
      wz = std::copysign(w, eyaw);
    }
  }

  // ── 转到机器人系（cmd_vel 的约定）──
  const double c = std::cos(current.yaw);
  const double s = std::sin(current.yaw);
  Twist2D target;
  target.vx = c * vx_w + s * vy_w;
  target.vy = -s * vx_w + c * vy_w;
  target.wz = wz;
  // 精确直进过点时，先停住再允许向反方向补正。
  if (short_approach_ && !linear_stopped &&
    (target.vx * measured.vx + target.vy * measured.vy < 0.0 ||
    target.vx * last_cmd_.vx + target.vy * last_cmd_.vy < 0.0))
  {
    return limit({}, dt);
  }
  return limit(target, dt);
}

}  // namespace rc2027_navigation
