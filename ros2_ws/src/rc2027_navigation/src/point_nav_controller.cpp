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
double speed_profile(double d, double max_speed, double kp, double decel)
{
  return std::min({max_speed, kp * d, std::sqrt(2.0 * decel * d)});
}

}  // namespace

PointNavController::PointNavController(const NavConfig & config)
: cfg_(config), xy_tol_(config.xy_tolerance), yaw_tol_(config.yaw_tolerance)
{
}

void PointNavController::set_goal(
  const Pose2D & goal, double now, double xy_tolerance, double yaw_tolerance)
{
  goal_ = goal;
  xy_tol_ = xy_tolerance > 0.0 ? xy_tolerance : cfg_.xy_tolerance;
  yaw_tol_ = yaw_tolerance > 0.0 ? yaw_tolerance : cfg_.yaw_tolerance;
  goal_.yaw = normalize_angle(goal.yaw);
  phase_ = NavPhase::Translate;
  goal_start_ = now;
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
  out.vx = slew(target.vx, last_cmd_.vx, cfg_.max_linear_accel * dt);
  out.vy = slew(target.vy, last_cmd_.vy, cfg_.max_linear_accel * dt);
  out.wz = slew(target.wz, last_cmd_.wz, cfg_.max_angular_accel * dt);
  last_cmd_ = out;
  return out;
}

Twist2D PointNavController::hold(double now)
{
  if (is_active(phase_) && now - goal_start_ > cfg_.goal_timeout) {
    return finish(NavPhase::Timeout);
  }
  last_cmd_ = Twist2D{};
  return Twist2D{};
}

Twist2D PointNavController::update(const Pose2D & current, double now, double dt)
{
  if (!is_active(phase_)) {
    last_cmd_ = Twist2D{};
    return Twist2D{};
  }

  const double ex = goal_.x - current.x;
  const double ey = goal_.y - current.y;
  const double dist = std::hypot(ex, ey);
  const double eyaw = normalize_angle(goal_.yaw - current.yaw);
  last_dist_ = dist;

  if (now - goal_start_ > cfg_.goal_timeout) {
    return finish(NavPhase::Timeout);
  }

  // ── 阶段切换 ──
  if (phase_ == NavPhase::Translate && dist <= xy_tol_) {
    phase_ = NavPhase::Rotate;
    rotate_start_ = now;
  } else if (phase_ == NavPhase::Rotate && dist > 2.0 * xy_tol_) {
    // 转身时被推开了（碰撞、打滑），回去重新平移；反复被推开说明顶着东西，别再挤了
    if (cfg_.max_pushes > 0 && ++pushes_ >= cfg_.max_pushes) {
      return finish(NavPhase::Stuck);
    }
    phase_ = NavPhase::Translate;
    best_dist_ = std::numeric_limits<double>::infinity();
    last_progress_ = now;
  }

  if (phase_ == NavPhase::Rotate && std::abs(eyaw) <= yaw_tol_ &&
    dist <= xy_tol_)
  {
    return finish(NavPhase::Arrived);
  }

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
    double speed = speed_profile(dist, cfg_.max_linear_speed, cfg_.kp_linear, cfg_.linear_decel);
    speed = std::max(speed, cfg_.min_linear_speed);
    vx_w = ex / dist * speed;  // dist > xy_tolerance > 0，不会除零
    vy_w = ey / dist * speed;
  } else {
    // ROTATE：转身 + 小增益稳住 xy（没有最低速度，误差小就慢慢收）
    vx_w = clamp_abs(cfg_.kp_hold * ex, cfg_.max_hold_speed);
    vy_w = clamp_abs(cfg_.kp_hold * ey, cfg_.max_hold_speed);
    if (std::abs(eyaw) > yaw_tol_) {
      double w = speed_profile(
        std::abs(eyaw), cfg_.max_angular_speed, cfg_.kp_angular, cfg_.angular_decel);
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
  return limit(target, dt);
}

}  // namespace rc2027_navigation
