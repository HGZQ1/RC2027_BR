// point_nav_controller 的单测——assert 风格 + main()，ctest 注册，host 上 g++ 就能跑。
//
// 用一个"理想全向车"（指令是多少就走多少，没有打滑和延迟）闭环跑控制律，检查：
// 能到、平移阶段不转身、跨 ±180° 走近路、加速度限幅、超时/卡住/被推开这些分支。
// 理想车只能证明控制律本身的逻辑和数学对，调参还是得在 Gazebo/真车上做。

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "rc2027_navigation/point_nav_controller.hpp"

using rc2027_navigation::NavConfig;
using rc2027_navigation::NavPhase;
using rc2027_navigation::normalize_angle;
using rc2027_navigation::PointNavController;
using rc2027_navigation::Pose2D;
using rc2027_navigation::to_string;
using rc2027_navigation::Twist2D;

namespace
{

int g_failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    std::cerr << "FAIL: " << what << std::endl;
    ++g_failures;
  } else {
    std::cout << "ok: " << what << std::endl;
  }
}

constexpr double kDt = 0.05;  // 20 Hz，跟节点默认一样

// 理想全向车：机器人系速度转到世界系积分
void integrate(Pose2D & p, const Twist2D & cmd, double dt)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  p.x += (c * cmd.vx - s * cmd.vy) * dt;
  p.y += (s * cmd.vx + c * cmd.vy) * dt;
  p.yaw = normalize_angle(p.yaw + cmd.wz * dt);
}

struct RunResult
{
  NavPhase final_phase;
  Pose2D final_pose;
  double time;
  double max_speed;
  double max_accel;
  double yaw_change_while_translating;
  double total_abs_rotation;
  std::vector<NavPhase> phases;
};

RunResult run(
  PointNavController & ctl, Pose2D pose, const Pose2D & goal, double max_time = 60.0,
  double xy_tol = 0.0, double yaw_tol = 0.0)
{
  RunResult r{};
  double t = 0.0;
  ctl.set_goal(goal, t, xy_tol, yaw_tol);
  Twist2D prev{};
  const double yaw0 = pose.yaw;
  r.phases.push_back(ctl.phase());
  while (t < max_time) {
    const auto cmd = ctl.update(pose, t, kDt);
    if (ctl.phase() != r.phases.back()) {
      r.phases.push_back(ctl.phase());
    }
    if (!rc2027_navigation::is_active(ctl.phase())) {
      break;
    }
    r.max_speed = std::max(r.max_speed, std::hypot(cmd.vx, cmd.vy));
    // 限幅是每个轴单独做的，按单轴最大值比
    r.max_accel = std::max(
      {r.max_accel, std::abs(cmd.vx - prev.vx) / kDt, std::abs(cmd.vy - prev.vy) / kDt});
    if (ctl.phase() == NavPhase::Translate) {
      r.yaw_change_while_translating =
        std::max(r.yaw_change_while_translating, std::abs(normalize_angle(pose.yaw - yaw0)));
    }
    r.total_abs_rotation += std::abs(cmd.wz) * kDt;
    prev = cmd;
    integrate(pose, cmd, kDt);
    t += kDt;
  }
  r.final_phase = ctl.phase();
  r.final_pose = pose;
  r.time = t;
  return r;
}

void test_reaches_far_goal()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  // 红方 BR 起点 -> 场地中间某点，朝向也变
  const Pose2D start{-4.35, -5.15, M_PI / 2};
  const Pose2D goal{1.2, 3.4, -2.0};
  const auto r = run(ctl, start, goal);
  const double dist = std::hypot(r.final_pose.x - goal.x, r.final_pose.y - goal.y);
  const double eyaw = std::abs(normalize_angle(r.final_pose.yaw - goal.yaw));
  check(r.final_phase == NavPhase::Arrived, "far goal: ARRIVED (got " + to_string(r.final_phase) + ")");
  check(dist <= cfg.xy_tolerance, "far goal: 位置误差 " + std::to_string(dist) + " <= 容差");
  check(eyaw <= cfg.yaw_tolerance, "far goal: 朝向误差 " + std::to_string(eyaw) + " <= 容差");
  check(r.yaw_change_while_translating < 1e-9, "far goal: 平移阶段不转身");
  check(r.max_speed <= cfg.max_linear_speed + 1e-9, "far goal: 不超最大速度");
  check(
    r.phases.size() == 3 && r.phases[0] == NavPhase::Translate &&
    r.phases[1] == NavPhase::Rotate && r.phases[2] == NavPhase::Arrived,
    "far goal: 阶段顺序 TRANSLATE -> ROTATE -> ARRIVED");
  std::cout << "   (far goal 用时 " << r.time << " s)" << std::endl;
}

void test_accel_limit()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  const auto r = run(ctl, {0, 0, 0}, {3.0, 0.0, 0.0});
  check(
    r.max_accel <= cfg.max_linear_accel + 1e-6,
    "加速度限幅: " + std::to_string(r.max_accel) + " <= " + std::to_string(cfg.max_linear_accel));
}

void test_yaw_takes_short_way_across_pi()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  // 170° -> -170°：近路是再转 +20°，不是倒转 340°
  const auto r = run(ctl, {0, 0, M_PI * 170 / 180}, {0.0, 0.0, -M_PI * 170 / 180});
  check(r.final_phase == NavPhase::Arrived, "跨 ±180°: ARRIVED");
  check(r.total_abs_rotation < M_PI * 25 / 180, "跨 ±180°: 总共只转了约 20°");
}

void test_already_there()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  const auto r = run(ctl, {1.0, 1.0, 0.5}, {1.0, 1.0, 0.5});
  check(r.final_phase == NavPhase::Arrived && r.time < 0.2, "原地目标: 立刻 ARRIVED");
}

void test_stuck_when_robot_does_not_move()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  ctl.set_goal({2.0, 0.0, 0.0}, 0.0);
  const Pose2D blocked{0.0, 0.0, 0.0};  // 怎么发指令都不动（撞墙）
  double t = 0.0;
  while (t < 20.0 && rc2027_navigation::is_active(ctl.phase())) {
    ctl.update(blocked, t, kDt);
    t += kDt;
  }
  check(ctl.phase() == NavPhase::Stuck, "撞墙不动: STUCK");
  check(
    std::abs(t - cfg.stuck_timeout) < 0.2,
    "撞墙不动: 大约 stuck_timeout 后判定（" + std::to_string(t) + " s）");
}

void test_goal_timeout()
{
  NavConfig cfg;
  cfg.goal_timeout = 1.0;
  PointNavController ctl(cfg);
  const auto r = run(ctl, {0, 0, 0}, {5.0, 0.0, 0.0});
  check(r.final_phase == NavPhase::Timeout, "总超时: TIMEOUT");
}

void test_hold_times_out_without_pose()
{
  NavConfig cfg;
  cfg.goal_timeout = 1.0;
  PointNavController ctl(cfg);
  ctl.set_goal({1.0, 0.0, 0.0}, 0.0);
  const auto cmd = ctl.hold(0.5);
  check(cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.wz == 0.0, "没位姿: 输出 0");
  check(rc2027_navigation::is_active(ctl.phase()), "没位姿: 超时前还在导航中");
  ctl.hold(1.5);
  check(ctl.phase() == NavPhase::Timeout, "没位姿: 超过总超时也会 TIMEOUT");
}

void test_pushed_away_while_rotating_goes_back_to_translate()
{
  NavConfig cfg;
  PointNavController ctl(cfg);
  ctl.set_goal({0.0, 0.0, M_PI / 2}, 0.0);
  ctl.update({0.01, 0.0, 0.0}, 0.0, kDt);
  check(ctl.phase() == NavPhase::Rotate, "推开: 先进入 ROTATE");
  ctl.update({0.2, 0.0, 0.3}, kDt, kDt);
  check(ctl.phase() == NavPhase::Translate, "推开: 偏出 2 倍容差回到 TRANSLATE");
}

void test_repeatedly_pushed_away_is_stuck()
{
  // 贴着方块 / 围栏：到点 → 转身时被顶开 → 回 TRANSLATE → 又到点……来回切不会触发 TRANSLATE 的卡住检测，
  // 反复 max_pushes 次要判 STUCK（2026-10-06 仿真 NAV @pick 顶着方块挤了 11.5 s）
  NavConfig cfg;
  PointNavController ctl(cfg);
  ctl.set_goal({0.0, 0.0, M_PI / 2}, 0.0);
  double t = 0.0;
  int cycles = 0;
  while (rc2027_navigation::is_active(ctl.phase()) && cycles < 10) {
    ctl.update({0.01, 0.0, 0.0}, t, kDt);    // 到点，进 ROTATE
    t += kDt;
    ctl.update({0.2, 0.0, 0.3}, t, kDt);     // 被顶开
    t += kDt;
    ++cycles;
  }
  check(ctl.phase() == NavPhase::Stuck, "反复被推开: STUCK");
  check(cycles == cfg.max_pushes, "反复被推开: 第 " + std::to_string(cycles) + " 次就判");

  NavConfig off;
  off.max_pushes = 0;
  PointNavController ctl2(off);
  ctl2.set_goal({0.0, 0.0, M_PI / 2}, 0.0);
  for (int i = 0; i < 10; ++i) {
    ctl2.update({0.01, 0.0, 0.0}, i * 2 * kDt, kDt);
    ctl2.update({0.2, 0.0, 0.3}, (i * 2 + 1) * kDt, kDt);
  }
  check(rc2027_navigation::is_active(ctl2.phase()), "max_pushes = 0: 不查");
}

void test_rotate_timeout()
{
  // 到了位置、朝向怎么都转不到（顶着东西转不动）
  NavConfig cfg;
  PointNavController ctl(cfg);
  ctl.set_goal({0.0, 0.0, M_PI / 2}, 0.0);
  const Pose2D stuck{0.0, 0.0, 0.0};
  double t = 0.0;
  while (t < 20.0 && rc2027_navigation::is_active(ctl.phase())) {
    ctl.update(stuck, t, kDt);
    t += kDt;
  }
  check(ctl.phase() == NavPhase::Stuck, "转不动: STUCK");
  check(std::abs(t - cfg.rotate_timeout) < 0.3, "转不动: 大约 rotate_timeout 后判（" + std::to_string(t) + " s）");
}

void test_cancel()
{
  PointNavController ctl;
  ctl.set_goal({1.0, 0.0, 0.0}, 0.0);
  ctl.update({0.0, 0.0, 0.0}, 0.0, kDt);
  ctl.cancel();
  const auto cmd = ctl.update({0.0, 0.0, 0.0}, kDt, kDt);
  check(ctl.phase() == NavPhase::Idle, "取消: IDLE");
  check(cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.wz == 0.0, "取消: 输出 0");
}

void test_per_goal_tolerance()
{
  // 拿 / 放前最后一步：单个目标用更严的门限（1 cm / 1°），不影响下一个普通目标
  NavConfig cfg;
  PointNavController ctl(cfg);
  const Pose2D goal{0.5, 0.2, 0.3};
  const auto r = run(ctl, {0.48, 0.18, 0.25}, goal, 60.0, 0.01, 0.0175);
  const double dist = std::hypot(r.final_pose.x - goal.x, r.final_pose.y - goal.y);
  const double eyaw = std::abs(normalize_angle(r.final_pose.yaw - goal.yaw));
  check(r.final_phase == NavPhase::Arrived, "严门限: ARRIVED (got " + to_string(r.final_phase) + ")");
  check(dist <= 0.01, "严门限: 位置误差 " + std::to_string(dist) + " <= 1 cm");
  check(eyaw <= 0.0175, "严门限: 朝向误差 " + std::to_string(eyaw) + " <= 1°");
  check(ctl.xy_tolerance() == 0.01, "严门限: 控制器用的是这个目标的门限");

  // 起点离目标 2.5 cm：普通门限（3 cm）下直接算到了（不动），严门限下要真的走过去
  PointNavController normal(cfg);
  const auto r2 = run(normal, {0.475, 0.2, 0.3}, goal);
  check(
    r2.final_phase == NavPhase::Arrived && r2.max_speed == 0.0,
    "普通门限: 2.5 cm 以内不动（这就是为什么拿放前要用严门限再发一次）");
  normal.set_goal(goal, 0.0);
  check(normal.xy_tolerance() == cfg.xy_tolerance, "不给门限时回到节点参数");
}

}  // namespace

int main()
{
  test_reaches_far_goal();
  test_accel_limit();
  test_yaw_takes_short_way_across_pi();
  test_already_there();
  test_stuck_when_robot_does_not_move();
  test_goal_timeout();
  test_hold_times_out_without_pose();
  test_pushed_away_while_rotating_goes_back_to_translate();
  test_repeatedly_pushed_away_is_stuck();
  test_rotate_timeout();
  test_cancel();
  test_per_goal_tolerance();

  if (g_failures != 0) {
    std::cerr << g_failures << " 个检查失败" << std::endl;
    return 1;
  }
  std::cout << "全部通过" << std::endl;
  return 0;
}
