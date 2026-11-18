// 点到点导航的控制律——纯逻辑，不依赖 ROS，host 上能单测。
//
// 2026-10-04 用户定：导航学 GAFA 去年的最终方案（自研点到点，不用 Nav2 规划/控制），
// 为了到点精度；C++ 写；底盘是舵轮（全向，cmd_vel 的 vx/vy/wz 都能用）。
// 见 docs/decisions/0005-navigation.md。
//
// 照搬 GAFA waypoint_navigator.py 的分阶段思路：
//   TRANSLATE：只平移（车头朝向不动），到 xy_tolerance 以内
//   ROTATE：   原地转到目标朝向，同时用小增益把 xy 稳住（舵轮能同时平移+转）
//   到达：     朝向和位置都在容差内
// 平移时不转身：全向底盘边走边转会让定位误差（尤其雷达外参误差）随朝向变化
// 掺进位置里，分开做更好调、更准。
//
// 跟 GAFA 不一样的地方：
//   - 速度曲线：GAFA 在减速区里用 max_speed * dist / decel_distance **替换**了
//     kp * dist，结果离目标 0.29 m 时给 0.97 m/s、0.5 m 时只给 0.2 m/s（越近越快），
//     靠加速度限幅才没出事。这里改成三者取最小：max_speed、kp*dist、
//     sqrt(2*decel*dist)（保证按 decel 刹得住）
//   - ROTATE 阶段继续纠正 xy（GAFA 转身时完全不管位置，转完可能已经偏出容差）
//   - 转身时被推开太远（> 2 倍 xy 容差）回到 TRANSLATE
//   - 坐标直接用 TF 的 map 系（GAFA 在导航器里用 loc_offset 做场地坐标换算，
//     我们这件事 field_localizer 已经在 TF 里做了）
//   - 视觉伺服交接还没做（没有视觉节点），以后要加的话加在"到达"之后
#pragma once

#include <string>

#include "rc2027_navigation/field_geometry.hpp"

namespace rc2027_navigation
{

struct NavConfig
{
  double max_linear_speed{1.0};      // m/s
  double min_linear_speed{0.03};     // m/s，TRANSLATE 时的下限，防止最后几厘米蹭不动
  double max_angular_speed{2.0};     // rad/s
  double min_angular_speed{0.05};    // rad/s，ROTATE 时的下限
  double kp_linear{1.5};             // 1/s，最后一段的指数收敛速度
  double kp_angular{2.0};            // 1/s
  double kp_hold{1.0};               // 1/s，ROTATE 阶段稳住 xy 的增益
  double max_hold_speed{0.2};        // m/s，ROTATE 阶段稳住 xy 的速度上限
  double linear_decel{0.8};          // m/s²，刹车曲线 sqrt(2*a*d) 用的减速度
  double angular_decel{3.0};         // rad/s²
  double max_linear_accel{1.0};      // m/s²，指令变化率限幅（加速和减速都限）
  double max_angular_accel{4.0};     // rad/s²
  double xy_tolerance{0.03};         // m
  double yaw_tolerance{0.03};        // rad（约 1.7°）
  double goal_timeout{30.0};         // s，整个目标的时间上限
  double stuck_timeout{5.0};         // s，TRANSLATE 阶段这么久没靠近 progress_min_delta 就算卡住
  double progress_min_delta{0.02};   // m
  // ROTATE 阶段的卡住检测（2026-10-06 加）：原来只有 TRANSLATE 查卡住，车贴着方块 / 顶着围栏时
  // "ROTATE → 被推开 → 回 TRANSLATE → 又进 ROTATE"来回切，每次回 TRANSLATE 都重置进展计时，永远不判卡住，
  // 只有 30 s 总超时兜底——仿真里 NAV @pick 就这样顶着方块挤了 11.5 s（正常 2.6 s），Point-LIO 被抖得 z 掉了 0.6 m
  int max_pushes{3};                 // 转身时被推开（回 TRANSLATE）这么多次就算卡住；0 = 不查
  double rotate_timeout{8.0};        // s，进了 ROTATE 这么久还没到位就算卡住；0 = 不查
};

/// 机器人系下的速度指令（跟 geometry_msgs/Twist 的 linear.x/linear.y/angular.z 对应）
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

enum class NavPhase
{
  Idle,
  Translate,
  Rotate,
  Arrived,
  Timeout,
  Stuck,
};

std::string to_string(NavPhase phase);

/// 是不是"导航中"（Translate/Rotate）
bool is_active(NavPhase phase);

/// 角度折回 [-pi, pi]
double normalize_angle(double a);

class PointNavController
{
public:
  explicit PointNavController(const NavConfig & config = NavConfig{});

  /// 设新目标（map 系），会覆盖正在进行的目标。now 单位秒，只用来算超时。
  /// xy_tolerance / yaw_tolerance 是这一个目标的到位门限，<= 0 用 NavConfig 里的
  /// （拿 / 放前最后一步要比普通走点更严，2026-10-04 加）
  void set_goal(
    const Pose2D & goal, double now, double xy_tolerance = 0.0, double yaw_tolerance = 0.0);

  /// 取消，回到 Idle，下一次 update 输出 0
  void cancel();

  /// 每个控制周期调一次。current 是 map 系下车的当前位姿，dt 是控制周期（秒），
  /// 用来做加速度限幅。返回机器人系下的速度指令；到达/超时/卡住时返回 0 并切换阶段
  Twist2D update(const Pose2D & current, double now, double dt);

  /// 拿不到当前位姿时调这个：输出 0（并清掉限幅的历史，下次从 0 开始加速），
  /// 但照样检查总超时，定位一直不回来也不会永远挂着
  Twist2D hold(double now);

  NavPhase phase() const {return phase_;}
  const Pose2D & goal() const {return goal_;}
  /// 最近一次 update 时离目标的距离（m）
  double distance_remaining() const {return last_dist_;}
  /// 当前目标实际用的到位门限
  double xy_tolerance() const {return xy_tol_;}
  double yaw_tolerance() const {return yaw_tol_;}

private:
  Twist2D finish(NavPhase phase);
  Twist2D limit(const Twist2D & target, double dt);

  NavConfig cfg_;
  NavPhase phase_{NavPhase::Idle};
  Pose2D goal_{};
  double xy_tol_{0.0};
  double yaw_tol_{0.0};
  double goal_start_{0.0};
  double best_dist_{0.0};
  double rotate_start_{0.0};
  int pushes_{0};
  double last_progress_{0.0};
  double last_dist_{0.0};
  Twist2D last_cmd_{};
};

}  // namespace rc2027_navigation
