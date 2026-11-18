// field_localizer —— 唯一负责发布 map -> lidar_odom 的节点（见
// docs/decisions/0003-field-localization.md）。
//
// 2026-10-03 组长明确定位算感知层，从 Python 版移植成 C++（rclcpp）。
// 10-03 第一次容器编译通过、A4 实测 TF 数值正确。
//
// ── 怎么算 map -> lidar_odom（10-04 改，0003 决定 8）──
// 以前直接把 Start Zone 配置位姿当成 map -> lidar_odom 发，隐含假设
// "Point-LIO 的原点就是车体"。实际上 lidar_odom 原点是开机时 IMU 所在位置、
// 而且重力对齐过，跟车体差一个雷达外参（偏心 + 安装角）。现在改成"锁定"：
//
//   1. 车被摆在某个已知位置（开机：Start Zone；Retry：对应 Retry Zone），
//      这个位置 = map -> base_footprint，从 field_geometry.yaml 读
//   2. 从 TF 查同一时刻的 lidar_odom -> base_footprint（lio_to_base 节点发的，
//      已经把雷达外参算进去了）
//   3. map -> lidar_odom = (map -> base_footprint) * (lidar_odom -> base_footprint)^-1
//      锁定这个值，之后 20 Hz 一直发
//
// 雷达外参只在 URDF 里写一次，这里和 lio_to_base 都从 TF 拿，不会两边各配一份。
// 锁定之前（Point-LIO 还没初始化好）不发 map -> lidar_odom——发一个没对齐的值
// 比不发更糟，查 map -> base_link 的节点会拿到错的位置而不是"查不到"。
//
// ── ICP 精修（2026-10-04，0003 决定 9）──
// 锁定用的是"车正好摆在 zone 中心、标准朝向"的假设，实际摆偏几厘米、几度；Point-LIO
// 跑远了也会累积漂移（仿真实测约 0.7% 行驶距离）。给了 prior_map_path 时：
//   - 每次锁定以后（开机、Retry）自动做一次：攒够 icp_frames 帧点云、车停稳，就把
//     /cloud_registered（lidar_odom 系）配准到先验地图（map 系），直接解出新的
//     map -> lidar_odom（map_matcher.hpp）
//   - /field_localizer/relocalize（std_srvs/Trigger）随时手动触发
//   - 验收（修正量、内点比例……）不过就保持原值并报警，防止对称场地配到镜像位置
//   - **车在动时不修正**：修正那一刻 map -> base_link 会跳一下，导航中跳会让
//     point_navigator 猛地改方向（助教 10-04 提的验收第 5 条）
// 没给 prior_map_path 就跟以前一样只靠锁定。
//
// ── Retry ──
// 操作员把车放进 Retry Zone 之后，调 /field_localizer/retry 服务，
// 带上 zone 参数（"start" 或 "l1"）。这个服务会：
//   1. 立刻发 PAUSED，告诉决策节点停止运动、把当前任务标记为"被打断"
//   2. 调用 point_lio 自带的 /point_lio_reset 清空定位状态
//   3. 等 Point-LIO 重新初始化（固定延时，没有真实的"完成"信号）
//   4. 解除锁定、把目标位姿换成 Retry Zone 的，等定时器用重置以后的
//      lidar_odom -> base_footprint 重新锁定
//   5. 有先验地图的话等锁定后的自动 ICP 做完（最多 kAutoIcpRetryWaitSec），
//      免得 RESUMED 以后决策开始动了才跳一下；**ICP 验收拒绝就算 Retry 失败**（点云和
//      zone 位姿对不上 = 车没摆在这个 zone，按错误位姿开车很危险）
//   6. 发 RESUMED（成功）或 RETRY_FAILED（失败）
// retry_status 的格式是 "<状态> <zone>"（IDLE 不带 zone），例如 "PAUSED l1"、"RESUMED start"。
// 2026-10-04 加的 zone：决策层 Flow 引擎要按 zone 选 Retry 后接着跑哪个流程（0003 决定 6 原文
// 就是 RESUMED(zone, pose)）。只关心状态的订阅者取第一个词
//
// 第 4 步怎么判断 TF 是"重置以后"的：Point-LIO 的 reset 服务只是置个标志位，
// 主循环下一帧才真的重置，服务返回时可能还有重置前的里程计在路上。所以记下
// 调用 reset 之前 TF 的时间戳，要求重新锁定用的 TF 时间戳比它晚至少
// kMinLidarTimeAfterResetSec。比的是里程计自己的时间戳，不跟 now() 比——
// 真车上雷达时间和系统时间不一定是同一个时钟。
//
// /point_lio_reset 是在这个服务的回调里发起的嵌套同步调用，需要
// ReentrantCallbackGroup + MultiThreadedExecutor（见 main()）才不会卡死。
// 定时器在默认回调组，跟 on_retry 在不同线程并发跑，共享状态用 state_mutex_ 保护。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rc2027_navigation/field_geometry.hpp"
#include "rc2027_navigation/floor_height_guard.hpp"
#include "rc2027_navigation/map_matcher.hpp"
#include "rc2027_navigation/pose_math.hpp"
#include "rc2027_navigation_msgs/srv/relatch.hpp"
#include "rc2027_navigation_msgs/srv/retry.hpp"

namespace rc2027_navigation
{

// Point-LIO 重置后到重新稳定之间的固定等待时间。凭经验拍的数，不是量出来的——
// point_lio 的 reset 服务没有"重置完成"信号（见
// external_ws/src/point_lio/docs/RESET_SERVICE_USAGE.md）。
constexpr double kPointLioReinitWaitSec = 2.0;
// 重新锁定用的 TF 时间戳至少要比调 reset 之前晚这么多（雷达时间）。Point-LIO
// 一帧 0.1 s，reset 标志最多一两帧就被处理，1 s 足够确保拿到的是重置以后的
constexpr double kMinLidarTimeAfterResetSec = 1.0;
// 固定等待之后，最多再等这么久让定时器重新锁定，超时算 Retry 失败
constexpr double kRelatchTimeoutSec = 5.0;
// 启动后超过这么久（墙钟）还没锁定，日志才从 INFO 升级成 WARN
constexpr double kLatchWarnAfterSec = 30.0;
// 锁定后的自动 ICP 等这么久（墙钟）还没条件做（点云帧不够 / 车一直在动）就放弃
constexpr double kAutoIcpGiveUpSec = 15.0;
// Retry 重新锁定以后，最多等自动 ICP 这么久再发 RESUMED
constexpr double kAutoIcpRetryWaitSec = 5.0;
// 判断"车停稳"用的位姿历史保留多久（雷达时间）
constexpr double kMotionHistorySec = 2.0;
// 停稳时定位 z 离开「这次锁定的台面」超过这么多，就当成 Point-LIO 的 z 跑了。
// 锁定与换位均包含 z_check_offset，8 cm 即保护，不再让 11 cm 漂移漏过。
// 换层走 relatch，会改掉这个台面高度。卡在 0.28 m 时配准内点只有 0.76，
// 「只有 z 错」的门限不放行，不拉回来检测就一直错、车就停在原地。
constexpr double kZRunaway = kFloorDriftLimit;
// 高度离开锁定台面这么多，里程计的水平也快不跟车了。按上次还在台面上的位置重置再锁定。
constexpr double kZRelock = 0.40;
constexpr double kZRelockCooldownSec = 8.0;

class FieldLocalizerNode : public rclcpp::Node
{
public:
  FieldLocalizerNode()
  : Node("field_localizer")
  {
    side_ = this->declare_parameter<std::string>("side", "red");
    // 本仓库这台车是 BR，2026-10-03 暂定 BR 用靠里的 box_b（TR 用 box_a），
    // 见 docs/decisions/0003-field-localization.md
    start_box_ = this->declare_parameter<std::string>("start_box", "box_b");
    // 开机锁定到哪个 zone：比赛开局是 "start"；仿真测 L1 上的东西（方块检测、L1 走廊定位）时
    // 直接把车放在 L1 Retry Zone 开机用 "l1"，不用每次先回放爬楼梯（2026-10-04 加）
    start_zone_ = this->declare_parameter<std::string>("start_zone", "start");
    const std::string geometry_path = this->declare_parameter<std::string>(
      "field_geometry_path", "/workspace/docs/rules/field_geometry.yaml");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "lidar_odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");

    if (side_ != "red" && side_ != "blue") {
      throw std::invalid_argument("side 参数必须是 'red' 或 'blue'，收到 '" + side_ + "'");
    }

    geometry_ = load_geometry(geometry_path);
    z_check_offset_ = declare_parameter<double>("z_check_offset", 0.0);
    target_pose_ = zone_pose(geometry_, side_, start_zone_, start_box_);
    target_pose_.z += z_check_offset_;
    RCLCPP_INFO(
      get_logger(), "起始位姿 side=%s start_box=%s: x=%.4f y=%.4f yaw=%.4f，等 %s -> %s 出现后锁定",
      side_.c_str(), start_box_.c_str(), target_pose_.x, target_pose_.y, target_pose_.yaw,
      odom_frame_.c_str(), base_frame_.c_str());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FieldLocalizerNode::on_timer, this));

    const auto status_qos = rclcpp::QoS(1).transient_local();
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "field_localizer/retry_status", status_qos);
    publish_status("IDLE");

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Humble 的 create_client/create_service 只收 rmw_qos_profile_t + 回调组
    // （rclcpp::ClientOptions、收 rclcpp::QoS 的重载是 Iron 以后才有的）。
    reset_client_ = this->create_client<std_srvs::srv::Trigger>(
      "point_lio_reset", rmw_qos_profile_services_default, callback_group_);

    relatch_srv_ = this->create_service<rc2027_navigation_msgs::srv::Relatch>(
      "field_localizer/relatch",
      std::bind(
        &FieldLocalizerNode::on_relatch, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    retry_srv_ = this->create_service<rc2027_navigation_msgs::srv::Retry>(
      "field_localizer/retry",
      std::bind(
        &FieldLocalizerNode::on_retry, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    health_srv_ = create_service<std_srvs::srv::Trigger>(
      "field_localizer/check_health",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        response->success = false;
        response->message = "定位未就绪";
        if (!latched_ || floor_transition_ || motion_history_.empty()) {return;}
        if (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_motion_update_).count() > 1.0) {
          response->message = "定位 TF 已过期";
          return;
        }
        const auto base = compose(map_to_odom_, motion_history_.back().second);
        const double since_pull = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_large_z_pull_).count();
        response->success = std::isfinite(base.z) &&
          std::abs(base.z - floor_z_) <= kZRunaway && since_pull > 2.0;
        response->message = response->success ? "高度正常" : "高度漂移，先重定位再观察";
      });
    setup_icp();
  }

private:
  void on_timer()
  {
    bool recover = false;
    Pose2D recover_pose{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latched_) {
        try_latch();
      }
      if (latched_) {
        record_motion();
        // 先拉高度再决定要不要重锁。Point-LIO 重置后的 z 速度清不掉，车一开
        // 一两秒就能冲过 kZRelock，重锁会把位置盖回开出去之前。
        if (!motion_history_.empty()) {
          pull_runaway_z(motion_history_.back().second);
        }
        recover = z_needs_relock(&recover_pose);
        if (recover) {
          map_to_odom_backup_ = map_to_odom_;
          latched_ = false;
          target_pose_ = recover_pose;
          hold_floor_ = false;
          auto_icp_pending_ = false;
        } else {
          check_base_height();
          publish_map_to_odom();
        }
      }
    }
    if (recover) {
      reset_and_relatch(recover_pose);
    }
  }

  // 调用方持有 state_mutex_。高度还在台面上就记下这个位置。跑过 kZRelock 就该重置里程计。
  bool z_needs_relock(Pose2D * pose)
  {
    if (motion_history_.empty() || !have_healthy_) {
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < relock_not_before_) {
      return false;
    }
    const auto base = compose(map_to_odom_, motion_history_.back().second);
    if (std::abs(base.z - floor_z_) < 0.10) {
      healthy_pose_.x = base.x;
      healthy_pose_.y = base.y;
      healthy_pose_.z = floor_z_;
      healthy_pose_.yaw = yaw_of(base);
      return false;
    }
    if (std::abs(base.z - floor_z_) <= kZRelock) {
      return false;
    }
    *pose = healthy_pose_;
    relock_not_before_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(kZRelockCooldownSec));
    RCLCPP_WARN(
      get_logger(),
      "定位 z = %.2f m，离开锁定台面 %.2f m 达 %+.2f m，按 (%.2f, %.2f) 重新锁定。"
      "再开下去估计位置不跟车，导航会在还差几厘米的地方放弃",
      base.z, floor_z_, base.z - floor_z_, pose->x, pose->y);
    return true;
  }

  // 不持有 state_mutex_。重置 Point-LIO，等 try_latch 按 target_pose_ 重新锁定。
  void reset_and_relatch(const Pose2D & pose)
  {
    RCLCPP_WARN(
      get_logger(), "高度跑了，重新锁定到 (%.3f, %.3f) z=%.3f", pose.x, pose.y, pose.z);
    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "高度跑了但 /point_lio_reset 不在，维持原锁定");
      restore_after_failed_relock();
      return;
    }
    double stamp_before_reset = 0.0;
    try {
      stamp_before_reset = rclcpp::Time(
        tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).header.stamp)
        .seconds();
    } catch (const tf2::TransformException &) {
    }
    auto future = reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
      !future.get()->success)
    {
      RCLCPP_ERROR(get_logger(), "高度跑了但 Point-LIO 重置失败，维持原锁定");
      restore_after_failed_relock();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      min_latch_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
      motion_history_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_frames_.clear();
      min_cloud_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(kPointLioReinitWaitSec * 1000)));
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(kRelatchTimeoutSec * 1000));
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latched_) {
          try_latch();
        }
        if (latched_) {
          RCLCPP_INFO(
            get_logger(), "高度跑了之后已重新锁定 (%.2f, %.2f) z=%.2f",
            pose.x, pose.y, pose.z);
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_ERROR(get_logger(), "高度跑了之后重新锁定超时");
    restore_after_failed_relock();
  }

  void restore_after_failed_relock()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latched_) {
      map_to_odom_ = map_to_odom_backup_;
      latched_ = true;
    }
  }

  // 调用方持有 state_mutex_。z 离开锁定台面超过 kZRunaway 之后，每拍把高度拉回来，
  // 行驶中也拉。只改 map->odom 的 z。Point-LIO 内部的 z 速度清不掉（重置之后还会
  // 继续掉），松开的话一两秒就冲过重锁门槛。
  void pull_runaway_z(const Transform3D & odom_to_base)
  {
    if (floor_transition_) {return;}
    const double z = compose(map_to_odom_, odom_to_base).z;
    if (std::abs(z - floor_z_) > kZRunaway) {
      last_large_z_pull_ = std::chrono::steady_clock::now();
    }
    if (!hold_floor_) {
      if (std::abs(z - floor_z_) <= kZRunaway) {
        return;
      }
      hold_floor_ = true;
      RCLCPP_WARN(
        get_logger(),
        "定位 z = %.2f m，离开锁定台面 %.2f m 达 %+.2f m，按台面拉回。"
        "之后每拍都拉，行驶中也不松开，否则会冲过重锁门槛",
        z, floor_z_, z - floor_z_);
    }
    map_to_odom_.z += floor_height_correction(z, floor_z_, hold_floor_, floor_transition_);
  }

  // 停稳时车脚下的高度（map 系）应该是某个台面高度（地面 0 / L1 / L2，仿真再加 z_check_offset）。
  // 差太多说明定位 z 跑了：Point-LIO 的 z 会在车顶着东西抖的时候一路掉（2026-10-06 仿真掉了 0.6 m），
  // 之后方块检测按高度截点云全错，VERIFY 只会说"覆盖率太低"，操作员得猜——这里直接喊出来。
  // 调用方持有 state_mutex_
  void check_base_height()
  {
    if (z_check_tolerance_ <= 0.0 || surface_heights_.empty()) {
      return;
    }
    Transform3D odom_to_base;
    if (!robot_still(&odom_to_base)) {
      return;  // 高度在 on_timer 里已经拉过。这条报警只在停稳时打
    }
    pull_runaway_z(odom_to_base);
    const double z = compose(map_to_odom_, odom_to_base).z;
    double nearest = surface_heights_.front();
    for (const double h : surface_heights_) {
      if (std::abs(z - h) < std::abs(z - nearest)) {nearest = h;}
    }
    const double off = z - nearest;
    if (std::abs(off) > z_check_tolerance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 5000,
        "车停着，定位 z = %.2f m，离最近的台面高度 %.2f m 差 %+.2f m（超过 %.2f m）：定位 z 疑似跑了，"
        "方块检测会全错。试 relocalize（水平几乎没动时会自动修高度），不行就 Retry",
        z, nearest, off, z_check_tolerance_);
    }
  }

  // 调用方持有 state_mutex_
  void try_latch()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      // 启动后头几秒 Point-LIO 还在初始化（仿真里还要等车 spawn 完再过 3 s 才起），
      // 查不到是正常的，用 INFO；等太久才升级成 WARN（2026-10-04 用户看到一启动就刷
      // WARN 以为出错了）
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_wall_).count();
      if (waited < kLatchWarnAfterSec) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), steady_clock_, 5000,
          "等 Point-LIO 初始化（还没有 %s -> %s，正常现象），锁定前不发 map -> %s",
          odom_frame_.c_str(), base_frame_.c_str(), odom_frame_.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock_, 5000,
          "已经等了 %.0f s 还没有 %s -> %s（Point-LIO 没输出？lio_to_base 没起？）: %s",
          waited, odom_frame_.c_str(), base_frame_.c_str(), exc.what());
      }
      return;
    }

    const double stamp = rclcpp::Time(tf.header.stamp).seconds();
    if (stamp < min_latch_stamp_) {
      return;  // 还是 reset 之前的里程计
    }

    Transform3D odom_to_base_now;
    odom_to_base_now.x = tf.transform.translation.x;
    odom_to_base_now.y = tf.transform.translation.y;
    odom_to_base_now.z = tf.transform.translation.z;
    odom_to_base_now.qx = tf.transform.rotation.x;
    odom_to_base_now.qy = tf.transform.rotation.y;
    odom_to_base_now.qz = tf.transform.rotation.z;
    odom_to_base_now.qw = tf.transform.rotation.w;

    map_to_odom_ = map_to_odom(target_pose_, odom_to_base_now);
    latched_ = true;
    floor_z_ = target_pose_.z;
    // 已知换位落点从锁定第一拍就保持高度，不能等漂出阈值才开始拉。
    hold_floor_ = hold_after_latch_;
    floor_transition_ = false;
    last_large_z_pull_ = std::chrono::steady_clock::time_point{};
    healthy_pose_ = target_pose_;
    have_healthy_ = true;
    ++latch_generation_;
    if (matcher_ && icp_on_latch_) {
      auto_icp_pending_ = true;
      auto_icp_since_ = std::chrono::steady_clock::now();
    }
    RCLCPP_INFO(
      get_logger(),
      "已锁定：车在 map 里 (%.4f, %.4f, z %.3f, yaw %.4f)，当时 %s -> %s = (%.4f, %.4f, %.4f, yaw %.4f)"
      " => map -> %s = (%.4f, %.4f, %.4f, yaw %.4f)",
      target_pose_.x, target_pose_.y, target_pose_.z, target_pose_.yaw, odom_frame_.c_str(),
      base_frame_.c_str(),
      odom_to_base_now.x, odom_to_base_now.y, odom_to_base_now.z, yaw_of(odom_to_base_now),
      odom_frame_.c_str(), map_to_odom_.x, map_to_odom_.y, map_to_odom_.z, yaw_of(map_to_odom_));
  }

  // 调用方持有 state_mutex_
  void publish_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    // 不能写成通用的 'odom'：0001 定了 Point-LIO 的世界系叫 'lidar_odom'
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = map_to_odom_.x;
    t.transform.translation.y = map_to_odom_.y;
    t.transform.translation.z = map_to_odom_.z;
    t.transform.rotation.x = map_to_odom_.qx;
    t.transform.rotation.y = map_to_odom_.qy;
    t.transform.rotation.z = map_to_odom_.qz;
    t.transform.rotation.w = map_to_odom_.qw;
    tf_broadcaster_->sendTransform(t);
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void fail(
    std::shared_ptr<rc2027_navigation_msgs::srv::Retry::Response> response,
    const std::string & message)
  {
    RCLCPP_ERROR(get_logger(), "Retry 失败: %s", message.c_str());
    publish_status("RETRY_FAILED " + retry_zone_);
    response->success = false;
    response->message = message;
  }

  // 仿真把车瞬移到新台面：Point-LIO 积分跟不上，旧的 map 位姿差一米左右，
  // relocalize 的 0.3 m 水平门限会拒。这里重置里程计，按调用方给的位姿重新锁定。
  void on_relatch(
    const std::shared_ptr<rc2027_navigation_msgs::srv::Relatch::Request> request,
    std::shared_ptr<rc2027_navigation_msgs::srv::Relatch::Response> response)
  {
    Pose2D pose{};
    pose.x = request->x;
    pose.y = request->y;
    pose.z = request->z + z_check_offset_;
    pose.yaw = request->yaw;
    RCLCPP_WARN(
      get_logger(), "换位重新锁定：(%.3f, %.3f) z=%.3f yaw=%.3f",
      pose.x, pose.y, pose.z, pose.yaw);

    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "/point_lio_reset 不在";
      return;
    }
    double stamp_before_reset = 0.0;
    try {
      stamp_before_reset = rclcpp::Time(
        tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).header.stamp)
        .seconds();
    } catch (const tf2::TransformException &) {
    }
    auto future = reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      response->success = false;
      response->message = "/point_lio_reset 超时";
      return;
    }
    if (!future.get()->success) {
      response->success = false;
      response->message = "point_lio_reset 失败";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latched_ = false;
      target_pose_ = pose;
      hold_after_latch_ = true;
      min_latch_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
      motion_history_.clear();
      auto_icp_pending_ = false;
      last_auto_icp_rejected_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_frames_.clear();
      min_cloud_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(kPointLioReinitWaitSec * 1000)));
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(kRelatchTimeoutSec * 1000));
    bool latched = false;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latched = latched_;
      }
      if (latched) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!latched) {
      response->success = false;
      response->message = "Point-LIO 重置后没有恢复";
      return;
    }
    response->success = true;
    response->message = "已按新位姿锁定";
    RCLCPP_INFO(
      get_logger(), "换位锁定完成：(%.3f, %.3f) z=%.3f yaw=%.3f",
      pose.x, pose.y, pose.z, pose.yaw);
  }

  void on_retry(
    const std::shared_ptr<rc2027_navigation_msgs::srv::Retry::Request> request,
    std::shared_ptr<rc2027_navigation_msgs::srv::Retry::Response> response)
  {
    const std::string zone = request->zone;
    const std::string start_box = request->start_box.empty() ? start_box_ : request->start_box;

    RCLCPP_WARN(get_logger(), "收到 Retry 请求：zone=%s start_box=%s", zone.c_str(), start_box.c_str());

    // 先算好新位姿再动手——算不出来（参数错/yaw 没定）就不要先把决策
    // 节点暂停了却什么都做不成
    Pose2D new_pose{};
    try {
      new_pose = zone_pose(geometry_, side_, zone, start_box);
      new_pose.z += z_check_offset_;
    } catch (const std::invalid_argument & exc) {
      RCLCPP_ERROR(get_logger(), "Retry 请求参数有问题: %s", exc.what());
      response->success = false;
      response->message = exc.what();
      return;
    }

    retry_zone_ = zone;
    publish_status("PAUSED " + zone);

    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
      fail(response, "/point_lio_reset service not available");
      return;
    }

    // reset 之前最后一帧里程计的时间戳，重新锁定时要比它晚
    double stamp_before_reset = 0.0;
    try {
      stamp_before_reset = rclcpp::Time(
        tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).header.stamp)
        .seconds();
    } catch (const tf2::TransformException &) {
      // 查不到就是 0，任何新的 TF 都算"重置以后"
    }

    auto reset_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = reset_client_->async_send_request(reset_request);

    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      fail(response, "/point_lio_reset call timed out");
      return;
    }

    const auto reset_result = future.get();
    if (!reset_result->success) {
      fail(response, "point_lio_reset failed: " + reset_result->message);
      return;
    }

    // 解除锁定：重置期间不发 map -> lidar_odom。继续发旧值的话，旧的
    // map -> lidar_odom 配上重置后从零开始的 lidar_odom -> base_footprint，
    // 别的节点会查到一个完全错误的车体位置；不发的话它们查到的是 TF 缓存里
    // 重置前最后一个一致的位姿（而且这时候决策已经 PAUSED 了）
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latched_ = false;
      target_pose_ = new_pose;
      hold_after_latch_ = false;
      min_latch_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
      // 重置以后 lidar_odom 是一个新的坐标系，之前的位姿历史和点云都作废
      motion_history_.clear();
      auto_icp_pending_ = false;
      last_auto_icp_rejected_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_frames_.clear();
      min_cloud_stamp_ = stamp_before_reset + kMinLidarTimeAfterResetSec;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(kPointLioReinitWaitSec * 1000)));

    // 等定时器用重置以后的 TF 重新锁定
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(kRelatchTimeoutSec * 1000));
    bool latched = false;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latched = latched_;
      }
      if (latched) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!latched) {
      // 不回滚：定时器会继续等，Point-LIO 恢复后自己会锁定；但决策侧要知道
      // 这次 Retry 没在预期时间内完成，操作员可以再调一次
      fail(response, "Point-LIO did not resume publishing after reset");
      return;
    }

    // 等锁定后的自动 ICP（有先验地图时），免得 RESUMED 以后才跳
    std::string icp_note;
    if (matcher_ && icp_on_latch_) {
      const auto icp_deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(kAutoIcpRetryWaitSec * 1000));
      bool pending = true;
      while (std::chrono::steady_clock::now() < icp_deadline) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          pending = auto_icp_pending_;
        }
        if (!pending) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::string icp_message;
      bool icp_rejected = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        icp_message = last_icp_message_;
        icp_rejected = !pending && last_auto_icp_rejected_;
      }
      // ICP 拒绝 = 点云跟 zone 位姿下的地图对不上：车多半没摆在这个 zone 里（或者 zone 选错了）。
      // 这时发 RESUMED 的话决策会按错误位姿开车——在 L1 平台上可能直接开下去。2026-10-04 用户在
      // 地面上调了 retry {zone: l1}，ICP 正确地拒绝了（内点 0.34），但原来照样 RESUMED
      if (icp_rejected) {
        fail(
          response, "ICP 对不上 zone=" + zone + " 的位姿，车可能没摆在这个 zone 里，重新摆好再调一次 retry（" +
          icp_message + "）");
        return;
      }
      icp_note = pending ? "，ICP 精修还没做完（车没停稳？）" : "，ICP: " + icp_message;
    }

    RCLCPP_INFO(
      get_logger(), "Retry 完成，zone=%s: x=%.4f y=%.4f z=%.3f yaw=%.4f%s",
      zone.c_str(), new_pose.x, new_pose.y, new_pose.z, new_pose.yaw, icp_note.c_str());
    publish_status("RESUMED " + zone);

    response->success = true;
    response->message = "resumed at zone=" + zone;
  }

  // ───────────────────────── ICP 精修 ─────────────────────────

  void setup_icp()
  {
    const auto prior_map_path = declare_parameter<std::string>("prior_map_path", "");
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    icp_on_latch_ = declare_parameter<bool>("icp_on_latch", true);
    icp_frames_ = static_cast<size_t>(declare_parameter<int>("icp_frames", 10));
    still_window_ = declare_parameter<double>("still_window", 0.5);
    still_max_move_ = declare_parameter<double>("still_max_move", 0.02);
    still_max_turn_ = declare_parameter<double>("still_max_turn", 0.0175);
    accept_.max_correction_xy =
      declare_parameter<double>("icp_max_correction_xy", accept_.max_correction_xy);
    accept_.max_correction_yaw =
      declare_parameter<double>("icp_max_correction_yaw", accept_.max_correction_yaw);
    accept_.min_inlier_ratio =
      declare_parameter<double>("icp_min_inlier_ratio", accept_.min_inlier_ratio);
    // 方向退化阈值（每点平均约束强度，见 map_matcher.hpp MatchResult::info_xy_min 的注释）：
    // host 射线模拟里退化的位置 0.5～0.7、好的位置 2.3～4.9，取中间。日志里每次都打这两个数，按实测再调
    icp_min_xy_info_ = declare_parameter<double>("icp_min_xy_info", 1.5);
    icp_weak_max_xy_ = declare_parameter<double>("icp_weak_max_xy", 0.05);
    icp_weak_max_yaw_ = declare_parameter<double>("icp_weak_max_yaw", 2.0 * M_PI / 180.0);
    // 死区：要应用的修正同时小于这两个就不改（0 = 关掉）
    icp_deadband_xy_ = declare_parameter<double>("icp_deadband_xy", 0.01);
    icp_deadband_yaw_ = declare_parameter<double>("icp_deadband_yaw", 0.0087);
    icp_deadband_z_ = declare_parameter<double>("icp_deadband_z", 0.02);
    accept_.z_only_max_dz = declare_parameter<double>("icp_z_only_max_dz", accept_.z_only_max_dz);
    accept_.z_only_max_xy = declare_parameter<double>("icp_z_only_max_xy", accept_.z_only_max_xy);
    accept_.z_only_min_inlier =
      declare_parameter<double>("icp_z_only_min_inlier", accept_.z_only_min_inlier);
    MatchParams mp;
    mp.translation_eps = declare_parameter<double>("icp_translation_eps", mp.translation_eps);
    mp.rotation_eps = declare_parameter<double>("icp_rotation_eps", mp.rotation_eps);
    mp.map_voxel = declare_parameter<double>("icp_map_voxel", mp.map_voxel);
    mp.scan_voxel = declare_parameter<double>("icp_scan_voxel", mp.scan_voxel);
    mp.num_threads = declare_parameter<int>("icp_threads", mp.num_threads);
    // 只用场地范围内的点配准（先验地图只有场地；场馆墙、观众席会把内点比例拉低）。
    // 场地含围栏 11.1 m，半边 5.55。以后先验地图如果实地扫进了场外的东西，设 0 关掉
    mp.crop_half_size = declare_parameter<double>("icp_crop_half_size", 5.6);
    // 停稳时高度合理性检查：台面高度来自 field_geometry.yaml 的 levels（地面 0、L1、L2），
    // 仿真场地整体抬了 5 cm 所以 z_check_offset 要传 0.05。0 = 不检查
    z_check_tolerance_ = declare_parameter<double>("z_check_tolerance", 0.10);
    // 上楼时放宽到能过一层（L1 0.6 → L2 0.9 是 0.3 m）。0.10 m 会把这一层的重定位拒掉。
    // 下楼再收回。两层（0.6 m）仍然过不了，避免配到地面。
    z_limit_tight_ = accept_.max_correction_z;
    z_check_tight_ = z_check_tolerance_;
    z_limit_wide_ = declare_parameter<double>("climb_z_allowance", 0.40);
    z_check_wide_ = z_limit_wide_;
    const double z_check_offset = z_check_offset_;
    surface_heights_ = {z_check_offset};
    for (const char * level : {"l1", "l2"}) {
      const auto node = geometry_["levels"][level]["height_m"];
      if (node && !node.IsNull()) {surface_heights_.push_back(node.as<double>() + z_check_offset);}
    }

    if (prior_map_path.empty()) {
      RCLCPP_INFO(get_logger(), "没给 prior_map_path，ICP 精修关闭，只靠锁定");
      return;
    }

    matcher_ = std::make_unique<MapMatcher>(load_pcd_xyz(prior_map_path), mp);
    RCLCPP_INFO(
      get_logger(), "先验地图 %s：降采样后 %zu 点，ICP 精修开启（锁定后自动做一次：%s）",
      prior_map_path.c_str(), matcher_->map_size(), icp_on_latch_ ? "是" : "否");

    // 点云回调单独一个组：配准要几十毫秒，不能堵住收点云
    cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_opts;
    cloud_opts.callback_group = cloud_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&FieldLocalizerNode::on_cloud, this, std::placeholders::_1), cloud_opts);

    // 手动服务和自动定时器放同一个互斥组：两次配准不会同时跑
    icp_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    floor_change_srv_ = create_service<std_srvs::srv::SetBool>(
      "field_localizer/floor_change",
      std::bind(
        &FieldLocalizerNode::on_floor_change, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    relocalize_srv_ = create_service<std_srvs::srv::Trigger>(
      "field_localizer/relocalize",
      std::bind(
        &FieldLocalizerNode::on_relocalize, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, icp_group_);
    auto_icp_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&FieldLocalizerNode::on_auto_icp, this),
      icp_group_);

    // 先验地图发一次给 RViz 看（transient local，晚打开的 RViz 也能收到）
    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "field_localizer/prior_map", rclcpp::QoS(1).transient_local());
    publish_prior_map();
  }

  void publish_prior_map()
  {
    const auto pts = matcher_->map_points();
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = get_clock()->now();
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (const auto & p : pts) {
      *x = p.x();
      *y = p.y();
      *z = p.z();
      ++x;
      ++y;
      ++z;
    }
    prior_map_pub_->publish(msg);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 10000, "点云的 frame_id 是 '%s'，ICP 需要 '%s' 系的点云",
        msg->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    Points pts;
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
        pts.emplace_back(*x, *y, *z);
      }
    }
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (stamp < min_cloud_stamp_) {
      return;  // Retry 重置前的旧坐标系
    }
    cloud_frames_.push_back(std::move(pts));
    while (cloud_frames_.size() > icp_frames_) {
      cloud_frames_.pop_front();
    }
  }

  // 调用方持有 state_mutex_。记录 lidar_odom -> base_footprint 的历史，判断车停没停稳
  void record_motion()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const double stamp = rclcpp::Time(tf.header.stamp).seconds();
    if (!motion_history_.empty() && stamp <= motion_history_.back().first) {
      return;
    }
    Transform3D t;
    t.x = tf.transform.translation.x;
    t.y = tf.transform.translation.y;
    t.z = tf.transform.translation.z;
    t.qx = tf.transform.rotation.x;
    t.qy = tf.transform.rotation.y;
    t.qz = tf.transform.rotation.z;
    t.qw = tf.transform.rotation.w;
    last_motion_update_ = std::chrono::steady_clock::now();
    motion_history_.emplace_back(stamp, t);
    while (!motion_history_.empty() && stamp - motion_history_.front().first > kMotionHistorySec) {
      motion_history_.pop_front();
    }
  }

  // 调用方持有 state_mutex_。最近 still_window_ 秒内车体挪动和转动都很小才算停稳；
  // 停稳时顺便给出当前的 lidar_odom -> base_footprint
  bool robot_still(Transform3D * odom_to_base_now) const
  {
    if (motion_history_.empty()) {
      return false;
    }
    const auto & latest = motion_history_.back();
    // 找窗口起点：最后一个比 latest 早 still_window_ 以上的记录
    const std::pair<double, Transform3D> * start = nullptr;
    for (const auto & entry : motion_history_) {
      if (latest.first - entry.first >= still_window_) {
        start = &entry;
      }
    }
    if (start == nullptr) {
      return false;  // 历史还不够长
    }
    const double move = std::hypot(
      latest.second.x - start->second.x, latest.second.y - start->second.y);
    const double turn = std::abs(normalize_yaw(yaw_of(latest.second) - yaw_of(start->second)));
    *odom_to_base_now = latest.second;
    return move <= still_max_move_ && turn <= still_max_turn_;
  }

  static double normalize_yaw(double a) {return std::atan2(std::sin(a), std::cos(a));}

  // Unchanged：配准正常，但修正量在死区内（或者只剩约束不住的方向），没改 map -> lidar_odom
  enum class IcpOutcome { Applied, Unchanged, Rejected, NotReady };

  IcpOutcome run_icp(const std::string & why, std::string * message)
  {
    Transform3D init;
    Transform3D odom_to_base;
    uint64_t generation = 0;
    AcceptParams accept_now;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latched_) {
        *message = "还没锁定";
        return IcpOutcome::NotReady;
      }
      if (!robot_still(&odom_to_base)) {
        *message = "车在动（或刚动过），停稳再修正";
        return IcpOutcome::NotReady;
      }
      // 先把跑掉的高度拉回锁定台面，否则点云和地图差几十米，内点是 0，高度永远修不回来
      pull_runaway_z(odom_to_base);
      init = map_to_odom_;
      generation = latch_generation_;
      accept_now = accept_;
    }

    Points scan;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (cloud_frames_.size() < icp_frames_) {
        *message = "点云帧不够（" + std::to_string(cloud_frames_.size()) + "/" +
          std::to_string(icp_frames_) + "）";
        return IcpOutcome::NotReady;
      }
      for (const auto & frame : cloud_frames_) {
        scan.insert(scan.end(), frame.begin(), frame.end());
      }
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto result = matcher_->match(scan, init);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    const auto c = correction_between(init, result.map_to_odom, odom_to_base);
    const auto d = decide(result, c, accept_now);

    char buf[320];
    std::snprintf(
      buf, sizeof(buf), "修正 %.1f cm (dx %+.1f, dy %+.1f) / %+.2f°，内点 %.2f，rmse %.1f cm，"
      "%zu 点，平面约束（每点）弱 %.2f / 强 %.2f，%.0f ms", c.xy() * 100, c.dx * 100, c.dy * 100,
      c.dyaw * 180 / M_PI, result.inlier_ratio, result.rmse * 100, result.num_points,
      result.info_xy_min, result.info_xy_max, ms);
    if (!d.accepted) {
      *message = "拒绝：" + d.reason + "（" + buf + "）";
      // 车头摆反的诊断（只提示，不自动采用）：原地转 180° 再配一次，能通过验收就说明多半是
      // 摆反了。2026-10-04 仿真里出现过：L1 Retry 车实际朝北、配置朝南，ICP 掉进错误解
      // （高度修正 -0.6 m）
      const auto flipped = flip_base_yaw(init, odom_to_base);
      const auto r2 = matcher_->match(scan, flipped);
      const auto d2 = decide(r2, correction_between(flipped, r2.map_to_odom, odom_to_base), accept_now);
      if (d2.accepted) {
        char hint[160];
        std::snprintf(
          hint, sizeof(hint), "。提示：把车体原地转 180° 再配能对上（内点 %.2f）——车头是不是摆反了？",
          r2.inlier_ratio);
        *message += hint;
      }
      RCLCPP_WARN(get_logger(), "ICP（%s）%s，保持原值", why.c_str(), message->c_str());
      return IcpOutcome::Rejected;
    }

    // 方向退化（0003 决定 9 补充，2026-10-04）：两个水平方向都约束不住就拒绝；只有一个方向弱就
    // 只修另一个方向。p1（西围栏边、车头朝南）实测 relocalize 稳定往 +y 偏 1～2 cm 就是这个原因
    if (result.info_xy_max < icp_min_xy_info_) {
      *message = std::string("拒绝：两个水平方向的约束都不足（") + buf + "）";
      RCLCPP_WARN(get_logger(), "ICP（%s）%s，保持原值", why.c_str(), message->c_str());
      return IcpOutcome::Rejected;
    }
    Transform3D applied = result.map_to_odom;
    std::string note;
    if (d.large_z) {
      char big[200];
      std::snprintf(
        big, sizeof(big), "；大幅高度修正 %+.2f m 已接受（水平 / 朝向几乎没动、内点 %.2f：Point-LIO 的 z 跑了）",
        c.dz, result.inlier_ratio);
      note = big;
      RCLCPP_WARN(get_logger(), "ICP（%s）%s%s", why.c_str(), buf, big);
    }
    if (result.info_xy_min < icp_min_xy_info_) {
      applied = drop_weak_direction(
        init, result.map_to_odom, odom_to_base, result.weak_dir_x, result.weak_dir_y);
      char weak[160];
      std::snprintf(
        weak, sizeof(weak), "；沿 (%+.2f, %+.2f) 方向约束不足（每点 %.2f < %.2f），这个方向不修",
        result.weak_dir_x, result.weak_dir_y, result.info_xy_min, icp_min_xy_info_);
      note = weak;
    }

    // 死区：水平修正比 ICP 自己的重复性噪声（助教 10-04 实测 0.1～0.7 cm）还小就不改，免得"本来就准还去加噪声"。
    // 高度单独判断（2026-10-05 修：原来死区把 dz 一起丢了，锁定时的 5 cm 高度差一直不修）
    auto ca = correction_between(init, applied, odom_to_base);
    // 单向退化时，投影后的大位移或转角仍可能把下一次放置带出建造点。
    if (result.info_xy_min < icp_min_xy_info_ &&
      (ca.xy() > icp_weak_max_xy_ || std::abs(ca.dyaw) > icp_weak_max_yaw_))
    {
      char reason[192];
      std::snprintf(
        reason, sizeof(reason), "拒绝：单向约束不足，投影后修正 %.1f cm / %+.2f° 超过限值 %.1f cm / %.2f°：",
        ca.xy() * 100, ca.dyaw * 180 / M_PI, icp_weak_max_xy_ * 100,
        icp_weak_max_yaw_ * 180 / M_PI);
      *message = std::string(reason) + buf;
      RCLCPP_WARN(get_logger(), "ICP（%s）%s，保持原值", why.c_str(), message->c_str());
      return IcpOutcome::Rejected;
    }
    if (ca.xy() < icp_deadband_xy_ && std::abs(ca.dyaw) < icp_deadband_yaw_) {
      char small[128];
      if (std::abs(ca.dz) < icp_deadband_z_) {
        std::snprintf(
          small, sizeof(small), "要应用的修正 %.1f cm / %+.2f° / 高度 %+.1f cm 在死区内，不改：",
          ca.xy() * 100, ca.dyaw * 180 / M_PI, ca.dz * 100);
        *message = std::string(small) + buf + note;
        RCLCPP_INFO(get_logger(), "ICP（%s）%s", why.c_str(), message->c_str());
        return IcpOutcome::Unchanged;
      }
      applied = keep_only_z(init, applied, odom_to_base);
      std::snprintf(
        small, sizeof(small), "；水平修正 %.1f cm / %+.2f° 在死区内不改，只修高度 %+.1f cm",
        ca.xy() * 100, ca.dyaw * 180 / M_PI, ca.dz * 100);
      note += small;
      ca = correction_between(init, applied, odom_to_base);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latched_ || generation != latch_generation_) {
        *message = "配准期间重新锁定了（Retry？），结果作废";
        return IcpOutcome::NotReady;
      }
      map_to_odom_ = applied;
      // 配准接受的高度比锁定时的规则书高度更接近网格（仿真常差 5 cm）。之后拉回用这个。
      floor_z_ = compose(applied, odom_to_base).z;
    }
    if (note.empty()) {
      *message = std::string("已修正：") + buf;
    } else {
      char partial[96];
      std::snprintf(
        partial, sizeof(partial), "已修正（部分，实际 %.1f cm / 高度 %+.1f cm）：", ca.xy() * 100,
        ca.dz * 100);
      *message = std::string(partial) + buf + note;
    }
    RCLCPP_INFO(get_logger(), "ICP（%s）%s", why.c_str(), message->c_str());
    return IcpOutcome::Applied;
  }

  void on_auto_icp()
  {
    double waited = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!auto_icp_pending_) {
        return;
      }
      waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - auto_icp_since_).count();
    }
    std::string message;
    const auto outcome = run_icp("锁定后自动精修", &message);
    if (outcome == IcpOutcome::NotReady && waited < kAutoIcpGiveUpSec) {
      return;  // 下次再试
    }
    if (outcome == IcpOutcome::NotReady) {
      message = "等了 " + std::to_string(static_cast<int>(waited)) + " s 还做不了：" + message;
      RCLCPP_WARN(get_logger(), "锁定后自动精修放弃（%s），可以停车后手动调 relocalize",
        message.c_str());
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto_icp_pending_ = false;
    last_auto_icp_rejected_ = (outcome == IcpOutcome::Rejected);
    last_icp_message_ = message;
  }

  void on_floor_change(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    floor_transition_ = request->data;
    if (request->data) {
      hold_floor_ = false;
      accept_.max_correction_z = z_limit_wide_;
      z_check_tolerance_ = z_check_wide_;
      response->message = "上楼：高度门限放宽到一层楼";
      RCLCPP_INFO(
        get_logger(), "上楼，高度门限 %.2f m → %.2f m（下楼再收回）",
        z_limit_tight_, z_limit_wide_);
    } else {
      accept_.max_correction_z = z_limit_tight_;
      z_check_tolerance_ = z_check_tight_;
      response->message = "下楼：高度门限收回";
      RCLCPP_INFO(
        get_logger(), "下楼，高度门限收回 %.2f m", z_limit_tight_);
    }
    response->success = true;
  }

  void on_relocalize(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string message;
    const auto outcome = run_icp("手动", &message);
    response->success = (outcome == IcpOutcome::Applied || outcome == IcpOutcome::Unchanged);
    response->message = message;
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_icp_message_ = message;
  }

  std::string side_;
  std::string start_box_;
  std::string start_zone_;
  std::string retry_zone_;            // 只在 on_retry 里读写（同一时间只有一个 Retry）
  std::string odom_frame_;
  std::string base_frame_;
  YAML::Node geometry_;

  // 以下由 state_mutex_ 保护（定时器线程和 on_retry 线程都会碰）
  std::mutex state_mutex_;
  Pose2D target_pose_{};
  bool latched_{false};
  double min_latch_stamp_{0.0};
  Transform3D map_to_odom_;
  std::chrono::steady_clock::time_point last_motion_update_{};
  double z_check_offset_{0.0};
  bool hold_after_latch_{false};
  bool floor_transition_{false};
  std::chrono::steady_clock::time_point last_large_z_pull_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_srv_;
  double floor_z_{0.6};             // 这次锁定（以及之后接受的配准）认定的台面高度
  bool hold_floor_{false};          // z 已经跑过，之后每拍拉回 floor_z_，行驶中也拉
  Pose2D healthy_pose_{};           // 最近一次高度还在台面上的位置，z 跑了就锁回这里
  bool have_healthy_{false};
  Transform3D map_to_odom_backup_{};
  std::chrono::steady_clock::time_point relock_not_before_{};
  uint64_t latch_generation_{0};      // 每次锁定 +1，ICP 结果回来时对一下，Retry 过就作废
  bool auto_icp_pending_{false};
  bool last_auto_icp_rejected_{false};  // 锁定后自动 ICP 是不是被验收拒了（Retry 用它判断失败）
  std::chrono::steady_clock::time_point auto_icp_since_;
  std::string last_icp_message_;
  std::deque<std::pair<double, Transform3D>> motion_history_;  // (雷达时间, lidar_odom->base)

  // ICP（没给先验地图时 matcher_ 为空，以下都不用）
  std::unique_ptr<MapMatcher> matcher_;
  AcceptParams accept_;
  bool icp_on_latch_{true};
  size_t icp_frames_{10};
  double icp_min_xy_info_{1.5};
  double icp_weak_max_xy_;
  double icp_weak_max_yaw_;
  double icp_deadband_xy_{0.01};
  double icp_deadband_yaw_{0.0087};
  double icp_deadband_z_{0.02};
  double z_check_tolerance_{0.10};
  double z_limit_tight_{0.10};
  double z_check_tight_{0.10};
  double z_limit_wide_{0.40};
  double z_check_wide_{0.40};
  std::vector<double> surface_heights_;
  double still_window_{0.5};
  double still_max_move_{0.02};
  double still_max_turn_{0.0175};
  std::mutex cloud_mutex_;            // 保护下面两个
  std::deque<Points> cloud_frames_;
  double min_cloud_stamp_{0.0};
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::CallbackGroup::SharedPtr icp_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr floor_change_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
  rclcpp::TimerBase::SharedPtr auto_icp_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;

  const std::chrono::steady_clock::time_point started_wall_{std::chrono::steady_clock::now()};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // 日志限频用墙钟

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Service<rc2027_navigation_msgs::srv::Relatch>::SharedPtr relatch_srv_;
  rclcpp::Service<rc2027_navigation_msgs::srv::Retry>::SharedPtr retry_srv_;
};

}  // namespace rc2027_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc2027_navigation::FieldLocalizerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
