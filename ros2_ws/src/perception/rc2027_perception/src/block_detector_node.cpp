// block_detector —— 雷达方块检测的 ROS 外壳（算法在 block_detector.hpp，有单测）。
//
//   - 订阅**雷达原始点云**（PointCloud2，雷达系），留最近 frames 帧；用的时候按每帧时间戳查 TF 转到 map。
//     不能用 Point-LIO 的 /cloud_registered：那是 feats_down_world，按 filter_size_surf 0.2 m 体素降采样过
//     （每帧约 1600 点、7 Hz），350 mm 的方块一个面只剩一两个点、高度被量化成 0.2 m 一档——2026-10-05 助教
//     仿真里量出 0.29 / 0.61 m 这种对不上的高度就是这个原因。原始点云没去畸变，所以要**停稳了再看**
//   - 区域从 field_geometry.yaml 读：building_spots → spot_01..spot_10，transfer_area → transfer_red /
//     transfer_blue；台面预期高度按 levels 算（L2 / L1 / 地面），再加 surface_z_offset（仿真 0.05）
//   - 用的时候查 TF：map <- 点云的 frame（lidar_odom），map <- sensor_frame（雷达在哪，判断看到的是哪一面）
//
// 两种用法：
//   1. 服务 ~/detect（rc2027_perception_msgs/srv/DetectBlocks）：停稳以后看一眼某个区域。
//      Flow 里"到锚点 → 看 → 算拿取 / 放置位姿"就用这个。结果也发一份到 ~/regions
//   2. 定时扫（scan_rate > 0）：scan_regions 里看得到的区域每秒发一次 ~/regions（RegionStateArray）
//      + ~/markers（RViz：方块画成方盒子，绿 = 地块、蓝 = 天空块、红 = 不认识）。
//      世界模型（rc2027_decision 的 field_state → /field_state）吃 ~/regions
//
// 车在动的时候也能看（点云已经在 lidar_odom 里对齐过），但放 / 拿之前应该停稳了看。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include "rc2027_perception/block_detector.hpp"
#include "rc2027_perception_msgs/msg/region_state_array.hpp"
#include "rc2027_perception_msgs/srv/detect_blocks.hpp"

namespace rc2027_perception
{

using rc2027_perception_msgs::msg::RegionState;
using rc2027_perception_msgs::msg::RegionStateArray;
using rc2027_perception_msgs::srv::DetectBlocks;

namespace
{

/// 场地配置 → 区域列表
std::vector<Region> load_regions(
  const std::string & path, double spot_half_size, double transfer_inset, double z_offset)
{
  const YAML::Node g = YAML::LoadFile(path);
  const auto l1 = g["levels"]["l1"];
  const auto l2 = g["levels"]["l2"];
  auto surface_at = [&](double x, double y) {
      auto inside = [&](const YAML::Node & level) {
          return std::abs(x) <= level["size_m"][0].as<double>() / 2 &&
                 std::abs(y) <= level["size_m"][1].as<double>() / 2;
        };
      if (inside(l2)) {return l2["height_m"].as<double>();}
      if (inside(l1)) {return l1["height_m"].as<double>();}
      return 0.0;
    };

  std::vector<Region> out;
  for (const auto & kv : g["building_spots"]) {
    Region r;
    r.name = kv.first.as<std::string>();
    r.cx = kv.second[0].as<double>();
    r.cy = kv.second[1].as<double>();
    r.half_x = r.half_y = spot_half_size;
    r.expected_surface_z = surface_at(r.cx, r.cy) + z_offset;
    out.push_back(r);
  }
  if (g["transfer_area"]) {
    for (const auto & kv : g["transfer_area"]) {
      Region r;
      r.name = "transfer_" + kv.first.as<std::string>();
      r.cx = kv.second["center"][0].as<double>();
      r.cy = kv.second["center"][1].as<double>();
      r.half_x = kv.second["size"][0].as<double>() / 2 - transfer_inset;
      r.half_y = kv.second["size"][1].as<double>() / 2 - transfer_inset;
      r.expected_surface_z = kv.second["z"].as<double>() + z_offset;
      out.push_back(r);
    }
  }
  return out;
}

tf2::Transform to_tf(const geometry_msgs::msg::TransformStamped & t)
{
  const auto & q = t.transform.rotation;
  const auto & v = t.transform.translation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(v.x, v.y, v.z));
}

}  // namespace

class BlockDetectorNode : public rclcpp::Node
{
public:
  BlockDetectorNode()
  : Node("block_detector")
  {
    const auto geometry_path = declare_parameter<std::string>(
      "field_geometry_path", "/workspace/docs/rules/field_geometry.yaml");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar_link");
    // cloud_crop 裁掉车体 + 货仓以后的 PointCloud2（雷达系）。真车驱动只发 CustomMsg，这一路由 cloud_crop 转出来；
    // 仿真不开裁剪框时可以直接用插件的 /livox/lidar_PointCloud2（frame livox_sim，每帧 20000 点）
    const auto cloud_topic =
      declare_parameter<std::string>("cloud_topic", "/cloud_crop/points");
    min_range_ = declare_parameter<double>("min_range", 0.3);   // 雷达附近的点（车自己、货）不要
    max_frames_ = static_cast<size_t>(declare_parameter<int>("frames", 5));
    // Gazebo 场地整体比规则高 5 cm：仿真里设 0.05（台面高度本来就先用点云找，这个只在看不到台面时用）
    const double z_offset = declare_parameter<double>("surface_z_offset", 0.0);
    const double spot_half = declare_parameter<double>("spot_half_size", 0.40);
    const double transfer_inset = declare_parameter<double>("transfer_inset", 0.02);
    view_max_elevation_ =
      declare_parameter<double>("max_view_elevation_deg", 22.0) * M_PI / 180.0;
    const double scan_rate = declare_parameter<double>("scan_rate", 1.0);
    auto scan_regions = declare_parameter<std::vector<std::string>>(
      "scan_regions", std::vector<std::string>{""});

    regions_ = load_regions(geometry_path, spot_half, transfer_inset, z_offset);
    for (const auto & name : scan_regions) {
      if (name.empty()) {
        continue;
      }
      if (!find_region(name)) {
        throw std::invalid_argument("scan_regions 里的 " + name + " 不在场地配置里");
      }
      scan_names_.push_back(name);
    }
    if (scan_names_.empty()) {
      for (const auto & r : regions_) {scan_names_.push_back(r.name);}
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&BlockDetectorNode::on_cloud, this, std::placeholders::_1));
    regions_pub_ = create_publisher<RegionStateArray>("~/regions", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    detect_srv_ = create_service<DetectBlocks>(
      "~/detect",
      std::bind(&BlockDetectorNode::on_detect, this, std::placeholders::_1, std::placeholders::_2));
    if (scan_rate > 0.0) {
      scan_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / scan_rate), std::bind(&BlockDetectorNode::on_scan, this));
    }

    std::string names;
    for (const auto & r : regions_) {names += " " + r.name;}
    RCLCPP_INFO(
      get_logger(), "方块检测：%zu 个区域（%s），点云 %s 留 %zu 帧，定时扫 %.1f Hz",
      regions_.size(), names.c_str(), cloud_topic.c_str(), max_frames_, scan_rate);
  }

private:
  struct Frame
  {
    std::string frame_id;
    rclcpp::Time stamp;
    std::vector<Point3> points;
  };

  const Region * find_region(const std::string & name) const
  {
    for (const auto & r : regions_) {
      if (r.name == name) {return &r;}
    }
    return nullptr;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    Frame f;
    f.frame_id = msg->header.frame_id;
    f.stamp = msg->header.stamp;
    f.points.reserve(msg->width * msg->height);
    const float min_sq = static_cast<float>(min_range_ * min_range_);
    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
        (*x) * (*x) + (*y) * (*y) + (*z) * (*z) >= min_sq)
      {
        f.points.push_back({*x, *y, *z});
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(std::move(f));
    while (frames_.size() > std::max<size_t>(max_frames_, 20)) {
      frames_.pop_front();  // 多留一些，服务可以要更多帧
    }
  }

  /// 最近 n 帧点云转到 map 系 + 雷达位置。失败返回 false 和原因
  bool gather(size_t n, std::vector<Point3> * points, Viewpoint * view, rclcpp::Time * stamp, std::string * why)
  {
    std::vector<Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (frames_.empty()) {
        *why = "还没收到点云";
        return false;
      }
      const size_t k = std::min(n, frames_.size());
      frames.assign(frames_.end() - static_cast<std::ptrdiff_t>(k), frames_.end());
    }
    try {
      const auto sensor = tf_buffer_->lookupTransform(map_frame_, sensor_frame_, tf2::TimePointZero);
      view->x = sensor.transform.translation.x;
      view->y = sensor.transform.translation.y;
      view->z = sensor.transform.translation.z;
      view->max_elevation = view_max_elevation_;
      size_t used = 0;
      for (const auto & f : frames) {
        // 按这一帧的时间戳查（车在动时每帧位置不一样）；TF 缓存里没有这个时刻就跳过这一帧
        geometry_msgs::msg::TransformStamped ts;
        try {
          ts = tf_buffer_->lookupTransform(map_frame_, f.frame_id, f.stamp);
        } catch (const tf2::TransformException &) {
          continue;
        }
        *stamp = f.stamp;
        ++used;
        const tf2::Transform t = to_tf(ts);
        for (const auto & p : f.points) {
          const tf2::Vector3 m = t * tf2::Vector3(p.x, p.y, p.z);
          points->push_back({static_cast<float>(m.x()), static_cast<float>(m.y()), static_cast<float>(m.z())});
        }
      }
      if (used == 0) {
        *why = "最近 " + std::to_string(frames.size()) + " 帧点云的时刻都查不到 TF（定位刚起来 / 时钟不对？）";
        return false;
      }
    } catch (const tf2::TransformException & e) {
      *why = std::string("查不到 TF（定位没起来？）: ") + e.what();
      return false;
    }
    return true;
  }

  RegionState to_msg(const Region & region, const RegionResult & r, const rclcpp::Time & stamp)
  {
    RegionState s;
    s.stamp = stamp;
    s.region = region.name;
    s.visible = r.visible;
    s.coverage = static_cast<float>(r.coverage);
    s.surface_z = r.surface_z;
    s.surface_measured = r.surface_measured;
    s.ground_layers = static_cast<uint8_t>(r.ground_layers);
    s.has_sky = r.has_sky;
    s.top_height = r.top_height;
    s.top_maybe_clipped = r.top_maybe_clipped;
    s.num_points = static_cast<uint32_t>(r.num_points);
    s.note = r.note;
    for (const auto & b : r.blocks) {
      rc2027_perception_msgs::msg::Block m;
      m.type = b.type == BlockType::Ground ? m.GROUND : b.type == BlockType::Sky ? m.SKY : m.UNKNOWN;
      m.x = b.x;
      m.y = b.y;
      m.z_bottom = b.z_bottom;
      m.yaw = b.yaw;
      m.extent_a = b.extent_a;
      m.extent_b = b.extent_b;
      m.layer = static_cast<uint8_t>(b.layer);
      m.num_points = static_cast<uint32_t>(b.num_points);
      m.center_reliable = b.center_reliable;
      s.blocks.push_back(m);
    }
    return s;
  }

  static std::string summary(const RegionState & s)
  {
    char buf[200];
    std::snprintf(
      buf, sizeof(buf), "%s: %s，%d 层地块%s，%zu 块，覆盖 %.0f%%", s.region.c_str(),
      s.visible ? "看得到" : "看不到", s.ground_layers, s.has_sky ? " + 天空块" : "",
      s.blocks.size(), s.coverage * 100);
    return std::string(buf) + (s.note.empty() ? "" : "（" + s.note + "）");
  }

  void on_detect(
    const std::shared_ptr<DetectBlocks::Request> req, std::shared_ptr<DetectBlocks::Response> res)
  {
    const Region * region = find_region(req->region);
    if (!region) {
      res->success = false;
      res->message = "没有区域 " + req->region + "（spot_01..spot_10 / transfer_red / transfer_blue）";
      return;
    }
    std::vector<Point3> points;
    Viewpoint view;
    std::string why;
    rclcpp::Time stamp;
    if (!gather(req->frames > 0 ? req->frames : max_frames_, &points, &view, &stamp, &why)) {
      res->success = false;
      res->message = why;
      return;
    }
    res->state = to_msg(*region, detect_blocks(points, *region, view, params_), stamp);
    res->success = true;
    res->message = summary(res->state);
    RCLCPP_INFO(get_logger(), "detect %s", res->message.c_str());
    RegionStateArray out;
    out.regions.push_back(res->state);
    regions_pub_->publish(out);
    publish_markers({res->state});
  }

  void on_scan()
  {
    std::vector<Point3> points;
    Viewpoint view;
    std::string why;
    rclcpp::Time stamp;
    if (!gather(max_frames_, &points, &view, &stamp, &why)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "定时扫跳过：%s", why.c_str());
      return;
    }
    RegionStateArray out;
    for (const auto & name : scan_names_) {
      const Region * region = find_region(name);
      const auto r = detect_blocks(points, *region, view, params_);
      if (r.visible) {
        out.regions.push_back(to_msg(*region, r, stamp));
      }
    }
    regions_pub_->publish(out);
    publish_markers(out.regions);
  }

  void publish_markers(const std::vector<RegionState> & states)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    for (const auto & s : states) {
      for (const auto & b : s.blocks) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = now();
        m.ns = s.region;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        const double size = b.type == b.SKY ? params_.sky_size : params_.ground_size;
        m.pose.position.x = b.x;
        m.pose.position.y = b.y;
        m.pose.position.z = b.z_bottom + size / 2;
        m.pose.orientation.z = std::sin(b.yaw / 2);
        m.pose.orientation.w = std::cos(b.yaw / 2);
        m.scale.x = m.scale.y = m.scale.z = size;
        m.color.a = b.center_reliable ? 0.7f : 0.35f;
        m.color.r = b.type == b.UNKNOWN ? 1.0f : 0.1f;
        m.color.g = b.type == b.GROUND ? 0.8f : 0.2f;
        m.color.b = b.type == b.SKY ? 1.0f : 0.1f;
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);
      }
      const Region * region = find_region(s.region);
      visualization_msgs::msg::Marker t;
      t.header.frame_id = map_frame_;
      t.header.stamp = now();
      t.ns = s.region + "_text";
      t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.pose.position.x = region->cx;
      t.pose.position.y = region->cy;
      t.pose.position.z = s.surface_z + 1.2;
      t.scale.z = 0.12;
      t.color.r = t.color.g = t.color.b = t.color.a = 1.0f;
      t.text = s.region + " " + std::to_string(s.ground_layers) + (s.has_sky ? "+S" : "") +
        (s.top_maybe_clipped ? "?" : "");
      t.lifetime = rclcpp::Duration::from_seconds(3.0);
      arr.markers.push_back(t);
    }
    markers_pub_->publish(arr);
  }

  std::string map_frame_;
  std::string sensor_frame_;
  size_t max_frames_{5};
  double min_range_{0.3};
  double view_max_elevation_{0.38};
  DetectorParams params_;
  std::vector<Region> regions_;
  std::vector<std::string> scan_names_;

  std::mutex mutex_;
  std::deque<Frame> frames_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<RegionStateArray>::SharedPtr regions_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<DetectBlocks>::SharedPtr detect_srv_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

}  // namespace rc2027_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc2027_perception::BlockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
