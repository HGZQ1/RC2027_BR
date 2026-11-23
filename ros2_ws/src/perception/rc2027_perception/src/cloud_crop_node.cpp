// cloud_crop —— "车体 + 货仓"裁剪框（算法在 crop_box.hpp，有单测）。
//
//   输入：雷达原始点云 livox_ros_driver2/CustomMsg（真车驱动、仿真插件都发这个，Point-LIO 也吃这个）
//   输出：
//     ~/custom（CustomMsg）     裁过的，给 Point-LIO（仿真 launch 把 Point-LIO 的 lid_topic 改到这里）
//     ~/points（PointCloud2）   裁过的，x y z intensity，雷达系，给 block_detector。
//                               真车驱动只能发一种格式（Point-LIO 要 CustomMsg），这一路顺便解决了
//                               "方块检测在真车上没有 PointCloud2"的问题
//
// 盒子在 base_link 系（参数 boxes，扁平数组），开机查一次 TF base_link <- 雷达系（URDF 里的静态变换）。
// 雷达系默认用消息头的 frame_id；真车驱动的 frame_id 是 livox_frame，TF 里没有，要设 lidar_frame:=lidar_link。

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rc2027_perception/crop_box.hpp"

namespace rc2027_perception
{

using livox_ros_driver2::msg::CustomMsg;

class CloudCropNode : public rclcpp::Node
{
public:
  CloudCropNode()
  : Node("cloud_crop")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");
    const auto input = declare_parameter<std::string>("input_topic", "/livox/lidar");
    boxes_ = boxes_from_flat(declare_parameter<std::vector<double>>("boxes", std::vector<double>{}));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // 雷达驱动和仿真插件都是 reliable 发布。best effort 订阅在机器忙时会丢掉整帧，
    // Point-LIO 的里程计时间戳就跟着跳，场地定位的 1 秒看门狗误报「发散」。
    const auto cloud_qos = rclcpp::QoS(10);
    custom_pub_ = create_publisher<CustomMsg>("~/custom", cloud_qos);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", cloud_qos);
    sub_ = create_subscription<CustomMsg>(
      input, cloud_qos,
      std::bind(&CloudCropNode::on_cloud, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "裁剪框：%s → ~/custom + ~/points，%zu 个盒子（%s 系）", input.c_str(),
      boxes_.size(), base_frame_.c_str());
  }

private:
  /// 第一次用的时候查 TF（静态的，查到一次就够）。查不到返回空
  std::optional<CropFilter> & filter(const std::string & frame)
  {
    if (filter_) {
      return filter_;
    }
    try {
      const auto t = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      const auto & v = t.transform.translation;
      const tf2::Matrix3x3 r(tf2::Quaternion(q.x, q.y, q.z, q.w));
      filter_.emplace(boxes_, Transform34{
          r[0][0], r[0][1], r[0][2], v.x,
          r[1][0], r[1][1], r[1][2], v.y,
          r[2][0], r[2][1], r[2][2], v.z});
      RCLCPP_INFO(get_logger(), "%s <- %s 查到了，开始裁", base_frame_.c_str(), frame.c_str());
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "查不到 %s <- %s，这一帧不发: %s", base_frame_.c_str(),
        frame.c_str(), e.what());
    }
    return filter_;
  }

  void on_cloud(const CustomMsg::SharedPtr msg)
  {
    const std::string frame = lidar_frame_.empty() ? msg->header.frame_id : lidar_frame_;
    const auto & f = filter(frame);
    if (!f) {
      return;
    }

    CustomMsg out;
    out.header = msg->header;
    out.header.frame_id = frame;
    out.timebase = msg->timebase;
    out.lidar_id = msg->lidar_id;
    out.rsvd = msg->rsvd;
    out.points.reserve(msg->points.size());
    for (const auto & p : msg->points) {
      if (f->keep(p.x, p.y, p.z)) {
        out.points.push_back(p);
      }
    }
    out.point_num = static_cast<uint32_t>(out.points.size());
    // 裁掉了多少：一眼看出货仓 / 车体到底有没有进点云（仿真里车上的东西默认雷达看不见，见 sim URDF 的 sim_cargo）
    removed_ += msg->points.size() - out.points.size();
    ++frames_;
    if (frames_ >= 50) {
      RCLCPP_INFO(
        get_logger(), "最近 %zu 帧平均每帧裁掉 %.0f 点（剩 %zu）", frames_,
        static_cast<double>(removed_) / static_cast<double>(frames_), out.points.size());
      removed_ = 0;
      frames_ = 0;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = out.header;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(out.points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
    in(cloud, "intensity");
    for (const auto & p : out.points) {
      *x = p.x;
      *y = p.y;
      *z = p.z;
      *in = static_cast<float>(p.reflectivity);
      ++x;
      ++y;
      ++z;
      ++in;
    }

    custom_pub_->publish(out);
    points_pub_->publish(cloud);
  }

  std::string base_frame_;
  std::string lidar_frame_;
  std::vector<AxisBox> boxes_;
  std::optional<CropFilter> filter_;
  size_t removed_{0};
  size_t frames_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
};

}  // namespace rc2027_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc2027_perception::CloudCropNode>());
  rclcpp::shutdown();
  return 0;
}
