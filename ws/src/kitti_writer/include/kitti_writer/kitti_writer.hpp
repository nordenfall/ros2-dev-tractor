#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // doTransform specialization
#include <filesystem>
#include <deque>
#include <fstream>
#include <optional>
#include <mutex>

class KittiWriterNode : public rclcpp::Node {
public:
  KittiWriterNode();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool ensureOutputDir();
  std::optional<geometry_msgs::msg::TransformStamped>
  lookupTransform(const std::string& target, const std::string& source, const rclcpp::Time& stamp);

  // params
  std::string input_topic_;
  std::string output_dir_;
  std::string target_frame_;
  bool write_bin_;
  bool publish_topic_;
  std::string intensity_field_;
  double intensity_scale_;
  int max_keep_bins_; // кольцевой буфер по количеству файлов (если >0)

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // bookkeeping
  std::mutex io_mtx_;
  size_t frame_idx_ = 0;
  std::deque<std::string> recent_bins_; // для авто-очистки

  // helpers
  bool writeKittiBinAndTimestamp(const sensor_msgs::msg::PointCloud2& cloud_in_target,
                                 const rclcpp::Time& stamp);
  static bool hasField(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name, int32_t& offset, int32_t& datatype, int32_t& count);
  float readIntensityAt(const sensor_msgs::msg::PointCloud2& cloud, size_t point_index,
                        const std::string& field_name, double scale);
};
