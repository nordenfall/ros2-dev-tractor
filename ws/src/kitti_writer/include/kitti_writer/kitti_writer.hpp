#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <filesystem>
#include <fstream>

class KittiWriterNode : public rclcpp::Node {
public:
  KittiWriterNode();

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::string output_dir_;
  size_t frame_idx_ = 0;
};
