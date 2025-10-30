#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

class KittiWriterNode : public rclcpp::Node {
public:
  KittiWriterNode() : Node("kitti_writer") {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&KittiWriterNode::callback, this, std::placeholders::_1)
    );

    output_dir_ = "/data/kitti/velodyne/";
    fs::create_directories(output_dir_);

    RCLCPP_INFO(get_logger(), "KITTI writer started. Saving to %s", output_dir_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::string fname = output_dir_ + "frame_" + std::to_string(frame_idx_++) + ".bin";
    std::ofstream file(fname, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Can't open %s", fname.c_str());
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float xyz[4] = {*iter_x, *iter_y, *iter_z, 1.0f};  // intensity = 1.0
      file.write(reinterpret_cast<char*>(xyz), sizeof(xyz));
    }

    RCLCPP_INFO(get_logger(), "Saved %s", fname.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::string output_dir_;
  size_t frame_idx_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KittiWriterNode>());
  rclcpp::shutdown();
  return 0;
}
