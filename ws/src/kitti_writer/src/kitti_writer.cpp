#include "kitti_writer/kitti_writer.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

KittiWriterNode::KittiWriterNode()
: rclcpp::Node("kitti_writer_node")
{
  // параметры (с дефолтами)
  input_topic_     = this->declare_parameter<std::string>("input_topic", "/livox/points");
  output_dir_      = this->declare_parameter<std::string>("output_dir", "/data/kitti/velodyne/");
  target_frame_    = this->declare_parameter<std::string>("target_frame", "base_link");
  write_bin_       = this->declare_parameter<bool>("write_bin", true);
  publish_topic_   = this->declare_parameter<bool>("publish_topic", true);
  intensity_field_ = this->declare_parameter<std::string>("intensity_field", "intensity");
  intensity_scale_ = this->declare_parameter<double>("intensity_scale", 1.0);
  max_keep_bins_   = this->declare_parameter<int>("max_keep_bins", 0); // 0 = без ограничений

  if (!ensureOutputDir()) {
    RCLCPP_FATAL(get_logger(), "Failed to create output directory: %s", output_dir_.c_str());
    throw std::runtime_error("output dir");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS для сенсоров
  rclcpp::SensorDataQoS qos;
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos,
    std::bind(&KittiWriterNode::cloudCallback, this, std::placeholders::_1)
  );

  if (publish_topic_) {
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/kitti/points", qos);
  }

  RCLCPP_INFO(get_logger(), "kitti_writer_node started. Listening: %s, target_frame: %s, output: %s",
              input_topic_.c_str(), target_frame_.c_str(), output_dir_.c_str());
}

bool KittiWriterNode::ensureOutputDir() {
  std::error_code ec;
  fs::create_directories(output_dir_, ec);
  if (ec) return false;
  // timestamps.txt рядом
  return true;
}

std::optional<geometry_msgs::msg::TransformStamped>
KittiWriterNode::lookupTransform(const std::string& target, const std::string& source, const rclcpp::Time& stamp)
{
  try {
    auto T = tf_buffer_->lookupTransform(target, source, stamp, tf2::durationFromSec(0.05));
    return T;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "TF lookup failed %s -> %s: %s", source.c_str(), target.c_str(), ex.what());
    return std::nullopt;
  }
}

bool KittiWriterNode::hasField(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name, int32_t& offset, int32_t& datatype, int32_t& count) {
  for (const auto& f : cloud.fields) {
    if (f.name == name) {
      offset = f.offset;
      datatype = f.datatype;
      count = f.count == 0 ? 1 : f.count;
      return true;
    }
  }
  return false;
}

float KittiWriterNode::readIntensityAt(const sensor_msgs::msg::PointCloud2& cloud, size_t point_index,
                                       const std::string& field_name, double scale)
{
  int32_t offset=0, datatype=0, count=0;
  if (!hasField(cloud, field_name, offset, datatype, count)) {
    return 1.0f; // fallback
  }
  const uint8_t* data = &cloud.data[0] + point_index * cloud.point_step + offset;

  // поддержим самые частые типы
  switch (datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
      float v = *reinterpret_cast<const float*>(data);
      return static_cast<float>(v * scale);
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t v = *reinterpret_cast<const uint16_t*>(data);
      return static_cast<float>(static_cast<double>(v) * scale);
    }
    case sensor_msgs::msg::PointField::UINT8: {
      uint8_t v = *reinterpret_cast<const uint8_t*>(data);
      return static_cast<float>(static_cast<double>(v) * scale);
    }
    default:
      return 1.0f;
  }
}

bool KittiWriterNode::writeKittiBinAndTimestamp(const sensor_msgs::msg::PointCloud2& cloud,
                                                const rclcpp::Time& stamp)
{
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (n == 0) return true;

  // подготовим буфер N×4 float32
  std::vector<float> out; out.reserve(n * 4);

  // находим смещения полей x,y,z
  int32_t off_x=-1, off_y=-1, off_z=-1, dt, cnt;
  if (!hasField(cloud, "x", off_x, dt, cnt) ||
      !hasField(cloud, "y", off_y, dt, cnt) ||
      !hasField(cloud, "z", off_z, dt, cnt)) {
    RCLCPP_WARN(get_logger(), "Cloud has no x/y/z fields");
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    const uint8_t* base = &cloud.data[0] + i * cloud.point_step;
    float x = *reinterpret_cast<const float*>(base + off_x);
    float y = *reinterpret_cast<const float*>(base + off_y);
    float z = *reinterpret_cast<const float*>(base + off_z);
    float intensity = readIntensityAt(cloud, i, intensity_field_, intensity_scale_);
    // KITTI ожидает x forward, y left, z up — предполагаем, что target_frame уже такой.
    out.push_back(x);
    out.push_back(y);
    out.push_back(z);
    out.push_back(intensity);
  }

  // имя файла: по счётчику и времени
  std::ostringstream fname;
  fname << "frame_" << std::setw(6) << std::setfill('0') << frame_idx_++ << ".bin";
  const fs::path bin_path = fs::path(output_dir_) / fname.str();
  const fs::path tmp_path = bin_path.string() + ".tmp";

  {
    std::lock_guard<std::mutex> lk(io_mtx_);
    std::ofstream ofs(tmp_path, std::ios::binary);
    if (!ofs) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", tmp_path.c_str());
      return false;
    }
    ofs.write(reinterpret_cast<const char*>(out.data()), out.size() * sizeof(float));
    ofs.flush();
    ofs.close();

    // атомарно переименуем
    std::error_code ec;
    fs::rename(tmp_path, bin_path, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Rename failed: %s -> %s", tmp_path.c_str(), bin_path.c_str());
      return false;
    }

    // timestamps.txt
    const fs::path ts_path = fs::path(output_dir_) / "timestamps.txt";
    std::ofstream tfs(ts_path, std::ios::app);
    if (tfs) {
      // секунды с точностью до нс (как строка)
      const double tsec = stamp.seconds();
      tfs << std::fixed << std::setprecision(9) << tsec << "\n";
    }

    // кольцевой буфер (если задан лимит)
    if (max_keep_bins_ > 0) {
      recent_bins_.push_back(bin_path.string());
      while (static_cast<int>(recent_bins_.size()) > max_keep_bins_) {
        const auto& old = recent_bins_.front();
        std::error_code ec2;
        fs::remove(old, ec2);
        recent_bins_.pop_front();
      }
    }
  }

  return true;
}

void KittiWriterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Если msg уже в нужном фрейме — можно не трансформировать
  sensor_msgs::msg::PointCloud2 cloud_target;
  if (msg->header.frame_id == target_frame_) {
    cloud_target = *msg;
  } else {
    auto T = lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
    if (!T) {
      return; // подождём валидную трансформацию
    }
    try {
      tf2::doTransform(*msg, cloud_target, T.value());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "doTransform failed: %s", e.what());
      return;
    }
  }

  // Запись KITTI
  if (write_bin_) {
    if (!writeKittiBinAndTimestamp(cloud_target, msg->header.stamp)) {
      RCLCPP_WARN(get_logger(), "Failed to write KITTI frame");
    }
  }

  // Репаблиш (опционально)
  if (publish_topic_ && pub_) {
    pub_->publish(cloud_target);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KittiWriterNode>());
  rclcpp::shutdown();
  return 0;
}
