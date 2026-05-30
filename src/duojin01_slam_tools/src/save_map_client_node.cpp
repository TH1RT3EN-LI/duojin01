#include <algorithm>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::uint8_t occupancy_to_pgm_value(
  const int8_t occupancy,
  const double free_thresh,
  const double occupied_thresh)
{
  if (occupancy < 0) {
    return 205;
  }

  const double value = static_cast<double>(occupancy) / 100.0;
  if (value >= occupied_thresh) {
    return 0;
  }
  if (value <= free_thresh) {
    return 254;
  }

  return 205;
}

void write_pgm(
  const fs::path & image_path,
  const nav_msgs::msg::OccupancyGrid & map,
  const double free_thresh,
  const double occupied_thresh)
{
  std::ofstream out(image_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open image for writing: " + image_path.string());
  }

  const auto width = map.info.width;
  const auto height = map.info.height;
  out << "P5\n";
  out << "# CREATOR: duojin01_slam_tools\n";
  out << width << " " << height << "\n255\n";

  for (int y = static_cast<int>(height) - 1; y >= 0; --y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      const auto pixel = occupancy_to_pgm_value(map.data[index], free_thresh, occupied_thresh);
      out.write(reinterpret_cast<const char *>(&pixel), sizeof(pixel));
    }
  }
}

void write_yaml(
  const fs::path & yaml_path,
  const fs::path & image_path,
  const nav_msgs::msg::OccupancyGrid & map,
  const double free_thresh,
  const double occupied_thresh)
{
  std::ofstream out(yaml_path, std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open yaml for writing: " + yaml_path.string());
  }

  const auto & origin = map.info.origin;
  out << "image: " << image_path.filename().string() << "\n";
  out << "mode: trinary\n";
  out << "resolution: " << map.info.resolution << "\n";
  out << "origin: [" << origin.position.x << ", " << origin.position.y << ", "
      << quaternion_to_yaw(origin.orientation) << "]\n";
  out << "negate: 0\n";
  out << "occupied_thresh: " << occupied_thresh << "\n";
  out << "free_thresh: " << free_thresh << "\n";
}

void save_map_files(
  const std::string & prefix,
  const nav_msgs::msg::OccupancyGrid & map,
  const double free_thresh,
  const double occupied_thresh)
{
  if (map.info.width == 0 || map.info.height == 0 || map.data.empty()) {
    throw std::runtime_error("received empty map from /slam_toolbox/dynamic_map");
  }

  const fs::path prefix_path(prefix);
  const fs::path image_path = prefix_path;
  const fs::path yaml_path = prefix_path;

  write_pgm(image_path.string() + ".pgm", map, free_thresh, occupied_thresh);
  write_yaml(yaml_path.string() + ".yaml", image_path.string() + ".pgm", map, free_thresh, occupied_thresh);
}

fs::path resolve_workspace_root()
{
  const fs::path package_prefix(ament_index_cpp::get_package_prefix("duojin01_slam_tools"));

  for (fs::path current = package_prefix; !current.empty(); current = current.parent_path()) {
    if (current.filename() == "install") {
      const fs::path workspace_root = current.parent_path();
      if (!workspace_root.empty()) {
        return workspace_root.lexically_normal();
      }
      break;
    }

    if (current == current.root_path()) {
      break;
    }
  }

  throw std::runtime_error(
          "failed to resolve workspace root from package prefix: " + package_prefix.string());
}

std::string resolve_output_dir(const std::string & output_dir)
{
  const fs::path dir = output_dir.empty() ? fs::path("maps") : fs::path(output_dir);
  if (dir.is_absolute()) {
    return dir.lexically_normal().string();
  }

  return (resolve_workspace_root() / dir).lexically_normal().string();
}

std::string next_numeric_map_name(const std::string & output_dir)
{
  int latest = -1;
  const fs::path dir(output_dir);

  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return "0";
  }

  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto & path = entry.path();
    if (path.extension() != ".yaml") {
      continue;
    }

    const std::string stem = path.stem().string();
    if (stem.empty()) {
      continue;
    }

    bool is_numeric = true;
    for (const unsigned char ch : stem) {
      if (!std::isdigit(ch)) {
        is_numeric = false;
        break;
      }
    }

    if (!is_numeric) {
      continue;
    }

    try {
      latest = std::max(latest, std::stoi(stem));
    } catch (const std::exception &) {
      continue;
    }
  }

  return std::to_string(latest + 1);
}

class SaveMapClientNode : public rclcpp::Node
{
public:
  SaveMapClientNode()
  : rclcpp::Node("save_map_client_node")
  {
    const std::string map_name_arg = this->declare_parameter<std::string>("map_name", "auto");
    const std::string output_dir_arg = this->declare_parameter<std::string>("output_dir", "maps");
    wait_timeout_sec_ = this->declare_parameter<double>("wait_timeout", 30.0);
    free_thresh_ = this->declare_parameter<double>("free_thresh", 0.25);
    occupied_thresh_ = this->declare_parameter<double>("occupied_thresh", 0.65);

    output_dir_ = resolve_output_dir(output_dir_arg);

    map_name_ =
      (map_name_arg.empty() || map_name_arg == "auto") ?
      next_numeric_map_name(output_dir_) :
      map_name_arg;

    fs::create_directories(output_dir_);
    prefix_ = (fs::path(output_dir_) / map_name_).string();
    client_ = this->create_client<nav_msgs::srv::GetMap>("/slam_toolbox/dynamic_map");
  }

  int run()
  {
    if (!wait_for_service()) {
      return 1;
    }

    RCLCPP_INFO(this->get_logger(), "[save_map] fetching /slam_toolbox/dynamic_map and saving to %s.pgm/.yaml", prefix_.c_str());

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = client_->async_send_request(request);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());

    while (rclcpp::ok()) {
      const auto result = executor->spin_until_future_complete(future, 100ms);
      if (result == rclcpp::FutureReturnCode::SUCCESS) {
        break;
      }
      if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
        executor->remove_node(this->shared_from_this());
        RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: interrupted before save completed");
        return 1;
      }
    }

    executor->remove_node(this->shared_from_this());

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: interrupted before save completed");
      return 1;
    }

    if (!future.valid()) {
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: dynamic_map service returned an invalid future");
      return 1;
    }

    const auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: dynamic_map service returned no response");
      return 1;
    }

    try {
      save_map_files(prefix_, response->map, free_thresh_, occupied_thresh_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: %s", exception.what());
      return 1;
    }

    RCLCPP_INFO(this->get_logger(), "[save_map] done");
    return 0;
  }

private:
  bool wait_for_service()
  {
    if (wait_timeout_sec_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[save_map] waiting for /slam_toolbox/dynamic_map ... (timeout=%gs)",
        wait_timeout_sec_);
    } else {
      RCLCPP_INFO(this->get_logger(), "[save_map] waiting for /slam_toolbox/dynamic_map ...");
    }

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (client_->wait_for_service(200ms)) {
        return true;
      }

      if (wait_timeout_sec_ > 0.0) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
        if (elapsed.count() >= wait_timeout_sec_) {
          RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: timeout waiting for /slam_toolbox/dynamic_map");
          return false;
        }
      }
    }

    RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: interrupted while waiting for /slam_toolbox/dynamic_map");
    return false;
  }

  std::string output_dir_;
  std::string map_name_;
  std::string prefix_;
  double wait_timeout_sec_{30.0};
  double free_thresh_{0.25};
  double occupied_thresh_{0.65};
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 1;

  try {
    auto node = std::make_shared<SaveMapClientNode>();
    exit_code = node->run();
    node.reset();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("save_map_client_node"),
      "[save_map] ERROR: %s",
      exception.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
