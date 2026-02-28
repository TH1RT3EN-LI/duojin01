#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/save_map.hpp"

namespace
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

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
    output_dir_ = this->declare_parameter<std::string>("output_dir", "");
    wait_timeout_sec_ = this->declare_parameter<double>("wait_timeout", 30.0);

    if (output_dir_.empty()) {
      throw std::runtime_error("output_dir parameter must not be empty");
    }

    map_name_ =
      (map_name_arg.empty() || map_name_arg == "auto") ?
      next_numeric_map_name(output_dir_) :
      map_name_arg;

    fs::create_directories(output_dir_);
    prefix_ = (fs::path(output_dir_) / map_name_).string();
    client_ = this->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
  }

  int run()
  {
    if (!wait_for_service()) {
      return 1;
    }

    RCLCPP_INFO(this->get_logger(), "[save_map] saving to %s.pgm/.yaml", prefix_.c_str());

    auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    request->name.data = prefix_;
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
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: save_map service returned an invalid future");
      return 1;
    }

    const auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: save_map service returned no response");
      return 1;
    }

    if (response->result != slam_toolbox::srv::SaveMap::Response::RESULT_SUCCESS) {
      if (response->result == slam_toolbox::srv::SaveMap::Response::RESULT_NO_MAP_RECEIEVD) {
        RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: slam_toolbox has not received a map yet");
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[save_map] ERROR: slam_toolbox save_map failed with result %u",
          static_cast<unsigned int>(response->result));
      }
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
        "[save_map] waiting for /slam_toolbox/save_map ... (timeout=%gs)",
        wait_timeout_sec_);
    } else {
      RCLCPP_INFO(this->get_logger(), "[save_map] waiting for /slam_toolbox/save_map ...");
    }

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (client_->wait_for_service(200ms)) {
        return true;
      }

      if (wait_timeout_sec_ > 0.0) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
        if (elapsed.count() >= wait_timeout_sec_) {
          RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: timeout waiting for /slam_toolbox/save_map");
          return false;
        }
      }
    }

    RCLCPP_ERROR(this->get_logger(), "[save_map] ERROR: interrupted while waiting for /slam_toolbox/save_map");
    return false;
  }

  std::string output_dir_;
  std::string map_name_;
  std::string prefix_;
  double wait_timeout_sec_{30.0};
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr client_;
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
