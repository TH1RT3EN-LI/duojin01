// SPDX-License-Identifier: Apache-2.0
// Offline protocol and ROS-output tests; no physical device is opened.
#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <thread>
#include "lslidar_driver/lslidar_x10_driver.hpp"

using Packet = lslidar_msgs::msg::LslidarPacket;
using lslidar_driver::LslidarX10Driver;

namespace {
Packet::UniquePtr makePacket(int start_angle, int end_angle, int background_distance = 1000) {
  auto packet = Packet::UniquePtr(new Packet());
  packet->data.fill(0);
  packet->data[0] = 0xA5;
  packet->data[1] = 0x5A;
  packet->data[2] = 108;
  const auto write16 = [&packet](int index, int value) {
    packet->data[index] = value >> 8;
    packet->data[index + 1] = value & 0xFF;
  };
  write16(5, start_angle % 36000);
  write16(105, end_angle % 36000);
  const int interval = (end_angle - start_angle + 36000) % 36000;
  for (int group = 0; group < 16; ++group) {
    const int angle = (start_angle + interval * group / 15) % 36000;
    const int distance = angle >= 8900 && angle <= 9100 ? 2300 : background_distance;
    write16(7 + group * 6, distance);
    packet->data[9 + group * 6] = 41;
    write16(10 + group * 6, 3200);
    packet->data[12 + group * 6] = 72;
  }
  uint8_t checksum = 0;
  for (size_t i = 0; i < 107; ++i) checksum += packet->data[i];
  packet->data[107] = checksum;
  return packet;
}

class PacketSource : public lslidar_driver::DataAcquisitionStrategy {
 public:
  std::vector<Packet::UniquePtr> packets;
  size_t next = 0;
  int getPacket(Packet::UniquePtr &packet) override {
    if (next >= packets.size()) return -1;
    packet = std::move(packets[next++]);
    return 108;
  }
};

class OfflineX10Driver : public LslidarX10Driver {
 public:
  using LslidarX10Driver::LslidarX10Driver;
  int io_created = 0;
  bool createRosIO() override {
    ++io_created;
    pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
    laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(san_topic_, 10);
    time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);
    if (publish_multiecholaserscan) {
      multiecho_scan_pub_ = node_->create_publisher<sensor_msgs::msg::MultiEchoLaserScan>("multiecho_scan", 10);
    }
    return true;
  }
};

class X10Test : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void createDriver(const std::vector<rclcpp::Parameter> &overrides = {}) {
    driver.reset();
    node.reset();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", HARDWARE_LIDAR_CONFIG});
    options.parameter_overrides(overrides);
    node = std::make_shared<rclcpp::Node>("lslidar_driver_node", options);
    driver.reset(new OfflineX10Driver(node));
  }
  void SetUp() override {
    createDriver();
    ASSERT_TRUE(driver->initialize());
  }
  void TearDown() override {
    driver.reset();
    node.reset();
  }
  sensor_msgs::msg::PointCloud2 cloud(const std::vector<pcl::PointXYZI> &points) {
    pcl::PointCloud<pcl::PointXYZI> source;
    source.points.assign(points.begin(), points.end());
    source.width = source.points.size();
    source.height = 1;
    source.header.frame_id = "laser";
    source.header.stamp = 123400000;
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(source, output);
    return output;
  }
  pcl::PointXYZI point(float x, float y, float intensity) {
    pcl::PointXYZI p;
    p.x = x; p.y = y; p.z = 0.0f; p.intensity = intensity;
    return p;
  }
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<OfflineX10Driver> driver;
};

TEST_F(X10Test, HardwareConfigurationMatchesLegacyPacketLayout) {
  EXPECT_EQ(driver->packet_length, 108);
  EXPECT_EQ(driver->packet_points_max, 16);
  EXPECT_EQ(driver->angle_bits_start, 5);
  EXPECT_EQ(driver->data_bits_start, 7);
  EXPECT_EQ(driver->end_angle_bits_start, 105);
  EXPECT_EQ(static_cast<int>(driver->baud_rate), 460800);
  EXPECT_EQ(driver->points_size, 540);
  EXPECT_TRUE(driver->invert_azimuth);
  EXPECT_EQ(driver->serial_port_, "/dev/lslidar");
  EXPECT_EQ(driver->san_topic_, "/scan");
  EXPECT_EQ(node->get_parameter("frame_id").as_string(), "laser");
  EXPECT_FALSE(node->get_parameter("use_sim_time").as_bool());
}

TEST_F(X10Test, DecodesBothEchoesAcrossZeroAngle) {
  auto packet = makePacket(35000, 500);
  ASSERT_TRUE(driver->checkPacketValidity(packet, 108));
  driver->decodePacket(packet);
  ASSERT_EQ(driver->points.size(), 32u);
  EXPECT_EQ(driver->points.front().azimuth, 35000);
  EXPECT_EQ(driver->points.back().azimuth, 500);
  EXPECT_NEAR(driver->points[0].distance, 1.0f, 1e-6);
  EXPECT_NEAR(driver->points[1].distance, 3.2f, 1e-6);
  EXPECT_FLOAT_EQ(driver->points[0].intensity, 41.0f);
  EXPECT_FLOAT_EQ(driver->points[1].intensity, 72.0f);
}

TEST_F(X10Test, RejectsCorruptedAndTruncatedPackets) {
  auto packet = makePacket(0, 1500);
  EXPECT_TRUE(driver->checkPacketValidity(packet, 108));
  EXPECT_TRUE(driver->checkPacketValidity(packet, 1206));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 0));
  EXPECT_FALSE(driver->checkPacketValidity(packet, -1));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 107));
  EXPECT_FALSE(driver->checkPacketValidity(packet, 1207));
  ++packet->data[8];
  EXPECT_FALSE(driver->checkPacketValidity(packet, 108));
}

TEST_F(X10Test, KeepsNearestEchoAndConsistentScanGeometry) {
  auto input = cloud({point(2, 0, 72), point(1, 0, 41), point(-1, 0, 11),
                      point(0.1f, 0, 99), point(101, 0, 99),
                      point(std::numeric_limits<float>::quiet_NaN(), 0, 99),
                      point(std::numeric_limits<float>::infinity(), 0, 99)});
  sensor_msgs::msg::LaserScan scan;
  driver->pointcloudToLaserscan(input, scan);
  ASSERT_EQ(scan.ranges.size(), 540u);
  EXPECT_FLOAT_EQ(scan.ranges[270], 1.0f);
  EXPECT_FLOAT_EQ(scan.intensities[270], 41.0f);
  EXPECT_FLOAT_EQ(scan.ranges[0], 1.0f);
  EXPECT_EQ(scan.header.frame_id, "laser");
  EXPECT_EQ(scan.header.stamp, input.header.stamp);
  EXPECT_NEAR(scan.angle_max, scan.angle_min + scan.angle_increment * 539, 1e-6);
  EXPECT_FLOAT_EQ(scan.time_increment, 0.0f);
}

TEST_F(X10Test, DropsDamagedPacketAndReportsTransportFailure) {
  auto source = std::make_shared<PacketSource>();
  auto damaged = makePacket(0, 1500);
  ++damaged->data[8];
  source->packets.push_back(std::move(damaged));
  source->packets.push_back(makePacket(0, 1500));
  driver->data_acquisition_strategy_ = source;
  EXPECT_TRUE(driver->poll());
  EXPECT_TRUE(driver->points.empty());
  EXPECT_TRUE(driver->poll());
  EXPECT_EQ(driver->points.size(), 32u);
  EXPECT_FALSE(driver->poll());
}

TEST_F(X10Test, HighPrecisionModeHasExactlyTenTimesAsManyBins) {
  driver->use_high_precision = true;
  sensor_msgs::msg::LaserScan scan;
  driver->pointcloudToLaserscan(cloud({point(1, 0, 41)}), scan);
  ASSERT_EQ(scan.ranges.size(), 5400u);
  EXPECT_FLOAT_EQ(scan.ranges[2700], 1.0f);
  EXPECT_NEAR(scan.angle_max, scan.angle_min + scan.angle_increment * 5399, 1e-6);
}

TEST_F(X10Test, RejectsInvalidFrequencyBeforeOpeningTransport) {
  createDriver({rclcpp::Parameter("N10Plus_hz", 0)});
  EXPECT_FALSE(driver->initialize());
  EXPECT_EQ(driver->io_created, 0);
}

TEST_F(X10Test, RejectsMalformedTransformBeforeOpeningTransport) {
  createDriver({rclcpp::Parameter("transform_main", std::vector<double>{1, 2})});
  EXPECT_FALSE(driver->initialize());
  EXPECT_EQ(driver->io_created, 0);
}

TEST_F(X10Test, PublishesCompleteFramesWithRobotAngleConvention) {
  std::vector<sensor_msgs::msg::LaserScan> received;
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { received.push_back(*scan); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (int attempt = 0; attempt < 50 && node->count_subscribers("/scan") == 0; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto source = std::make_shared<PacketSource>();
  for (int turn = 0; turn < 4; ++turn) {
    for (int group = 0; group < 24; ++group) {
      source->packets.push_back(makePacket(group * 1500, (group + 1) * 1500, 1000 + turn * 100));
    }
  }
  driver->data_acquisition_strategy_ = source;
  for (int packet = 0; packet < 96; ++packet) ASSERT_TRUE(driver->poll());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received.size() < 3 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(received.size(), 3u);
  for (size_t i = 0; i < received.size(); ++i) {
    const auto &scan = received[i];
    ASSERT_EQ(scan.ranges.size(), 540u);
    EXPECT_EQ(scan.header.frame_id, "laser");
    bool has_clockwise_target = false;
    for (int index = 134; index <= 136; ++index) {
      has_clockwise_target |= std::abs(scan.ranges[index] - 2.3f) < 1e-5;
    }
    EXPECT_TRUE(has_clockwise_target);
    // The initial partial revolution is discarded before the first zero crossing.
    EXPECT_NEAR(scan.ranges[405], 1.1f + i * 0.1f, 1e-5);
    if (i > 0) {
      EXPECT_GT(rclcpp::Time(scan.header.stamp).nanoseconds(),
                rclcpp::Time(received[i - 1].header.stamp).nanoseconds());
      EXPECT_GT(scan.scan_time, 0.0f);
    }
  }
}

TEST_F(X10Test, ReassemblesFragmentedSerialPacketAndReportsDisconnect) {
  int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  auto expected = makePacket(0, 1500);
  lslidar_driver::LSIOSR serial(ptsname(master), lslidar_driver::BaudRate::BAUD_460800);
  std::thread writer([master, &expected]() {
    const uint8_t noise = 0x42;
    if (write(master, &noise, 1) != 1) return;
    for (size_t offset = 0; offset < 108;) {
      const size_t chunk = std::min<size_t>(7, 108 - offset);
      const ssize_t written = write(master, expected->data.data() + offset, chunk);
      if (written <= 0) return;
      offset += written;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  auto packet = Packet::UniquePtr(new Packet());
  const int noise_result = serial.getSerialData(packet, 108, "N10Plus");
  const int packet_size = serial.getSerialData(packet, 108, "N10Plus");
  writer.join();
  EXPECT_EQ(noise_result, 0);
  ASSERT_EQ(packet_size, 108);
  EXPECT_EQ(packet->data, expected->data);
  EXPECT_TRUE(driver->checkPacketValidity(packet, packet_size));
  close(master);
  EXPECT_EQ(serial.getSerialData(packet, 108, "N10Plus"), -1);
}
}  // namespace
