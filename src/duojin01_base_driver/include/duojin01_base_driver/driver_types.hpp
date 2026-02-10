#pragma once

#include <cstdint>
#include <array>
#include <cmath>

// ---------------------------
// Protocol / constants
// ---------------------------
namespace duojin01
{
constexpr uint8_t SEND_DATA_CHECK   = 1;          // send checksum mode
constexpr uint8_t READ_DATA_CHECK   = 0;          // recv checksum mode
constexpr uint8_t FRAME_HEADER      = 0x7B;
constexpr uint8_t FRAME_TAIL        = 0x7D;
constexpr std::size_t RECEIVE_DATA_SIZE = 24;
constexpr std::size_t SEND_DATA_SIZE    = 11;

constexpr float PI = 3.1415926f;

constexpr float MAX_DELTA_T = 1.0f; // max value of deltat in on_time

// IMU unit conversion ratios 
constexpr float GYROSCOPE_RATIO = 0.00026644f;  // rad/s per raw unit
constexpr float ACCEl_RATIO     = 1671.84f;     // m/s^2 per raw unit

// ---------------------------
// Covariance matrices 
// ---------------------------
inline constexpr std::array<double, 36> odom_pose_covariance = {
  1e-3, 0,   0,   0,   0,   0,
  0,   1e-3, 0,   0,   0,   0,
  0,   0,   1e6,  0,   0,   0,
  0,   0,   0,   1e6,  0,   0,
  0,   0,   0,   0,   1e6,  0,
  0,   0,   0,   0,   0,   1e3
};

inline constexpr std::array<double, 36> odom_pose_covariance2 = {
  1e-9, 0,   0,   0,   0,   0,
  0,   1e-3, 1e-9,0,   0,   0,
  0,   0,   1e6,  0,   0,   0,
  0,   0,   0,   1e6,  0,   0,
  0,   0,   0,   0,   1e6,  0,
  0,   0,   0,   0,   0,   1e-9
};

inline constexpr std::array<double, 36> odom_twist_covariance = {
  1e-3, 0,   0,   0,   0,   0,
  0,   1e-3, 0,   0,   0,   0,
  0,   0,   1e6,  0,   0,   0,
  0,   0,   0,   1e6,  0,   0,
  0,   0,   0,   0,   1e6,  0,
  0,   0,   0,   0,   0,   1e3
};

inline constexpr std::array<double, 36> odom_twist_covariance2 = {
  1e-9, 0,   0,   0,   0,   0,
  0,   1e-3, 1e-9,0,   0,   0,
  0,   0,   1e6,  0,   0,   0,
  0,   0,   0,   1e6,  0,   0,
  0,   0,   0,   0,   1e6,  0,
  0,   0,   0,   0,   0,   1e-9
};

// ---------------------------
// Data structures
// ---------------------------
struct VelPosData
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f}; 
};

struct IMUData
{
  int16_t accele_x_data{0};
  int16_t accele_y_data{0};
  int16_t accele_z_data{0};
  int16_t gyros_x_data{0};
  int16_t gyros_y_data{0};
  int16_t gyros_z_data{0};
};

struct SendFrame
{
  std::array<uint8_t, SEND_DATA_SIZE> tx{};
};

struct ReceiveFrame
{
  std::array<uint8_t, RECEIVE_DATA_SIZE> rx{};
  uint8_t flag_stop{0};
  uint8_t frame_header{0};
  uint8_t frame_tail{0};
};

} // namespace duojin01
