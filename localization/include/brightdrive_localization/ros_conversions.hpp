// ROS message -> loc_core types. Header-only, included by the node and the
// tools only; loc_core itself has no ROS.
#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <stdexcept>
#include <string>

#include "brightdrive_localization/params.hpp"
#include "brightdrive_localization/types.hpp"

namespace brightdrive_localization {

inline double toSeconds(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

inline Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose& m) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(m.orientation.w, m.orientation.x,
                                  m.orientation.y, m.orientation.z)
                   .normalized()
                   .toRotationMatrix();
  T.translation() = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
  return T;
}

namespace detail {

inline const sensor_msgs::msg::PointField* findField(
    const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
  for (const auto& f : msg.fields) {
    if (f.name == name) return &f;
  }
  return nullptr;
}

// Reads one scalar of the given PointField type as double.
inline double readField(const uint8_t* p, uint8_t datatype) {
  using PF = sensor_msgs::msg::PointField;
  switch (datatype) {
    case PF::FLOAT32: {
      float v;
      std::memcpy(&v, p, 4);
      return v;
    }
    case PF::FLOAT64: {
      double v;
      std::memcpy(&v, p, 8);
      return v;
    }
    case PF::UINT32: {
      uint32_t v;
      std::memcpy(&v, p, 4);
      return v;
    }
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

inline bool timeTypeSupported(uint8_t datatype) {
  using PF = sensor_msgs::msg::PointField;
  return datatype == PF::FLOAT32 || datatype == PF::FLOAT64 ||
         datatype == PF::UINT32;
}

}  // namespace detail

// PointCloud2 -> Scan with the points.* rules (params.hpp): per-point time
// t_i = stamp + scale * field (stamp_is start) or stamp - scale * (max(field)
// - field) (stamp_is end). Throws std::runtime_error when x/y/z or the time
// field are missing or the time field has a type other than float32,
// float64 or uint32; the message names the field. Non-finite points are
// dropped.
inline Scan scanFromMsg(const sensor_msgs::msg::PointCloud2& msg,
                        const PointsConfig& cfg) {
  using PF = sensor_msgs::msg::PointField;
  const PF* fx = detail::findField(msg, "x");
  const PF* fy = detail::findField(msg, "y");
  const PF* fz = detail::findField(msg, "z");
  const PF* ft = detail::findField(msg, cfg.time_field);
  if (!fx || !fy || !fz) {
    throw std::runtime_error("PointCloud2 on frame '" + msg.header.frame_id +
                             "' has no x/y/z fields");
  }
  if (!ft) {
    throw std::runtime_error("PointCloud2 has no per-point time field '" +
                             cfg.time_field + "' (points.time_field)");
  }
  if (!detail::timeTypeSupported(ft->datatype)) {
    throw std::runtime_error(
        "PointCloud2 time field '" + cfg.time_field + "' has datatype " +
        std::to_string(ft->datatype) +
        "; supported: float32 (7), float64 (8), uint32 (6)");
  }

  Scan scan;
  scan.t_stamp = toSeconds(msg.header.stamp);
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  scan.points.reserve(n);
  std::vector<double> field;
  field.reserve(n);
  double field_max = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = msg.data.data() + i * msg.point_step;
    const double x = detail::readField(p + fx->offset, fx->datatype);
    const double y = detail::readField(p + fy->offset, fy->datatype);
    const double z = detail::readField(p + fz->offset, fz->datatype);
    const double t = detail::readField(p + ft->offset, ft->datatype);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(t)) {
      continue;
    }
    scan.points.emplace_back(x, y, z, 1.0);
    field.push_back(t);
    field_max = std::max(field_max, t);
  }
  scan.time.resize(field.size());
  const bool start = cfg.stamp_is == "start";
  for (size_t i = 0; i < field.size(); ++i) {
    scan.time[i] = start
                       ? scan.t_stamp + cfg.time_scale * field[i]
                       : scan.t_stamp - cfg.time_scale * (field_max - field[i]);
  }
  return scan;
}

}  // namespace brightdrive_localization
