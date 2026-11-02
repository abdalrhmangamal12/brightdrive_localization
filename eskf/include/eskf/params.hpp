// Filter settings, loaded once at startup.
// Noise meaning and units: docs/math.md section 4.
#pragma once
#include <Eigen/Core>
#include <optional>

namespace eskf {

// How sigma_a and sigma_g should be read.
// Density: datasheet noise density, [m/s^2/sqrt(Hz)] and [rad/s/sqrt(Hz)].
// PerSample: std of one reading, [m/s^2] and [rad/s] (what CARLA gives).
enum class NoiseModel { Density, PerSample };

struct Params {
  // No default on purpose: every config must choose. The Eskf constructor
  // rejects a Params where this is still empty.
  std::optional<NoiseModel> noise_model;

  // Left at 0 on purpose: the Eskf constructor rejects sigmas <= 0,
  // so a value missing from the YAML fails at startup instead of silently.

  // White noise on the IMU readings. Units depend on noise_model.
  double sigma_a = 0.0;
  double sigma_g = 0.0;

  // Bias random walk.
  double sigma_ba = 0.0;
  double sigma_bg = 0.0;

  // Gravity in the world frame [m/s^2].
  Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, -9.81);

  // GNSS antenna position relative to the IMU, in IMU axes [m].
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();

  // Chi-square gate on the GNSS innovation (3 DOF, 95 %).
  double gate = 7.81;
};
}  // namespace eskf