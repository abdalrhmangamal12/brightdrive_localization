// Checks geo::Projector without GTest; exit 0 when every check passes.
//   ros2 run brightdrive_mapping geo_check
//
// LocalCartesian at (30 N, 31 E, 0): the origin maps to zero, +0.001 deg of
// latitude is about +110.85 m north and +0.001 deg of longitude about
// +96.49 m east (WGS-84 radii of curvature at 30 deg), and mapToLla returns
// the input within 1e-9 deg. SphericalMercator at (0, 0, 0): three fixes of
// the run c bag against x = R lon, y = R ln tan(pi/4 + lat/2), z = alt, the
// fixed formula the backend used before the projection became configuration.
#include <cmath>
#include <cstdio>

#include "brightdrive_mapping/geo.hpp"

namespace {

int failures = 0;

void expect(const char* what, double got, double want, double tol) {
  const bool ok = std::fabs(got - want) <= tol;
  if (!ok) ++failures;
  std::printf("%s  %-52s got %17.9f  want %17.9f  tol %g\n",
              ok ? "ok  " : "FAIL", what, got, want, tol);
}

}  // namespace

int main() {
  using brightdrive_mapping::geo::Projection;
  using brightdrive_mapping::geo::Projector;

  std::printf("LocalCartesian, origin 30 N 31 E 0 m\n");
  const Projector lc(Projection::LocalCartesian, 30.0, 31.0, 0.0);
  const Eigen::Vector3d o = lc.llaToMap(30.0, 31.0, 0.0);
  expect("origin -> x [m]", o.x(), 0.0, 1e-9);
  expect("origin -> y [m]", o.y(), 0.0, 1e-9);
  expect("origin -> z [m]", o.z(), 0.0, 1e-9);
  const Eigen::Vector3d n = lc.llaToMap(30.001, 31.0, 0.0);
  expect("+0.001 deg lat -> y north [m]", n.y(), 110.85, 0.01);
  expect("+0.001 deg lat -> x east [m]", n.x(), 0.0, 1e-6);
  const Eigen::Vector3d e = lc.llaToMap(30.0, 31.001, 0.0);
  expect("+0.001 deg lon -> x east [m]", e.x(), 96.49, 0.01);
  expect("+0.001 deg lon -> y north [m] (curvature only)", e.y(), 0.0, 1e-3);
  const Eigen::Vector3d up = lc.llaToMap(30.0, 31.0, 12.5);
  expect("+12.5 m alt -> z up [m]", up.z(), 12.5, 1e-9);
  const double lat_in = 30.0123456789, lon_in = 31.0987654321, alt_in = 123.456;
  const Eigen::Vector3d back = lc.mapToLla(lc.llaToMap(lat_in, lon_in, alt_in));
  expect("round trip lat [deg]", back.x(), lat_in, 1e-9);
  expect("round trip lon [deg]", back.y(), lon_in, 1e-9);
  expect("round trip alt [m]", back.z(), alt_in, 1e-6);

  std::printf(
      "SphericalMercator, origin 0 0 0: three fixes of the run c bag\n");
  const Projector sm(Projection::SphericalMercator, 0.0, 0.0, 0.0);
  constexpr double kR = 6378137.0;
  constexpr double kDegToRad = M_PI / 180.0;
  const double fixes[3][4] = {
      {534.595, -0.000040864254, 0.000894554199, 1.601782},
      {634.495, 0.000313268269, 0.000079722934, 1.461232},
      {834.495, -0.000210190315, 0.000058250549, 1.492184}};
  char what[64];
  for (const auto& f : fixes) {
    const double lat = f[1] * kDegToRad;
    const double lon = f[2] * kDegToRad;
    const Eigen::Vector3d ref(
        kR * lon, kR * std::log(std::tan(M_PI / 4.0 + lat / 2.0)), f[3]);
    const Eigen::Vector3d got = sm.llaToMap(f[1], f[2], f[3]);
    std::snprintf(what, sizeof(what), "t=%.3f x [m]", f[0]);
    expect(what, got.x(), ref.x(), 1e-6);
    std::snprintf(what, sizeof(what), "t=%.3f y [m]", f[0]);
    expect(what, got.y(), ref.y(), 1e-6);
    std::snprintf(what, sizeof(what), "t=%.3f z [m]", f[0]);
    expect(what, got.z(), ref.z(), 1e-6);
  }
  const Eigen::Vector3d back_sm =
      sm.mapToLla(sm.llaToMap(0.000313268269, 0.000079722934, 1.461232));
  expect("Mercator round trip lat [deg]", back_sm.x(), 0.000313268269, 1e-12);
  expect("Mercator round trip lon [deg]", back_sm.y(), 0.000079722934, 1e-12);

  std::printf("%s\n",
              failures ? "geo_check: FAILED" : "geo_check: all checks passed");
  return failures ? 1 : 0;
}
