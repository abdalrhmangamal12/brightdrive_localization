// Build check: Eigen and GTest are found and linked. Delete once real tests exist.
#include <gtest/gtest.h>
#include <Eigen/Dense>

TEST(Setup, EigenWorks) {
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  EXPECT_TRUE((I * v).isApprox(v));
}
