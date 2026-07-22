#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

#include "phantom_panda_teleop/rcm_geometry.hpp"

using phantom_panda_teleop::SphericalRcmState;
using phantom_panda_teleop::flangeTargetFromSpherical;
using phantom_panda_teleop::pointToShaftLineDistance;
using phantom_panda_teleop::shaftAxisFromAngles;
using phantom_panda_teleop::sphericalStateFromFlange;

TEST(RcmGeometry, ReconstructedTargetIntersectsRcm)
{
  const Eigen::Vector3d rcm(0.45, -0.08, 0.22);
  constexpr double tool_length = 0.25;

  for (double theta = -M_PI; theta <= M_PI; theta += 0.2) {
    for (double phi = 0.0; phi <= 0.52; phi += 0.05) {
      for (double insertion = 0.0; insertion <= 0.23; insertion += 0.02) {
        const SphericalRcmState state{theta, phi, insertion};
        const Eigen::Vector3d flange = flangeTargetFromSpherical(rcm, tool_length, state);
        const Eigen::Vector3d flange_to_rcm = -shaftAxisFromAngles(theta, phi);
        EXPECT_NEAR(
          pointToShaftLineDistance(rcm, flange, flange_to_rcm), 0.0, 1e-12);
      }
    }
  }
}

TEST(RcmGeometry, SphericalRoundTripPreservesFlange)
{
  const Eigen::Vector3d rcm(0.4, 0.0, 0.2);
  constexpr double tool_length = 0.25;
  const SphericalRcmState requested{0.7, 0.35, 0.08};
  const Eigen::Vector3d flange = flangeTargetFromSpherical(rcm, tool_length, requested);
  const SphericalRcmState recovered = sphericalStateFromFlange(flange, rcm, tool_length);
  const Eigen::Vector3d reconstructed =
    flangeTargetFromSpherical(rcm, tool_length, recovered);

  EXPECT_TRUE(flange.isApprox(reconstructed, 1e-12));
  EXPECT_NEAR(recovered.insertion, requested.insertion, 1e-12);
}

TEST(RcmGeometry, PointToLineDistanceMeasuresLateralMiss)
{
  const Eigen::Vector3d origin(0.0, 0.0, 0.0);
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d rcm(0.003, -0.004, 0.2);

  EXPECT_NEAR(pointToShaftLineDistance(rcm, origin, axis), 0.005, 1e-12);
}
