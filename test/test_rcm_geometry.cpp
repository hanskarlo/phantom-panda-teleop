#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

#include "phantom_panda_teleop/rcm_geometry.hpp"

using phantom_panda_teleop::SphericalRcmState;
using phantom_panda_teleop::blendShaftDirectionNearApex;
using phantom_panda_teleop::cartesianTipTargetWithRcm;
using phantom_panda_teleop::flangeTargetFromSpherical;
using phantom_panda_teleop::pointToShaftLineDistance;
using phantom_panda_teleop::rcmFromInsertedToolTip;
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

TEST(RcmGeometry, FeasibleCartesianTipTargetIsPreserved)
{
  const Eigen::Vector3d rcm(0.4, 0.0, 0.2);
  const Eigen::Vector3d desired_tip(0.38, 0.01, 0.12);
  const auto target = cartesianTipTargetWithRcm(
    desired_tip, rcm, Eigen::Vector3d::UnitZ(), 0.25, 0.0, 0.23, 0.52);

  EXPECT_TRUE(target.tip_position.isApprox(desired_tip, 1e-12));
  EXPECT_TRUE(
    (target.flange_position + 0.25 * (-target.shaft_axis)).isApprox(
      target.tip_position, 1e-12));
  EXPECT_NEAR(
    pointToShaftLineDistance(rcm, target.flange_position, -target.shaft_axis),
    0.0, 1e-12);
}

TEST(RcmGeometry, CartesianTipTargetProjectsOntoTiltCone)
{
  const Eigen::Vector3d rcm = Eigen::Vector3d::Zero();
  const auto target = cartesianTipTargetWithRcm(
    Eigen::Vector3d(-0.10, 0.0, -0.01), rcm, Eigen::Vector3d::UnitZ(),
    0.25, 0.0, 0.23, 0.52);

  const double tilt = std::acos(std::clamp(target.shaft_axis.z(), -1.0, 1.0));
  EXPECT_NEAR(tilt, 0.52, 1e-12);
  EXPECT_NEAR(
    pointToShaftLineDistance(rcm, target.flange_position, -target.shaft_axis),
    0.0, 1e-12);
}

TEST(RcmGeometry, CartesianTargetAboveTrocarProjectsToApexWithoutAxisFlip)
{
  const Eigen::Vector3d rcm(0.4, 0.0, 0.2);
  const Eigen::Vector3d fallback =
    phantom_panda_teleop::shaftAxisFromAngles(0.3, 0.2);
  const auto target = cartesianTipTargetWithRcm(
    rcm + Eigen::Vector3d(0.0, 0.0, 0.05), rcm, fallback,
    0.25, 0.0, 0.23, 0.52);

  EXPECT_NEAR(target.insertion, 0.0, 1e-12);
  EXPECT_TRUE(target.tip_position.isApprox(rcm, 1e-12));
  EXPECT_TRUE(target.shaft_axis.isApprox(fallback.normalized(), 1e-12));
}

TEST(RcmGeometry, CartesianTipTargetClampsMaximumInsertion)
{
  const Eigen::Vector3d rcm = Eigen::Vector3d::Zero();
  const auto target = cartesianTipTargetWithRcm(
    Eigen::Vector3d(0.0, 0.0, -1.0), rcm, Eigen::Vector3d::UnitZ(),
    0.25, 0.0, 0.23, 0.52);

  EXPECT_NEAR(target.insertion, 0.23, 1e-12);
  EXPECT_TRUE(target.tip_position.isApprox(Eigen::Vector3d(0.0, 0.0, -0.23), 1e-12));
  EXPECT_TRUE(target.flange_position.isApprox(Eigen::Vector3d(0.0, 0.0, 0.02), 1e-12));
}

TEST(RcmGeometry, ShaftDirectionTransitionsContinuouslyFromApex)
{
  const Eigen::Vector3d start = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d target =
    phantom_panda_teleop::shaftAxisFromAngles(0.4, 0.5);

  const Eigen::Vector3d at_apex = blendShaftDirectionNearApex(start, target, 0.0, 0.02);
  const Eigen::Vector3d halfway = blendShaftDirectionNearApex(start, target, 0.01, 0.02);
  const Eigen::Vector3d after_transition =
    blendShaftDirectionNearApex(start, target, 0.03, 0.02);

  EXPECT_TRUE(at_apex.isApprox(start, 1e-12));
  EXPECT_GT(halfway.dot(start), target.dot(start));
  EXPECT_GT(halfway.dot(target), start.dot(target));
  EXPECT_TRUE(after_transition.isApprox(target, 1e-12));
}

TEST(RcmGeometry, InsertedTipMarkerReconstructsRcmAndInsertion)
{
  const Eigen::Vector3d rcm(0.45, -0.08, 0.22);
  const Eigen::Vector3d shaft_axis =
    phantom_panda_teleop::shaftAxisFromAngles(0.4, 0.25);
  const Eigen::Vector3d flange_to_tip_axis = -shaft_axis;
  constexpr double tool_length = 0.25;
  constexpr double insertion = 0.08;
  const Eigen::Vector3d flange = rcm + (tool_length - insertion) * shaft_axis;
  const Eigen::Vector3d tip = flange + tool_length * flange_to_tip_axis;

  const Eigen::Vector3d reconstructed_rcm =
    rcmFromInsertedToolTip(tip, flange_to_tip_axis, insertion);
  const SphericalRcmState state = sphericalStateFromFlange(
    flange, reconstructed_rcm, tool_length);

  EXPECT_TRUE(reconstructed_rcm.isApprox(rcm, 1e-12));
  EXPECT_NEAR(state.insertion, insertion, 1e-12);
}
