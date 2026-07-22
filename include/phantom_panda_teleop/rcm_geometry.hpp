#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace phantom_panda_teleop
{

struct SphericalRcmState
{
  double theta;
  double phi;
  double insertion;
};

inline Eigen::Vector3d shaftAxisFromAngles(double theta, double phi)
{
  return Eigen::Vector3d(
    std::sin(phi) * std::cos(theta),
    std::sin(phi) * std::sin(theta),
    std::cos(phi));
}

inline SphericalRcmState sphericalStateFromFlange(
  const Eigen::Vector3d & flange_position,
  const Eigen::Vector3d & rcm_position,
  double tool_length)
{
  const Eigen::Vector3d rcm_to_flange = flange_position - rcm_position;
  const double flange_distance = rcm_to_flange.norm();
  if (!std::isfinite(flange_distance) || flange_distance < 1e-9 || tool_length <= 0.0) {
    throw std::invalid_argument("RCM and flange geometry must be finite and non-degenerate.");
  }

  const Eigen::Vector3d axis = rcm_to_flange / flange_distance;
  return {
    std::atan2(axis.y(), axis.x()),
    std::acos(std::clamp(axis.z(), -1.0, 1.0)),
    tool_length - flange_distance};
}

inline Eigen::Vector3d flangeTargetFromSpherical(
  const Eigen::Vector3d & rcm_position,
  double tool_length,
  const SphericalRcmState & state)
{
  return rcm_position +
         (tool_length - state.insertion) * shaftAxisFromAngles(state.theta, state.phi);
}

inline double pointToShaftLineDistance(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & shaft_origin,
  const Eigen::Vector3d & shaft_axis)
{
  const double axis_norm = shaft_axis.norm();
  if (!std::isfinite(axis_norm) || axis_norm < 1e-9) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Vector3d unit_axis = shaft_axis / axis_norm;
  const Eigen::Vector3d origin_to_point = point - shaft_origin;
  const Eigen::Vector3d perpendicular =
    origin_to_point - origin_to_point.dot(unit_axis) * unit_axis;
  return perpendicular.norm();
}

inline Eigen::Vector3d closestPointOnShaftLine(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & shaft_origin,
  const Eigen::Vector3d & shaft_axis)
{
  const Eigen::Vector3d unit_axis = shaft_axis.normalized();
  return shaft_origin + (point - shaft_origin).dot(unit_axis) * unit_axis;
}

}  // namespace phantom_panda_teleop
