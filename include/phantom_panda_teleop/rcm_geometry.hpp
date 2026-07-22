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

struct CartesianRcmTarget
{
  Eigen::Vector3d tip_position;
  Eigen::Vector3d flange_position;
  Eigen::Vector3d shaft_axis;
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

inline Eigen::Vector3d projectIntoRcmCone(
  const Eigen::Vector3d & inward_vector,
  double max_tilt_angle)
{
  if (!inward_vector.allFinite() || !std::isfinite(max_tilt_angle) ||
    max_tilt_angle < 0.0 || max_tilt_angle >= M_PI_2)
  {
    throw std::invalid_argument("RCM cone inputs must be finite and max tilt in [0, pi/2).");
  }

  const double radial = std::hypot(inward_vector.x(), inward_vector.y());
  const double axial = inward_vector.z();
  const double sin_tilt = std::sin(max_tilt_angle);
  const double cos_tilt = std::cos(max_tilt_angle);

  // The desired vector is already inside the solid cone about robot-base +Z.
  if (axial >= 0.0 && radial * cos_tilt <= axial * sin_tilt) {
    return inward_vector;
  }

  // Points behind the cone apex project to the apex instead of flipping the tool.
  const double boundary_length = radial * sin_tilt + axial * cos_tilt;
  if (boundary_length <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  if (radial < 1e-12 || sin_tilt < 1e-12) {
    return Eigen::Vector3d(0.0, 0.0, boundary_length);
  }

  const double projected_radial = boundary_length * sin_tilt;
  const double xy_scale = projected_radial / radial;
  return Eigen::Vector3d(
    inward_vector.x() * xy_scale,
    inward_vector.y() * xy_scale,
    boundary_length * cos_tilt);
}

inline CartesianRcmTarget cartesianTipTargetWithRcm(
  const Eigen::Vector3d & desired_tip_position,
  const Eigen::Vector3d & rcm_position,
  const Eigen::Vector3d & fallback_shaft_axis,
  double tool_length,
  double min_insertion,
  double max_insertion,
  double max_tilt_angle)
{
  if (!desired_tip_position.allFinite() || !rcm_position.allFinite() ||
    !fallback_shaft_axis.allFinite() || fallback_shaft_axis.norm() < 1e-9 ||
    !std::isfinite(tool_length) || !std::isfinite(min_insertion) ||
    !std::isfinite(max_insertion) || tool_length <= 0.0 || min_insertion < 0.0 ||
    max_insertion <= min_insertion || max_insertion >= tool_length)
  {
    throw std::invalid_argument("Cartesian RCM geometry is invalid or degenerate.");
  }

  Eigen::Vector3d fallback = fallback_shaft_axis.normalized();
  fallback = projectIntoRcmCone(fallback, max_tilt_angle);
  if (fallback.norm() < 1e-9) {
    fallback = Eigen::Vector3d::UnitZ();
  } else {
    fallback.normalize();
  }

  // v = p_rcm - p_tip = r*u points inward from the desired tip to the trocar.
  Eigen::Vector3d projected = projectIntoRcmCone(
    rcm_position - desired_tip_position, max_tilt_angle);
  double insertion = projected.norm();
  Eigen::Vector3d shaft_axis = insertion > 1e-9 ? projected / insertion : fallback;

  if (insertion < min_insertion) {
    insertion = min_insertion;
  } else if (insertion > max_insertion) {
    insertion = max_insertion;
  }

  const Eigen::Vector3d tip_position = rcm_position - insertion * shaft_axis;
  const Eigen::Vector3d flange_position =
    rcm_position + (tool_length - insertion) * shaft_axis;
  return {tip_position, flange_position, shaft_axis, insertion};
}

inline Eigen::Vector3d blendShaftDirectionNearApex(
  const Eigen::Vector3d & start_axis,
  const Eigen::Vector3d & target_axis,
  double insertion,
  double transition_depth)
{
  if (!start_axis.allFinite() || !target_axis.allFinite() ||
    start_axis.norm() < 1e-9 || target_axis.norm() < 1e-9 ||
    !std::isfinite(insertion) || insertion < 0.0 ||
    !std::isfinite(transition_depth) || transition_depth <= 0.0)
  {
    throw std::invalid_argument("RCM shaft-direction blend inputs are invalid.");
  }

  const double blend = std::clamp(insertion / transition_depth, 0.0, 1.0);
  const Eigen::Vector3d blended =
    (1.0 - blend) * start_axis.normalized() + blend * target_axis.normalized();
  if (blended.norm() < 1e-9) {
    throw std::invalid_argument("RCM shaft-direction blend is singular.");
  }
  return blended.normalized();
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
