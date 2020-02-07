#include "maliput/api/lane_data.h"

#include <iostream>

namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const LaneEnd::Which& which_end) {
  return out << (which_end == LaneEnd::kStart ? "start" : "finish");
}

std::ostream& operator<<(std::ostream& out, const Rotation& rotation) {
  return out << "(roll = " << rotation.roll() << ", pitch = " << rotation.pitch() << ", yaw = " << rotation.yaw()
             << ")";
}

std::ostream& operator<<(std::ostream& out, const GeoPosition& geo_position) {
  return out << "(x = " << geo_position.x() << ", y = " << geo_position.y() << ", z = " << geo_position.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position) {
  return out << "(s = " << lane_position.s() << ", r = " << lane_position.r() << ", h = " << lane_position.h() << ")";
}

double GeoPosition::Distance(const GeoPosition& geo_position) const {
  return (this->xyz() - geo_position.xyz()).norm();
}

GeoPosition Rotation::Apply(const GeoPosition& geo_position) const {
  const double sa = std::sin(this->roll());
  const double ca = std::cos(this->roll());
  const double sb = std::sin(this->pitch());
  const double cb = std::cos(this->pitch());
  const double sg = std::sin(this->yaw());
  const double cg = std::cos(this->yaw());
  // clang-format off
  return GeoPosition(
        ((cb*cg) * geo_position.x()) +
        ((-ca*sg + sa*sb*cg) * geo_position.y()) +
        ((sa*sg + ca*sb*cg) * geo_position.z()),

        ((cb*sg) * geo_position.x()) +
        ((ca*cg + sa*sb*sg) * geo_position.y()) +
        ((-sa*cg + ca*sb*sg) * geo_position.z()),

        ((-sb) * geo_position.x()) +
        ((sa*cb) * geo_position.y()) +
        ((ca*cb) * geo_position.z()));
  // clang-format on
}

math::Matrix3 Rotation::matrix() const {
  /// TODO(francocipollone): Move the following implementation to maliput::math::Quaternion.
  const math::Vector4 q{math::Vector4{quaternion_.w(), quaternion_.x(), quaternion_.y(), quaternion_.z()}.normalized()};
  // clang-format off
  return{
    {1-2*(q.z()*q.z() + q.w()*q.w()),   2*(q.y()*q.z() - q.w()*q.x()),   2*(q.y()*q.w() + q.z()*q.x())},
    {  2*(q.y()*q.z() + q.w()*q.x()), 1-2*(q.y()*q.y() + q.w()*q.w()),   2*(q.z()*q.w() - q.y()*q.x())},
    {  2*(q.y()*q.w() - q.z()*q.x()),   2*(q.z()*q.w() + q.y()*q.x()), 1-2*(q.y()*q.y() + q.z()*q.z())}};
  // clang-format on
}

double Rotation::Distance(const Rotation& rot) const {
  // Compute transformed unit vectors of a frame A.
  const GeoPosition as = this->Apply({1., 0., 0.});
  const GeoPosition ar = this->Apply({0., 1., 0.});
  const GeoPosition ah = this->Apply({0., 0., 1.});
  // Compute transformed unit vectors of b frame B.
  const GeoPosition bs = rot.Apply({1., 0., 0.});
  const GeoPosition br = rot.Apply({0., 1., 0.});
  const GeoPosition bh = rot.Apply({0., 0., 1.});
  // Compute angles between pairs of unit vectors.
  const double ds = std::acos(as.xyz().dot(bs.xyz()));
  const double dr = std::acos(ar.xyz().dot(br.xyz()));
  const double dh = std::acos(ah.xyz().dot(bh.xyz()));

  return std::sqrt((ds * ds) + (dr * dr) + (dh * dh));
}

}  // namespace api
}  // namespace maliput
