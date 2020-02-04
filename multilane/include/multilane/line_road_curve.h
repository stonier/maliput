#pragma once

#include <cmath>
#include <limits>
#include <utility>

#include "maliput/api/lane_data.h"
#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_unused.h"

#include "multilane/road_curve.h"

namespace maliput {
namespace multilane {

/// RoadCurve specification for a reference curve that describes a line.
class LineRoadCurve : public RoadCurve {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LineRoadCurve)

  /// Constructor. Computes a line from @p xy0 as the initial point of the line
  /// and @p dxy as the difference vector that connects the @p xy0 with the end
  /// point of the reference curve.
  /// @param xy0 A 2D vector that represents the first point of the lane.
  /// @param dxy A 2D difference vector between the last point and @p xy0.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. See RoadCurve class constructor for more details.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. See RoadCurve class constructor for more
  /// details.
  /// @param linear_tolerance The linear tolerance, in meters, for all
  /// computations. See RoadCurve class constructor for more details.
  /// @param scale_length The minimum spatial period of variation in the curve,
  /// in meters. See RoadCurve class constructor for more details.
  /// @param computation_policy Policy to guide all computations. If geared
  /// towards speed, computations will make use of analytical expressions even
  /// if not actually correct for the curve as specified.
  /// @throws maliput::common::assertion_error if @p linear_tolerance is not a
  /// positive number.
  /// @throws maliput::common::assertion_error if @p scale_length is not a
  /// positive number.
  explicit LineRoadCurve(const drake::Vector2<double>& xy0, const drake::Vector2<double>& dxy,
                         const CubicPolynomial& elevation, const CubicPolynomial& superelevation,
                         double linear_tolerance, double scale_length, ComputationPolicy computation_policy)
      : RoadCurve(linear_tolerance, scale_length, elevation, superelevation, computation_policy),
        p0_(xy0),
        dp_(dxy),
        heading_(std::atan2(dxy.y(), dxy.x())) {
    MALIPUT_DEMAND(dxy.norm() > kMinimumNorm);
  }

  ~LineRoadCurve() override = default;

  drake::Vector2<double> xy_of_p(double p) const override { return p0_ + p * dp_; }

  drake::Vector2<double> xy_dot_of_p(double p) const override {
    maliput::common::unused(p);
    return dp_;
  }

  double heading_of_p(double p) const override {
    maliput::common::unused(p);
    return heading_;
  }

  double heading_dot_of_p(double p) const override {
    maliput::common::unused(p);
    return 0.;
  }

  double l_max() const override { return dp_.norm(); }

  drake::Vector3<double> ToCurveFrame(const drake::Vector3<double>& geo_coordinate, double r_min, double r_max,
                                      const api::HBounds& height_bounds) const override;

  bool IsValid(double r_min, double r_max, const api::HBounds& height_bounds) const override {
    maliput::common::unused(r_min);
    maliput::common::unused(r_max);
    maliput::common::unused(height_bounds);
    return true;
  }

 private:
  double FastCalcPFromS(double s, double r) const override;

  double FastCalcSFromP(double p, double r) const override;

  double CalcMinimumRadiusAtOffset(double r) const override {
    maliput::common::unused(r);
    return std::numeric_limits<double>::infinity();
  }

  // The first point in world coordinates over the z=0 plane of the reference
  // curve.
  const drake::Vector2<double> p0_{};
  // The difference vector that joins the end point of the reference curve with
  // the first one, p0_.
  const drake::Vector2<double> dp_{};
  // The constant angle deviation of dp_ with respect to the x axis of the world
  // frame.
  const double heading_{};
  // The minimum dp_ norm to avoid having issues when computing heading_.
  static const double kMinimumNorm;
};

}  // namespace multilane
}  // namespace maliput
