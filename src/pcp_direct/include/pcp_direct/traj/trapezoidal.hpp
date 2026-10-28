// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// ORIGIN: copied from my own repository ur5e-teaching-controllers (MIT),
// https://github.com/giridhar-shenoy/ur5e-teaching-controllers, file
// src/ur5e_teaching_controllers/src/profiles/trapezoidal.hpp at commit 52fc1ba.
// Only change: the namespace (ur5e_teaching_controllers::profiles -> pcp_direct::profiles).
// The v0.2 brief asks for this generator, unmodified, instead of a new library.
//
// Header-only, synchronised, velocity- and acceleration-limited trapezoidal profiles.
//
// Each segment between two waypoints is rest-to-rest and uses the "1/3 rule": accelerate for
// T/3, cruise for T/3, decelerate for T/3. For a distance D over a duration T this needs a peak
// velocity of 1.5 D / T and an acceleration of 4.5 D / T^2, so the shortest duration that
// respects |v| <= v_max and |a| <= a_max on a joint is
//
//     T_j = max(1.5 |D| / v_max, sqrt(4.5 |D| / a_max)).
//
// All joints of a segment share T = max(T_requested, max_j T_j), so they start and stop
// together and every joint stays inside its own limits. Waypoints are reached at rest (no
// via-point blending); that is a deliberate v0.1 simplification, see docs/controllers.md.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace pcp_direct::profiles
{

/// Position, velocity and acceleration of one joint at one instant.
struct JointSample
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

/// Shortest 1/3-rule duration for one joint moving |distance| under the given limits.
inline double min_duration(double distance, double v_max, double a_max)
{
  if (v_max <= 0.0 || a_max <= 0.0) {
    throw std::invalid_argument("trapezoidal: limits must be positive");
  }
  const double d = std::abs(distance);
  return std::max(1.5 * d / v_max, std::sqrt(4.5 * d / a_max));
}

/// Sample a single rest-to-rest 1/3-rule move of `distance` over `duration` at time t
/// (t is clamped to [0, duration]); returns the offset from the start position.
inline JointSample sample_segment(double distance, double duration, double t)
{
  JointSample s;
  if (duration <= 0.0) {
    s.position = distance;
    return s;
  }
  const double tc = std::clamp(t, 0.0, duration);
  const double ta = duration / 3.0;            // acceleration (and deceleration) time
  const double v = 1.5 * distance / duration;  // cruise velocity
  const double a = v / ta;                     // = 4.5 D / T^2
  if (tc < ta) {
    s.acceleration = a;
    s.velocity = a * tc;
    s.position = 0.5 * a * tc * tc;
  } else if (tc < duration - ta) {
    s.velocity = v;
    s.position = 0.5 * a * ta * ta + v * (tc - ta);
  } else {
    const double td = duration - tc;  // time left
    s.acceleration = (t >= duration) ? 0.0 : -a;
    s.velocity = a * td;
    s.position = distance - 0.5 * a * td * td;
  }
  return s;
}

/// A multi-joint, multi-waypoint trajectory of rest-to-rest trapezoidal segments.
class TrapezoidalTrajectory
{
public:
  TrapezoidalTrajectory() = default;

  /// @param start      joint positions at t = 0
  /// @param waypoints  target positions, one vector per waypoint (same size as start)
  /// @param times      requested time_from_start of each waypoint [s], strictly increasing
  /// @param v_max      per-joint velocity limit [rad/s]
  /// @param a_max      per-joint acceleration limit [rad/s^2]
  /// Segments are stretched (never shortened) so that every limit holds.
  /// @throws std::invalid_argument on size mismatches, decreasing times or non-finite values
  TrapezoidalTrajectory(
    const std::vector<double> & start, const std::vector<std::vector<double>> & waypoints,
    const std::vector<double> & times, const std::vector<double> & v_max,
    const std::vector<double> & a_max)
  {
    const std::size_t n = start.size();
    if (v_max.size() != n || a_max.size() != n) {
      throw std::invalid_argument("trapezoidal: limit vectors must match the joint count");
    }
    if (waypoints.size() != times.size()) {
      throw std::invalid_argument("trapezoidal: one time per waypoint is required");
    }
    const auto finite = [](const std::vector<double> & v) {
      return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
    };
    if (!finite(start) || !finite(times)) {
      throw std::invalid_argument("trapezoidal: start and times must be finite");
    }
    std::vector<double> from = start;
    double previous_request = 0.0;
    double t0 = 0.0;
    for (std::size_t k = 0; k < waypoints.size(); ++k) {
      if (waypoints[k].size() != n) {
        throw std::invalid_argument("trapezoidal: waypoint size must match the joint count");
      }
      if (!finite(waypoints[k])) {
        throw std::invalid_argument("trapezoidal: waypoint positions must be finite");
      }
      const double requested = times[k] - previous_request;
      if (requested < 0.0) {
        throw std::invalid_argument("trapezoidal: waypoint times must be non-decreasing");
      }
      previous_request = times[k];
      Segment seg;
      seg.t0 = t0;
      seg.from = from;
      seg.distance.resize(n);
      seg.duration = requested;
      for (std::size_t j = 0; j < n; ++j) {
        seg.distance[j] = waypoints[k][j] - from[j];
        seg.duration = std::max(seg.duration, min_duration(seg.distance[j], v_max[j], a_max[j]));
      }
      t0 += seg.duration;
      from = waypoints[k];
      segments_.push_back(std::move(seg));
    }
    end_ = from;
    duration_ = t0;
  }

  std::size_t joint_count() const { return end_.size(); }
  /// Position the trajectory starts from (empty if there are no segments).
  const std::vector<double> & start() const
  {
    static const std::vector<double> kEmpty;
    return segments_.empty() ? kEmpty : segments_.front().from;
  }
  std::size_t segment_count() const { return segments_.size(); }
  double duration() const { return duration_; }
  double segment_duration(std::size_t k) const { return segments_.at(k).duration; }
  bool empty() const { return segments_.empty(); }

  /// Sample all joints at time t >= 0 into `out` (resized only if its size differs).
  /// Before 0 the start is held, after duration() the last waypoint is held at rest.
  void sample(double t, std::vector<JointSample> & out) const
  {
    out.resize(end_.size());
    if (segments_.empty()) {
      return;
    }
    if (t >= duration_) {
      for (std::size_t j = 0; j < end_.size(); ++j) {
        out[j] = JointSample{end_[j], 0.0, 0.0};
      }
      return;
    }
    // Segments are few (one per waypoint); a linear scan keeps this allocation-free.
    std::size_t k = 0;
    while (k + 1 < segments_.size() && t >= segments_[k + 1].t0) {
      ++k;
    }
    const Segment & seg = segments_[k];
    for (std::size_t j = 0; j < end_.size(); ++j) {
      JointSample s = sample_segment(seg.distance[j], seg.duration, t - seg.t0);
      s.position += seg.from[j];
      out[j] = s;
    }
  }

private:
  struct Segment
  {
    double t0{0.0};
    double duration{0.0};
    std::vector<double> from;
    std::vector<double> distance;
  };

  std::vector<Segment> segments_;
  std::vector<double> end_;
  double duration_{0.0};
};

}  // namespace pcp_direct::profiles
