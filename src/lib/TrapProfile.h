#pragma once

#include "lib/util/Util.h"
#include "stdio.h"

namespace frc973 {

namespace Profiler {

struct Waypoint {
    const double time;

    const double linear_dist;
    const double linear_vel;

    const double angular_dist;
    const double angular_vel;

    const bool done;

    Waypoint(double time_, double linear_dist_, double linear_vel_,
            double angular_dist_, double angular_vel_, bool done_)
         : time(time_)
         , linear_dist(linear_dist_)
         , linear_vel(linear_vel_)
         , angular_dist(angular_dist_)
         , angular_vel(angular_vel_)
         , done(done_) {
    }
};

/**
 * C++ doesn't support floating point non-type template arguments so
 * this is a little hack to let us do static asserts on floats
 */
template<int N, int D = 1>
struct FakeFloat {
    static constexpr int numerator = N;
    static constexpr int denomenator = D;
    static constexpr double value = static_cast<double>(N) / static_cast<double>(D);
};

template<typename DISTANCE, typename ANGLE,
        typename MAX_VELOCITY, typename MAX_ACCELERATION,
        bool END_HALT, bool START_HALT>
Waypoint TrapProfile(double time) {
    static constexpr double distance = DISTANCE::value;
    static constexpr double angle = ANGLE::value;
    static constexpr double max_velocity = MAX_VELOCITY::value;
    static constexpr double max_acceleration = MAX_ACCELERATION::value;
    static constexpr bool end_halt = END_HALT;
    static constexpr bool start_halt = START_HALT;

    static_assert(
            !(!START_HALT && END_HALT) ||
            0.5 * MAX_VELOCITY::value * MAX_VELOCITY::value /
            MAX_ACCELERATION::value < Util::abs(DISTANCE::value),
            "Profile is over-constrained");

    return Waypoint(time, 0.0, 0.0, 0.0, 0.0, false);
};

}

}
