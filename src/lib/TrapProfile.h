#pragma once

#include "lib/util/Util.h"
#include "stdio.h"
#include <math.h>

namespace frc973 {

namespace Profiler {

struct Waypoint {
    double time;

    double linear_dist;
    double linear_vel;

    double angular_dist;
    double angular_vel;

    bool done;
    bool error;

    Waypoint(double time_,
            double linear_vel_,
            double linear_dist_,
            double angular_vel_,
            double angular_dist_,
            bool done_,
            bool error_)
         : time(time_)
         , linear_dist(linear_dist_)
         , linear_vel(linear_vel_)
         , angular_dist(angular_dist_)
         , angular_vel(angular_vel_)
         , done(done_)
         , error(error_) {
    }
};

/**
 * C++ doesn't support floating point non-type template arguments so
 * this is a little hack to let us do static asserts on floats
 */
/*template<int N, int D = 1>
struct FakeFloat {
    static constexpr int numerator = N;
    static constexpr int denomenator = D;
    static constexpr double value = static_cast<double>(N) / static_cast<double>(D);
};*/

/**
 * TrapProfileUnsafe does the calculation at runtime like one would expect
 * and is a normal function.  Do not call this function directly, it is
 * dangerous.  Instead, call TrapProfile.
 */
Waypoint TrapProfileUnsafe(double time,
        double distance, double angle,
        double max_velocity, double max_acceleration,
        bool start_halt, bool end_halt);

/**
 * Safely generates a trapazoidal motion profile.  Checks at compile time
 * for profile safety.
 */
template<typename DISTANCE, typename ANGLE,
        typename MAX_VELOCITY, typename MAX_ACCELERATION,
        bool START_HALT, bool END_HALT>
Waypoint TrapProfile(double time) {
    constexpr double distance = DISTANCE::value;
    constexpr double angle = ANGLE::value;
    constexpr double max_velocity = MAX_VELOCITY::value;
    constexpr double max_acceleration = MAX_ACCELERATION::value;
    constexpr bool start_halt = START_HALT;
    constexpr bool end_halt = END_HALT;

    constexpr double dist_ramp = 0.5 * max_velocity *
        max_velocity / max_acceleration * Util::signum(distance);

    static_assert(
            !(!START_HALT && END_HALT) ||
            dist_ramp < Util::abs(DISTANCE::value),
            "Profile is over-constrained");

    return TrapProfileUnsafe(time,
            distance, angle,
            max_velocity, max_acceleration,
            start_halt, end_halt);
}

}

}
