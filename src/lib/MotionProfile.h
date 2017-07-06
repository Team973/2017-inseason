#pragma once

#include "lib/util/Util.h"
#include "stdio.h"
#include <cmath>

namespace frc973 {

namespace Profiler {

struct NewWaypoint {
    double time;

    double linear_dist;
    double linear_vel;

    double angular_dist;
    double angular_vel;

    bool done;
    bool error;

    NewWaypoint(double time_,
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
template<int N, int D = 1>
struct FakeFloat {
    static constexpr int numerator = N;
    static constexpr int denomenator = D;
    static constexpr double value = static_cast<double>(N) / static_cast<double>(D);
};

/**
 * TrapProfileUnsafe does the calculation at runtime like one would expect
 * and is a normal function.  Do not call this function directly, it is
 * dangerous.  Instead, call TrapProfile.
 */
NewWaypoint TrapezoidProfileUnsafe(double time, double distance, double angle,
                                 double max_velocity, double acceleration,
                                 double start_velocity, double end_velocity);

/**
 * Safely generates a trapazoidal motion profile.  Checks at compile time
 * for profile safety.
 */
template<typename DISTANCE, typename ANGLE,
        typename MAX_VELOCITY, typename ACCELERATION,
        typename START_VELOCITY, typename END_VELOCITY>
NewWaypoint TrapezoidProfile(double time) {
    constexpr double distance = DISTANCE::value;
    constexpr double angle = ANGLE::value;
    constexpr double max_velocity = MAX_VELOCITY::value;
    constexpr double acceleration = ACCELERATION::value;
    constexpr bool start_velocity = START_VELOCITY::value;
    constexpr bool end_velocity = END_VELOCITY::value;

    constexpr double dist_ramp = 0.5 * max_velocity *
        max_velocity / acceleration * Util::signum(distance);

    static_assert(
            acceleration == 0.0 && max_velocity == start_velocity && start_velocity == end_velocity,
             "Velocity must stay constant (ie. accel = 0)");

    return TrapezoidProfileUnsafe(time, distance, angle, max_velocity, acceleration,
                                  start_velocity, end_velocity);
}

NewWaypoint TriProfileUnsafe(double time, double distance, double angle,
                                 double max_velocity, double acceleration,
                                 double start_velocity, double end_velocity);

/**
 * Safely generates a trapazoidal motion profile.  Checks at compile time
 * for profile safety.
 */
template<typename DISTANCE, typename ANGLE,
        typename MAX_VELOCITY, typename ACCELERATION,
        typename START_VELOCITY, typename END_VELOCITY>
NewWaypoint TriProfile(double time) {
    constexpr double distance = DISTANCE::value;
    constexpr double angle = ANGLE::value;
    constexpr double max_velocity = MAX_VELOCITY::value;
    constexpr double acceleration = ACCELERATION::value;
    constexpr double start_velocity = START_VELOCITY::value;
    constexpr double end_velocity = END_VELOCITY::value;

    constexpr double dist_ramp = 0.5 * max_velocity *
        max_velocity / acceleration * Util::signum(distance);
    constexpr double cap_velocity = sqrt(Util::abs(acceleration * distance)
                                        + (Util::square(start_velocity) + Util::square(end_velocity)) / 2.0);
    constexpr double t0 = 0.0;
    constexpr double t_half = (Util::abs(cap_velocity) - Util::abs(start_velocity)) / acceleration;
    constexpr double t1 = Util::abs((Util::abs(end_velocity) - Util::abs(cap_velocity))) / acceleration + t_half;

    static_assert(
            acceleration == 0.0 && max_velocity == start_velocity && start_velocity == end_velocity,
             "Velocity must stay constant (ie. accel = 0)");
    static_assert(Util::abs(cap_velocity) + Util::abs(end_velocity) / 2.0 * t1
                    > Util::abs(distance) + 1.0, "Overconstrained profile");

    return TriProfileUnsafe(time, distance, angle, max_velocity, acceleration, start_velocity, end_velocity);
}

}

}
