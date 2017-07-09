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

    NewWaypoint():
      time(0.0),
      linear_dist(0.0),
      linear_vel(0.0),
      angular_dist(0.0),
      angular_vel(0.0),
      done(false),
      error(false){
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

NewWaypoint TriProfileUnsafe(double time, double distance, double angle,
                                 double max_velocity, double acceleration,
                                 double start_velocity, double end_velocity);

/**
 * Safely generates a trapazoidal motion profile.  Checks at compile time
 * for profile safety.
 */

}

}
