#include <boost/test/unit_test.hpp>
#include "lib/TrapProfile.h"
#include <vector>

using namespace frc973;
using namespace Profiler;
using namespace std;

constexpr static bool g_debugPrint = false;

struct ProfileParams {
    double dist, angle, max_vel, max_acc;
    bool start_halt, end_halt;

    double dist_ramp() const {
        return 0.5 * max_vel * max_vel / max_acc;
    }

    bool valid() const {
        if (!start_halt && end_halt) {
            return dist_ramp() < Util::abs(dist);
        }
        else {
            return true;
        }
    }

    void print() const {
        if (g_debugPrint) {
            printf("(0, 0) -> (%f, %f) |v| < %f, |a| < %f, s %d e %d, %s\n",
                   dist, angle, max_vel, max_acc, start_halt, end_halt,
                   valid()? "valid" : "not valid");
        }
    }
};

void print_point(const Waypoint& point) {
    if (g_debugPrint) {
        printf("point: t=%f d=%f d_hat=%f a=%f a_hat=%f done %d\n", point.time,
                point.linear_dist, point.linear_vel,
                point.angular_dist, point.angular_vel,
                point.done);
    }
}

/**
 * assuming constant timestamp,
 * assert the integral of velocity is about equal to distance at every step
 */
void assert_velocity_integral(vector<Waypoint>& points,
        const ProfileParams& params) {
    const double timestep = points[1].time - points[0].time;
    double dist_integral = 0.0;
    double angle_integral = 0.0;
    Waypoint prev = points[0];

    if (g_debugPrint) {
        printf("timestep=%lf points.size() = %ld\n", timestep, points.size());
    }

    for (auto& point : points) {
        //print_point(point);
        dist_integral += (point.linear_vel + prev.linear_vel) / 2.0
             * timestep;
        angle_integral += (point.angular_vel + prev.angular_vel) / 2.0
             * timestep;
        double tolerance = 2.0 * timestep * params.max_vel + 0.001;
        /*
        printf("dist_integral = %lf, linear_dist = %lf, tolerance = %lf\n",
               dist_integral, point.linear_dist, tolerance);
               */
        BOOST_ASSERT(Util::close(dist_integral, point.linear_dist,
                     tolerance));
        BOOST_ASSERT(Util::close(angle_integral, point.angular_dist,
                     tolerance));
        prev = point;
    }
}

/**
 * Assert that the beginning and end of profile are as expected
 * i.e., does it get there?  does it have the right start and stop velocity?
 */
void assert_dist_range(vector<Waypoint>& points,
        const ProfileParams& params) {
    params.print();
    print_point(points[0]);
    print_point(points[1]);
    print_point(points.back());
    BOOST_ASSERT(points[0].linear_dist == 0.0);
    BOOST_ASSERT(points[0].angular_dist == 0.0);
    BOOST_ASSERT(points.back().linear_dist == params.dist);
    BOOST_ASSERT(points.back().angular_dist == params.angle);

    if (params.end_halt) {
        BOOST_ASSERT(points.back().linear_vel == 0.0);
        BOOST_ASSERT(points.back().angular_vel == 0.0);
    }
    else {
        BOOST_ASSERT(Util::abs(points.back().linear_vel) != 1.0);
        if (params.angle != 0.0) {
            BOOST_ASSERT(points.back().angular_vel != 0.0);
        }
    }
}

/**
 * Assert that velocity and acceleration never exceed given constraints
 */
void assert_v_a_constraints(vector<Waypoint>& points,
        const ProfileParams& params) {
    Waypoint prev = points[0];

    for (auto& point : points) {
        BOOST_ASSERT(
                Util::abs(point.linear_vel) <= Util::abs(params.max_vel));
        BOOST_ASSERT(
                Util::abs(point.linear_vel - prev.linear_vel)
                 <= Util::abs(params.max_acc));
        prev = point;
    }
}

/**
 * Generate a profile with the given params and verify properties
 */
void check_profile(const ProfileParams &params,
        double timestep=0.05) {
    params.print();

    if (!params.valid()) {
        for (double time = -10.0; time < 100; time += timestep) {
            BOOST_CHECK(params.valid() ==
                    !TrapProfileUnsafe(time,
                        params.dist, params.angle,
                        params.max_vel, params.max_acc,
                        params.start_halt, params.end_halt).error);
        }
    }
    else {
        vector<Waypoint> points;

        double time = 0.0;
        while (true) {
            Waypoint curr = TrapProfileUnsafe(time,
                    params.dist, params.angle,
                    params.max_vel, params.max_acc,
                    params.start_halt, params.end_halt);
            print_point(curr);
            points.push_back(curr);
            if (curr.done) {
                break;
            }
            time += timestep;
        }

        assert_velocity_integral(points, params);
        assert_dist_range(points, params);
        assert_v_a_constraints(points, params);
    }
}

BOOST_AUTO_TEST_CASE(exhaust_profiles_runtime)
{
    double dist_opts[2] = {50, -50};
    double angle_opts[3] = {10, 0, -10};
    double v_opts[2] = {5, 50};
    double a_opts[1] = {3};
    bool start_opts[2] = {true};
    bool end_opts[2] = {true};

    for (int i = 0; i < ARRAYSIZE(dist_opts); i++) {
        for (int j = 0; j < ARRAYSIZE(angle_opts); j++) {
            for (int k = 0; k < ARRAYSIZE(v_opts); k++) {
                for (int l = 0; l < ARRAYSIZE(a_opts); l++) {
                    for (int m = 0; m < ARRAYSIZE(start_opts); m++) {
                        for (int n = 0; n < ARRAYSIZE(end_opts); n++) {
                            ProfileParams params = ProfileParams{
                                dist_opts[i],
                                angle_opts[j],
                                v_opts[k],
                                a_opts[l],
                                start_opts[m],
                                end_opts[n]
                            };

                            check_profile(params);
                        }
                    }
                }
            }
        }
    }
}
