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

/**
 * assuming constant timestamp,
 * assert the integral of velocity is about equal to distance at every step
 */
void assert_velocity_integral(vector<Waypoint>& points,
        const ProfileParams& params) {
    const double timestep = points[1].time - points[0].time;
    double dist_integral = 0.0;
    double angle_integral = 0.0;
    Waypoint& prev = points[0];

    if (g_debugPrint) {
        printf("timestep=%lf points.size() = %ld\n", timestep, points.size());
    }
    params.print();
    for (auto& point : points) {
        /*
        printf("point: t=%f d=%f d_hat=%f a=%f a_hat=%f done %d\n", point.time,
                point.linear_dist, point.linear_vel,
                point.angular_dist, point.angular_vel,
                point.done);
                */
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

/*
def assert_vel_integral(points, params):
    """
    Assume constant timestep
    Assert that the integral of velocity is about equal to the distance at every step
    """
    timestep = points[1].time - points[0].time
    dist_integral = 0.0
    angle_integral = 0.0
    prev = points[0]
    for point in points:
        #print(point)
        dist_integral += (point.linear_velocity + prev.linear_velocity) / 2.0 * timestep
        angle_integral += (point.angular_velocity + prev.angular_velocity) / 2.0 * timestep
        
        tolerance = 2.0 * timestep * params.max_velocity + 0.001
        assert_near(angle_integral, point.angular_dist, tolerance)
        assert_near(dist_integral, point.linear_dist, tolerance)
        prev = point

def assert_dist_range(points, params):
    """
    Assert that the beginning and end of the profile are as expected (does it get there ever?)
    """
    assert points[0].linear_dist == 0
    assert points[0].angular_dist == 0
    assert points[-1].linear_dist == params.distance
    assert points[-1].angular_dist == params.angle
    
    if params.halt:
        assert points[-1].linear_velocity == 0
        assert points[-1].angular_velocity == 0
    else:
        assert points[-1].linear_velocity != 0
        if params.angle != 0.0:
            assert points[-1].angular_velocity != 0

def assert_v_a_constraints(points, params):
    """
    Assert that at no point velocity exceeds maximum velocity
    and that at no point acceleration exceeds maximum acceleration
    """
    prev = points[0]
    for point in points:
        assert(abs(point.linear_velocity) <= abs(params.max_velocity))
        assert(abs(point.linear_velocity - prev.linear_velocity) <= abs(params.max_acceleration))
        prev = point

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
            points.push_back(curr);
            if (curr.done) {
                break;
            }
            time += timestep;
        }

        assert_velocity_integral(points, params);
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
