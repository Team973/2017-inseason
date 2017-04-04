#include <boost/test/unit_test.hpp>

#include "lib/util/Util.h"

using namespace frc973;

BOOST_AUTO_TEST_CASE(util_magnitude)
{
    BOOST_CHECK(magnitude(3, 4) == 5);
}

BOOST_AUTO_TEST_CASE(util_bound)
{
    BOOST_CHECK(Util::bound(5, 0, 10) == 5);
    BOOST_CHECK(Util::bound(-1, 0, 10) == 0);
    BOOST_CHECK(Util::bound(11, 0, 10) == 10);
}

BOOST_AUTO_TEST_CASE(util_min_max)
{
    BOOST_CHECK(Util::min(5, 2) == 2);

    BOOST_CHECK(Util::max(5, 2) == 5);
}

BOOST_AUTO_TEST_CASE(util_abs)
{
    BOOST_CHECK(Util::abs(-3) == 3);
    BOOST_CHECK(Util::abs(3) == 3);
    BOOST_CHECK(Util::abs(0) == 0);
}

BOOST_AUTO_TEST_CASE(util_antideadband)
{
    BOOST_CHECK(Util::antideadband(5.0, 0.1) == 5.0);
    BOOST_CHECK(Util::antideadband(0.0, 0.1) == 0.1);
    BOOST_CHECK(Util::antideadband(0.05, 0.1) == 0.1);
    BOOST_CHECK(Util::antideadband(-0.05, 0.1) == -0.1);
}

BOOST_AUTO_TEST_CASE(util_sign)
{
    BOOST_CHECK(Util::signedIncrease(5, 1) == 6);
    BOOST_CHECK(Util::signedIncrease(-5, 1) == -6);

    BOOST_CHECK(Util::signSquare(5) == 25);
    BOOST_CHECK(Util::signSquare(-5) == -25);

    BOOST_CHECK(Util::signum(5) == 1);
    BOOST_CHECK(Util::signum(0) == 0);
    BOOST_CHECK(Util::signum(-5) == -1);
}

BOOST_AUTO_TEST_CASE(util_floats)
{
    BOOST_CHECK(Util::close(1.0, 1.0));
    BOOST_CHECK(Util::close(1.0, 1.0000001));
    BOOST_CHECK(!Util::close(1.0, 1.01));
}
