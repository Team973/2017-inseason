#define BOOST_TEST_MODULE
#include <boost/test/included/unit_test.hpp>
#include "Robot.h"

BOOST_AUTO_TEST_CASE(construct_robot)
{
    Robot* robot = new Robot();
    BOOST_CHECK(robot != nullptr);
    delete robot;
}
