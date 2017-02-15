#define BOOST_TEST_MODULE
#include <boost/test/included/unit_test.hpp>
#include "Robot.h"
#include "lib/InterpLookupTable.h"

BOOST_AUTO_TEST_CASE(construct_robot)
{
    Robot* robot = new Robot();
    BOOST_CHECK(robot != nullptr);
    delete robot;
}

BOOST_AUTO_TEST_CASE(interp_lookup_table)
{
    InterpLookupTable lookup;

    lookup.AddPoint(5, 2);
    BOOST_CHECK(lookup.LookupPoint(5) == 2);
}
