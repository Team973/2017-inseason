#define BOOST_TEST_MODULE
#include <boost/test/included/unit_test.hpp>
//#include "Robot.h"
#include "lib/InterpLookupTable.h"

using namespace frc973;

BOOST_AUTO_TEST_CASE(construct_robot)
{
    /*
    Robot* robot = new Robot();
    BOOST_CHECK(robot != nullptr);
    delete robot;
    */
}

BOOST_AUTO_TEST_CASE(interp_lookup_table)
{
    InterpLookupTable lookup;

    lookup.AddPoint(5, 2);
    BOOST_CHECK(lookup.LookupPoint(5) == 2);
    BOOST_CHECK(lookup.LookupPoint(4) == 2);
    BOOST_CHECK(lookup.LookupPoint(6) == 2);

    lookup.AddPoint(6, 3);
    BOOST_CHECK(lookup.LookupPoint(4) == 1);
    BOOST_CHECK(lookup.LookupPoint(5) == 2);
    BOOST_CHECK(lookup.LookupPoint(5.5) == 2.5);
    BOOST_CHECK(lookup.LookupPoint(6) == 3);
    BOOST_CHECK(lookup.LookupPoint(7) == 4);

    lookup.AddPoint(7, 5);
    BOOST_CHECK(lookup.LookupPoint(4) == 1);
    BOOST_CHECK(lookup.LookupPoint(5) == 2);
    BOOST_CHECK(lookup.LookupPoint(5.5) == 2.5);
    BOOST_CHECK(lookup.LookupPoint(6) == 3);
    BOOST_CHECK(lookup.LookupPoint(6.5) == 4);
    BOOST_CHECK(lookup.LookupPoint(7) == 5);
    BOOST_CHECK(lookup.LookupPoint(8) == 7);
}
