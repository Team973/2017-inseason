#include <boost/test/unit_test.hpp>
#include "lib/TrapProfile.h"

using namespace frc973;
using namespace Profiler;

BOOST_AUTO_TEST_CASE(construct_profile)
{
    Waypoint w = TrapProfile<FakeFloat<10>, FakeFloat<5>,
            FakeFloat<5>, FakeFloat<5>,
            false, false>(0.0);
}
