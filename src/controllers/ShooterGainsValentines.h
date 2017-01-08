/*
 * FlywheelGains.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_FLYWHEELGAINS_VALENTINES_H_
#define SRC_CONTROLLERS_FLYWHEELGAINS_VALENTINES_H_

#include "lib/StateSpaceGains.h"

namespace frc973 {

namespace Valentines {
	static double AAA[] = {1.000000, 0.000000, 5.482128, 0.930196};
	static double BBB[] = {1.0, 0.0};
	static double CCC[] = {0.0, 1.0};
	static double DDD[] = {0.0};
	static double LLL[] = {0.009121, 0.380196};
	static double KKK[] = {0.910196, 0.025622};
	static double UUUMAX[] = {12.000000};
	static double UUUMIN[] = {-2.000000};
}

class FlywheelGainsValentines {
public:
	static StateSpaceGains *MakeGains() {
		return new StateSpaceGains(
				Valentines::AAA, 4, Valentines::BBB, 2, Valentines::CCC, 2, Valentines::DDD, 1,
				Valentines::LLL, 2, Valentines::KKK, 2, Valentines::UUUMAX, 1, Valentines::UUUMIN, 1);
	}
};

}

#endif /* SRC_CONTROLLERS_FLYWHEELGAINS_VALENTINES_H_ */
