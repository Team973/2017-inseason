/*
 * FlywheelGains.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_FLYWHEELGAINS_H_
#define SRC_CONTROLLERS_FLYWHEELGAINS_H_

#include "lib/StateSpaceGains.h"

namespace frc973 {

double AAA[] = {1.0, 0.0, 3.508416, 0.957147};
double BBB[] = {1.0, 0.0};
double CCC[] = {0.0, 1.0};
double DDD[] = {0.0};
double LLL[] = {0.014251, 0.407147};
double KKK[] {0.937147, 0.046699};
double UUUMAX[] = {10.000000};
double UUUMIN[] = {-2.000000};

class FlywheelGains {
public:
	static StateSpaceGains *MakeGains() {
		return new StateSpaceGains(
				AAA, 4, BBB, 2, CCC, 2, DDD, 1,
				LLL, 2, KKK, 2, UUUMAX, 1, UUUMIN, 1);
	}
};

}

#endif /* SRC_CONTROLLERS_FLYWHEELGAINS_H_ */
