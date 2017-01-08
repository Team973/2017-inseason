/*
 * FlywheelGains.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_FLYWHEELGAINS_REAR_H_
#define SRC_CONTROLLERS_FLYWHEELGAINS_REAR_H_

#include "lib/StateSpaceGains.h"

namespace frc973 {

namespace Back {
	double AAA[] = {1.0, 0.0, 2.790776, 0.964558};
	double BBB[] = {1.0, 0.0};
	double CCC[] = {0.0, 1.0};
	double DDD[] = {0.0};
	double LLL[] = {0.017916, 0.414558};
	double KKK[] {0.944558, 0.061102};
	double UUUMAX[] = {12.000000};
	double UUUMIN[] = {-2.000000};
}

class FlywheelGainsBack {
public:
	static StateSpaceGains *MakeGains() {
		return new StateSpaceGains(
				Back::AAA, 4, Back::BBB, 2, Back::CCC, 2, Back::DDD, 1,
				Back::LLL, 2, Back::KKK, 2, Back::UUUMAX, 1, Back::UUUMIN, 1);
	}
};

}

#endif /* SRC_CONTROLLERS_FLYWHEELGAINS_H_ */
