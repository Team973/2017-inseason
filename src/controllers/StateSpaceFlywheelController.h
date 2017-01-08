#ifndef _CONTROLLER_STATE_SPACE_FLYWHEEL_H
#define _CONTROLLER_STATE_SPACE_FLYWHEEL_H

/**
 * Blatant copy of 254's state space flywheel
 * https://github.com/Team254/FRC-2014/blob/master/src/com/team254/lib/FlywheelController.java
 * #noshame
 */

#include "lib/StateSpaceController.h"

namespace frc973 {

class Debouncer;
class Matrix;

class StateSpaceFlywheelController :
		public StateSpaceController {
public:
	double velGoal;
	bool enabled;
	Matrix *y;
	Matrix *r;
	double prevPos;
	double goal;
	double curVel;
	double outputVoltage;
	double targetError;
	Debouncer *filter;

	StateSpaceFlywheelController(StateSpaceGains *gains);

	void CapU();
	double Update(double curSensorVel);
	double GetVelocity();
	void SetVelocityGoal(double v);

	void Enable();

	void Disable();
};

}

#endif /* SRC_CONTROLLERS_STATESPACEFLYWHEELCONTROLLER_H_ */
