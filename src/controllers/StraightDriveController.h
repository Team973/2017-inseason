/*
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_STRAIGHTDRIVECONTROLLER_H_
#define SRC_CONTROLLERS_STRAIGHTDRIVECONTROLLER_H_

#include "lib/DriveBase.h"

namespace frc973 {

class PID;

class StraightDriveController : public DriveController {
public:
	StraightDriveController();
	virtual ~StraightDriveController();

	/*
	 * Calculate motor output given the most recent joystick commands.
	 * In this case just return the most recent joystick commands.
	 */
	void CalcDriveOutput(DriveStateProvider *state,
			DriveControlSignalReceiver *out);

	/*
	 * This controller is open-loop so OnTarget doesn't make sense here...
	 * just return false I guess...
	 */
	bool OnTarget() { return false; }

	/*
	 * Set the joystick values (which in this case will be output)
	 */
	void SetJoysticks(double throttle);

	/*
	 * Set the target heading that this controller should maintain
	 * while driving
	 */
	void SetTargetHeading(double headingDegrees);
private:
	double m_leftOutput;
	double m_rightOutput;
	PID *m_turnPid;
	double m_targetHeading;
	double m_throttle;
};

}

#endif /* SRC_CONTROLLERS_STRAIGHTDRIVECONTROLLER_H_ */
