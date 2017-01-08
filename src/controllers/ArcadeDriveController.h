/*
 * ArcadeDrive.h
 *
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_ARCADEDRIVECONTROLLER_H_
#define SRC_CONTROLLERS_ARCADEDRIVECONTROLLER_H_

#include "lib/DriveBase.h"
#include <stdio.h>

namespace frc973 {

class ArcadeDriveController : public DriveController {
public:
	ArcadeDriveController();
	virtual ~ArcadeDriveController();

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
	void SetJoysticks(double throttle, double turn);

	void Start() override {
		printf("Turning on Arcade Mode\n");
	}

	void Stop() override {
		printf("Turning off arcade Mode\n");
	}
private:
	double m_leftOutput;
	double m_rightOutput;
};

}

#endif /* SRC_CONTROLLERS_ARCADEDRIVECONTROLLER_H_ */
