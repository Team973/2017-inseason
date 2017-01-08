/*
 * CheesyDrive.h
 *
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_CHEESYDRIVECONTROLLER_H_
#define SRC_CONTROLLERS_CHEESYDRIVECONTROLLER_H_

#include "lib/DriveBase.h"

namespace frc973 {

/*
 * CheesyDrive does the calculations and maintains state for a cheesy drive
 * control scheme.  At the moment you need to call SetJoysticks and
 * SetQuickturn from the outside.  An alternative would be for CheesyDrive
 * to hold a reference to the joysticks and just to poll them itself.
 */
class CheesyDriveController : public DriveController {
public:
	/*
	 * Initialize CheesyDrive motor controller.  For the most part this
	 * just means set up everything to zero.
	 */
	CheesyDriveController();
	virtual ~CheesyDriveController();

	/*
	 * Calculate motor output given the most recent joystick commands.
	 * CheesyDrive is open-loop, so it doesn't read angle or dist (they could
	 * be null for all we care)
	 */
	void CalcDriveOutput(DriveStateProvider *state,
			DriveControlSignalReceiver *out);

	/*
	 * Since CheesyDrive is open-loop, OnTarget doesn't make sense here...
	 * just return true I guess...
	 */
	bool OnTarget() { return true; }

	/*
	 * Set the joystick values so that when we go to calculate drive output,
	 * cheesy drive can use them.
	 */
	void SetJoysticks(double throttle, double turn);

	/*
	 * Set whether or not quickturn is enabled.
	 */
	void SetQuickTurn(bool quickturn);
private:
	double m_leftOutput;
	double m_rightOutput;

	bool m_quickTurn;
	bool m_highGear;

	double m_oldWheel;
	double m_negInertiaAccumulator;
	double m_quickStopAccumulator;
};

}

#endif /* SRC_CONTROLLERS_CHEESYDRIVECONTROLLER_H_ */
