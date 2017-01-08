/*
 * VelocityTurnPID.h
 *
 *  Created on: Apr 3, 2016
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_VELOCITYTURNPID_H_
#define SRC_CONTROLLERS_VELOCITYTURNPID_H_

#include "lib/DriveBase.h"

namespace frc973 {

class PID;

class VelocityTurnPID : public DriveController {
public:
	VelocityTurnPID();
	virtual ~VelocityTurnPID();

	/*
	 * Calculate the motor output to achieve the most recently set setpoint.
	 * This reads in position data from the angle and dist providers and uses
	 * the pid object to decide on an ideal set of outputs
	 */
	void CalcDriveOutput(DriveStateProvider *state,
			DriveControlSignalReceiver *out) override;

	/*
	 * On CalcDriveOutput, the robot sets the internal m_onTarget flag if it
	 * is within tolerance of the target.  This method returns whether we are
	 * on target.
	 */
	bool OnTarget() { return m_onTarget; }

	/*
	 * Set the target position/heading relative to absolute world
	 */
	void SetTarget(double heading, DriveBase::RelativeTo relativity,
			DriveStateProvider *state);
private:
	//maximum velocity in degrees per second (either direction)
	static constexpr double MAX_VELOCITY = 20.0;

	bool m_onTarget;

	double m_prevAngleVel;
	double m_prevAnglePos;
	double m_targetAngleVel;
	double m_targetAnglePos;

	PID *m_velPid;
	PID *m_posPid;
};

} /* namespace frc973 */

#endif /* SRC_CONTROLLERS_VELOCITYTURNPID_H_ */
