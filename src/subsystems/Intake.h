/*
 * Intake.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 *
 * Intake module handles extending and retracting the the intake arm, and
 * running the roller on the end of that intake arm.
 *
 * Arm extends with the powering of one solenoid.  It retracts with the
 * unpowering of that solenoid because there's surgical tubing pulling
 * it in.
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include "WPILib.h"
#include "lib/CoopTask.h"

namespace frc973 {

class Intake : public CoopTask {
public:
	enum class IntakeMode {
		off,
		forward,
		reverse,
		manual
	};

	enum class IntakePosition {
		extended,
		retracted
	};

	Intake(TaskMgr *scheduler);
	virtual ~Intake();

	void SetIntakeMode(IntakeMode mode);

	void SetIntakePosition(IntakePosition newPos);

	void SetIntakePower(double pow);

	void TaskPeriodic(RobotMode mode);
private:
	VictorSP *m_intakeMotor;
	VictorSP *m_intakeMotorB;
	Solenoid *m_intakeSolenoid;

	IntakeMode m_intakeMode;
	IntakePosition m_intakePosition;

	TaskMgr *m_scheduler;

	static constexpr int INTAKE_FORWARD_SPEED = 1.0;
	static constexpr int INTAKE_REVERSE_SPEED = -1.0;
};

}

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
