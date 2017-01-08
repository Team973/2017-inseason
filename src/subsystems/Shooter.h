/*
 * Shooter.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#ifndef SRC_SUBSYSTEMS_SHOOTER_H_
#define SRC_SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include "lib/CoopTask.h"

namespace frc973 {

class TaskMgr;
class CascadingFilter;
class MovingAverageFilter;
class DelaySwitch;
class PID;
class LogSpreadsheet;
class MedianFilter;
class StateSpaceFlywheelController;
class LogCell;
class LogSpreadsheet;

/**
 * Open loop control on flywheel at the moment... will do fine tuning
 * once it's shown that everything else works
 */
class Shooter : public CoopTask
{
public:
	enum ElevatorHeight {
		wayHigh,
		midHigh,
		midLow,
		wayLow
	};

	Shooter(TaskMgr *scheduler, LogSpreadsheet *logger, VictorSP *conveyorMotor);
	virtual ~Shooter();

	void SetFlywheelEnabled(bool enabledP);

	void SetFrontFlywheelSSShoot(double goal);
	void SetBackFlywheelSSShoot(double goal);

	void SetFlywheelSSShoot(double goal) {
		SetFrontFlywheelSSShoot(goal);
		SetBackFlywheelSSShoot(goal);
	}

	void SetFrontFlywheelPower(double pow);
	void SetBackFlywheelPower(double pow);

	void SetFlywheelPower(double pow) {
		SetFrontFlywheelPower(pow);
		SetBackFlywheelPower(pow);
	}

	void SetFlywheelStop();

	void SetConveyerPower(double pow) {
		if (m_conveyorControl) {
			m_conveyor->Set(pow);
		}
	}

	void SetConveyorControl(bool enabled) {
		m_conveyorControl = enabled;
	}

	void TaskPeriodic(RobotMode mode);

	double GetFrontFlywheelRate(void);
	double GetFrontFlywheelFilteredRate(void);

	double GetRearFlywheelRate(void);
	double GetRearFlywheelFilteredRate(void);

	bool FlywheelOnTarget() { return m_flywheelReady; }
	
	void Print() {
		printf("front flywheel dist %d speed %lf\n", m_frontFlywheelEncoder->Get(), GetFrontFlywheelFilteredRate());
		printf("back flywheel dist %d\n", m_backFlywheelEncoder->Get());
	}

	void SetElevatorHeight(ElevatorHeight newHeight);

	enum FlywheelState {
		openLoop,
		ssControl
	};
private:

	VictorSP *m_frontFlywheelMotor;
	VictorSP *m_backFlywheelMotor;
	VictorSP *m_conveyor;

	Counter *m_frontFlywheelEncoder;
	Counter *m_backFlywheelEncoder;

	FlywheelState m_frontFlywheelState;
	FlywheelState m_backFlywheelState;
	bool m_flywheelEnabled;

	double m_frontFlywheelTargetSpeed;
	double m_backFlywheelTargetSpeed;
	StateSpaceFlywheelController *m_frontController;
	StateSpaceFlywheelController *m_backController;
	double m_frontFlywheelSetPower;
	double m_backFlywheelSetPower;

	bool m_flywheelReady;

	CascadingFilter *m_frontFilter;
	MovingAverageFilter *m_frontMovingAvgFilt;
	double m_oldFSpeed;
	MedianFilter *m_fmedfilt;
	CascadingFilter *m_backFilter;
	DelaySwitch *m_readyFilter;

	ElevatorHeight m_elevatorState;
	Solenoid *m_longSolenoid;
	Solenoid *m_shortSolenoid;

	LogCell *m_frontFlywheelSpeed;
	LogCell *m_frontFlywheelFilteredSpeed;
	LogCell *m_shooterPow;
	LogCell *m_shooterTime;

	TaskMgr *m_scheduler;

	Solenoid *m_runningLight;
	Solenoid *m_readyLight;

	bool m_conveyorControl;

	static constexpr double SLOW_FLYWHEEL_SPEED_SCALEDOWN = 0.7;
public:
	ElevatorHeight GetElevatorState() {
		return m_elevatorState;
	}
	FlywheelState GetFrontFlywheelState() {
		return m_frontFlywheelState;
	}
	FlywheelState GetBackFlywheelState() {
		return m_backFlywheelState;
	}
};

}

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
