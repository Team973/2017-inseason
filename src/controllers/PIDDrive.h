/*
 * PIDDriveController.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#ifndef SRC_CONTROLLERS_PIDDRIVECONTROLLER_H_
#define SRC_CONTROLLERS_PIDDRIVECONTROLLER_H_


#include "lib/DriveBase.h"

namespace frc973 {

class PID;

class PIDDriveController : public DriveController {
public:
	PIDDriveController();
	virtual ~PIDDriveController();

	/*
	 * Calculate the motor output to achieve the most recently set setpoint.
	 * This reads in position data from the angle and dist providers and uses
	 * the pid object to decide on an ideal set of outputs
	 */
	void CalcDriveOutput(DriveStateProvider *state,
			DriveControlSignalReceiver *out) override;

	void Start() override {
		m_needSetControlMode = true;
	}

	/*
	 * On CalcDriveOutput, the robot sets the internal m_onTarget flag if it
	 * is within tolerance of the target.  This method returns whether we are
	 * on target.
	 */
	bool OnTarget() override { return m_onTarget; }

	/*
	 * Set the target position/heading relative to absolute world
	 */
	void SetTarget(double dist, double heading, DriveBase::RelativeTo relativity,
			DriveStateProvider *state);

	/*
	 * Enable distance pid, do angle and dist together.
	 */
	void EnableDist() {
		m_distEnabled = true;
	}

	/*
	 * Disable distance pid, do angle only.
	 *
	 * This is a cheap hack because we only have an encoder on left drive...
	 * if we had left and right we could actually determine our forward-ness
	 */
	void DisableDist() {
		m_distEnabled = false;
	}

    /*
     * Scale the pseed down by |newCap|
     *
     * |newCap| of 1.0 means max speed
     */
	void SetCap (double newCap){
		m_speedCap = newCap;
	}

    /*
     * Set the tolerance for distance exiting
     */
    PIDDriveController *SetDistTolerance(double dist=2.0, double rate=2.0) {
        m_distTolerance = dist;
        m_distRateTolerance = rate;
        return this;
    }

    /*
     * Set the tolerance for distance exiting
     */
    PIDDriveController *SetAngleTolerance(double angle=2.0, double rate=2.0) {
        m_angleTolerance = angle;
        m_angleRateTolerance = rate;
        return this;
    }

	void Zero() {
		m_prevDist = 0.0;
		m_prevAngle = 0.0;
		m_targetDist = 0.0;
		m_targetAngle = 0.0;
		m_onTarget = false;
	}
private:
    bool m_needSetControlMode = true;
	double m_prevDist;
	double m_prevAngle;

	double m_targetDist;
	double m_targetAngle;

	bool m_onTarget;

	PID *m_drivePID;
	PID *m_turnPID;

	bool m_distEnabled;

	double m_speedCap;
    double m_lastThrottle;

    double m_distTolerance;
    double m_distRateTolerance;
    double m_angleTolerance;
    double m_angleRateTolerance;

    /**
     * In in/sec
     */
    static constexpr double DEFAULT_DIST_TOLERANCE = 2.0;
    static constexpr double DEFAULT_DIST_RATE_TOLERANCE = 5.0;

    /**
     * In deg/sec
     */
    static constexpr double DEFAULT_ANGLE_TOLERANCE = 2.0;
    static constexpr double DEFAULT_ANGLE_RATE_TOLERANCE = 5.0;
};

}

#endif /* SRC_CONTROLLERS_PIDDRIVECONTROLLER_H_ */
