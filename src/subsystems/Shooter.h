/*
 * Shooter.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#pragma once

#include "WPILib.h"
#include "lib/CoopTask.h"
#include "CANTalon.h"
#include "lib/filters/Debouncer.h"
#include "Drive.h"
#include "BoilerPixy.h"

using namespace frc;

namespace frc973 {

class TaskMgr;
class LogSpreadsheet;

/**
 * Open loop control on flywheel at the moment... will do fine tuning
 * once it's shown that everything else works
 */
class Shooter : public CoopTask
{
public:
    Shooter(TaskMgr *scheduler, LogSpreadsheet *logger, CANTalon *leftAgitator, Drive *drive, BoilerPixy *boilerPixy);
    virtual ~Shooter();
    void TaskPeriodic(RobotMode mode);
    void SetFlywheelPow(double pow);
    void SetFlywheelStop();
    void SetFlywheelSpeed(double speed);

    enum ShootingSequenceState{
      idle,
      targeting,
      shooting,
      manual
    };

    bool OnTarget();

    void StartAgitator(double speed, bool side);
    void StopAgitator();
    void StartConveyor(double speed);
    void StopConveyor();
    void SetShooterState(ShootingSequenceState state);

    double GetFlywheelRate();

    static constexpr int DEFAULT_FLYWHEEL_SPEED_SETPOINT = 3000;

    enum FlywheelState {
        power,
        notRunning,
        speed
    };

private:
    TaskMgr *m_scheduler;

    FlywheelState m_flywheelState;
    ShootingSequenceState m_shootingSequenceState;

    CANTalon *m_flywheelMotorPrimary;
    CANTalon *m_flywheelMotorReplica;

    CANTalon *m_leftAgitator;
    CANTalon *m_rightAgitator;

    CANTalon *m_ballConveyor;

    double m_flywheelPow;
    double m_flywheelSpeedSetpt;

    LogCell *m_flywheelRate;
    LogCell *m_flywheelPowLog;
    LogCell *m_flywheelAmpsLog;
    LogCell *m_flywheelStateLog;
    LogCell *m_speedSetpoint;
    LogCell *m_conveyorLog;
    LogCell *m_leftAgitatorLog;
    LogCell *m_rightAgitatorLog;
    Debouncer m_flywheelOnTargetFilter;
    Drive *m_drive;
    BoilerPixy  *m_boilerPixy;

};

}
