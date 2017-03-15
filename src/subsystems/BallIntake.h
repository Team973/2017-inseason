/**
  * BallIntake.h and .cpp
  * Created on January 10,2017
  */

#ifndef SRC_SUBSYSTEMS_BALL_INTAKE_H
#define SRC_SUBSYSTEMS_BALL_INTAKE_H

#include "WPILib.h"
#include "RobotInfo.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "CANTalon.h"

using namespace frc;

namespace frc973 {
  class TaskMgr;
  class LogSpreadsheet;
  class LogCell;

  class BallIntake : public CoopTask{
  public:
    enum BallIntakeState{
      running,
      notRunning,
      manual,
      reverse
    };
    BallIntake(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~BallIntake();
    void BallIntakeStart();
    void BallIntakeStop();
    void SetIntakePower(double power);
    void BallIntakeStartReverse();
    void ExpandHopper();
    void RetractHopper();
    void TaskPeriodic(RobotMode mode) override;

  private:
    TaskMgr *m_scheduler;
    CANTalon *m_ballIntakeMotor;

    BallIntakeState m_ballIntakeState;
    Solenoid *m_hopperSolenoidLeft;
    Solenoid *m_hopperSolenoidRight;

    double m_ballIntakePow;

    static constexpr int BALL_INTAKE_RUNNING_POW = 0.75;
    static constexpr int BALL_INTAKE_REVERSE_POW = -0.75;
  };
}

#endif /*SRC_SUBSYSTEMS_BALL_INTAKE_H*/
