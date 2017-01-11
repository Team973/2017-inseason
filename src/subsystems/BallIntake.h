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

using namespace frc;

namespace frc973 {
  class TaskMgr;

  class BallIntake : public CoopTask{
  public:
    enum BallIntakeState{
      running,
      notRunning,
      manual,
      reverse
    };
      BallIntake(TaskMgr *scheduler);
      virtual ~BallIntake();
      void BallIntakeStart();
      void BallIntakeStop();
      void SetIntakePower(double power);
      void TaskPeriodic();

  private:
      TaskMgr *m_scheduler;
      VictorSP *m_ballIntakeMotor;
      VictorSP *m_ballIntakeMotorB;

      BallIntakeState m_ballIntakeState;

      double m_ballIntakePow;

      static constexpr int BALL_INTAKE_RUNNING_SPEED = 1.0;
      static constexpr int BALL_INTAKE_REVERSE_SPEED = -1.0;
  };
}

#endif /*SRC_SUBSYSTEMS_BALL_INTAKE_H*/
