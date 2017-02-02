#include "subsystems/BallIntake.h"
#include "lib/WrapDash.h"
#include "Robot.h"
#include "RobotInfo.h"
#include "lib/TaskMgr.h"

namespace frc973{
  BallIntake::BallIntake(TaskMgr *scheduler)
  :
  m_scheduler(scheduler),
  m_ballIntakeState(BallIntakeState::notRunning),
  m_ballIntakeMotor(new CANTalon(BALL_INTAKE_CAN_ID)),
  m_ballIntakePow(0.0)
  {
    this->m_scheduler->RegisterTask("BallIntake", this, TASK_PERIODIC);
  }

  BallIntake::~BallIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void BallIntake::BallIntakeStart(){
    m_ballIntakeState = BallIntakeState::running;
  }

  void BallIntake::BallIntakeStop(){
    m_ballIntakeState = BallIntakeState::notRunning;
  }

  void BallIntake::SetIntakePower(double power){
    m_ballIntakePow = power;
    m_ballIntakeState = BallIntakeState::manual;
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_ballIntakeMotor->Set(power);
  }

  void BallIntake::TaskPeriodic(){
      switch (m_ballIntakeState) {
        case running:
          m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
          m_ballIntakeMotor->Set(BALL_INTAKE_RUNNING_SPEED);
        break;
        case notRunning:
          m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
          m_ballIntakeMotor->Set(0.0);
        break;
        case reverse:
          m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
          m_ballIntakeMotor->Set(BALL_INTAKE_REVERSE_SPEED);
        break;
        case manual:
        break;
      }
  }
}
