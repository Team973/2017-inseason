#include "subsystems/BallIntake.h"
#include "lib/WrapDash.h"
#include "Robot.h"
#include "RobotInfo.h"
#include "lib/TaskMgr.h"

namespace frc973{
  BallIntake::BallIntake(TaskMgr *scheduler)
  :
  m_scheduler(scheduler),
  m_ballIntakeMotor(new CANTalon(BALL_INTAKE_CAN_ID, 50)),
  m_ballIntakeState(BallIntakeState::notRunning),
  m_ballIntakePow(0.0)
  {
    this->m_scheduler->RegisterTask("BallIntake", this, TASK_PERIODIC);
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_ballIntakeMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_ballIntakeMotor->EnableCurrentLimit(true);
    m_ballIntakeMotor->SetCurrentLimit(20);
    m_ballIntakeMotor->SetVoltageRampRate(120.0);
  }

  BallIntake::~BallIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void BallIntake::BallIntakeStart(){
    m_ballIntakeState = BallIntakeState::running;
  }

  void BallIntake::BallIntakeStartReverse(){
    m_ballIntakeState = BallIntakeState::reverse;
  }

  void BallIntake::BallIntakeStop(){
    m_ballIntakeState = BallIntakeState::notRunning;
  }

  void BallIntake::SetIntakePower(double power){
    m_ballIntakePow = power;
    m_ballIntakeState = BallIntakeState::manual;
    m_ballIntakeMotor->Set(power);
  }

  void BallIntake::TaskPeriodic(RobotMode mode){
      switch (m_ballIntakeState) {
        case running:
          m_ballIntakeMotor->Set(BALL_INTAKE_RUNNING_POW);
          printf("running ball Intake");
        break;
        case notRunning:
          m_ballIntakeMotor->Set(0.0);
        break;
        case reverse:
          m_ballIntakeMotor->Set(BALL_INTAKE_REVERSE_POW);
        break;
        case manual:
        break;
      }
  }
}
