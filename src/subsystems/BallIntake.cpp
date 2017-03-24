#include "subsystems/BallIntake.h"
#include "lib/WrapDash.h"
#include "Robot.h"
#include "RobotInfo.h"
#include "lib/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"

namespace frc973{
  BallIntake::BallIntake(TaskMgr *scheduler, LogSpreadsheet *logger)
  :
  m_scheduler(scheduler),
  m_ballIntakeMotor(new CANTalon(BALL_INTAKE_CAN_ID, 50)),
  m_ballIntakeState(BallIntakeState::notRunning),
  m_hopperSolenoid(new Solenoid(HOPPER_SOLENOID)),
  m_ballIntakePow(0.0)
  {
    this->m_scheduler->RegisterTask("BallIntake", this, TASK_PERIODIC);
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_ballIntakeMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_ballIntakeMotor->EnableCurrentLimit(true);
    m_ballIntakeMotor->SetCurrentLimit(40);
    m_ballIntakeMotor->SetVoltageRampRate(120.0);
  }

  BallIntake::~BallIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void BallIntake::BallIntakeStart(){
    m_ballIntakeState = BallIntakeState::running;
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  }

  void BallIntake::BallIntakeStartReverse(){
    m_ballIntakeState = BallIntakeState::reverse;
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  }

  void BallIntake::BallIntakeStop(){
    m_ballIntakeState = BallIntakeState::notRunning;
    m_ballIntakeMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  }

  void BallIntake::SetIntakePower(double power){
    m_ballIntakePow = power;
    m_ballIntakeState = BallIntakeState::manual;
    m_ballIntakeMotor->Set(power);
  }

  void BallIntake::ExpandHopper(){
    m_hopperSolenoid->Set(true);
  }

  void BallIntake::RetractHopper(){
    m_hopperSolenoid->Set(false);
  }

  void BallIntake::TaskPeriodic(RobotMode mode){
      switch (m_ballIntakeState) {
        case running:
          m_ballIntakeMotor->Set(1.0);
          break;
        case notRunning:
          m_ballIntakeMotor->Set(0.0);
          break;
        case reverse:
          m_ballIntakeMotor->Set(BALL_INTAKE_REVERSE_POW);
          break;
        case manual:
          m_ballIntakeMotor->Set(m_ballIntakePow);
          break;
      }
  }
}
