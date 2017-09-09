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
  m_logger(logger),
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

    m_voltage = new LogCell("BallIntake Voltage", 32, true);
    m_current = new LogCell("BallIntake Current", 32, true);
    m_logger->RegisterCell(m_voltage);
    m_logger->RegisterCell(m_current);
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

  /**
   * Sets ball intake power (open loop)
   *
   * @param power power to be sent from -1.0 to 1.0
   */
  void BallIntake::SetIntakePower(double power){
    m_ballIntakePow = power;
    m_ballIntakeState = BallIntakeState::manual;
    m_ballIntakeMotor->Set(power);
  }

  void BallIntake::ExpandHopper(){
    m_hopperSolenoid->Set(true);
    printf("actuate hopper");
  }

  void BallIntake::RetractHopper(){
    m_hopperSolenoid->Set(false);
  }

  void BallIntake::TaskPeriodic(RobotMode mode){
    m_voltage->LogDouble(m_ballIntakeMotor->GetOutputVoltage());
    m_current->LogDouble(m_ballIntakeMotor->GetOutputCurrent());
      switch (m_ballIntakeState) {
        case running:
          m_ballIntakeMotor->Set(0.8);
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
