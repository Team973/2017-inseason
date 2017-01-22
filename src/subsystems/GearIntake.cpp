#include "GearIntake.h"
#include "RobotInfo.h"

namespace frc973{
  static constexpr double RIGHT_INDEXER_POWER = 0.8;
  static constexpr double LEFT_INDEXER_POWER = -0.4;

  GearIntake::GearIntake(TaskMgr *scheduler) :
  m_scheduler(scheduler),
  m_gearIntakeGrip(new Solenoid(GEAR_INTAKE_GRIP)),
  m_gearIntakePos(new Solenoid(GEAR_INTAKE_POS)),
  m_rightIndexer(new CANTalon(RIGHT_INDEXER_CAN_ID)),
  m_leftIndexer(new CANTalon(LEFT_INDEXER_CAN_ID)),
  m_indexer(GearIntake::Indexer::holding),
  m_gearPosition(GearPosition::up),
  m_gearIntakeState(GearIntake::GearIntakeState::released),
  m_bannerSensor(false)
  {
    this->m_scheduler->RegisterTask("GearIntake", this, TASK_PERIODIC);

  }

  GearIntake::~GearIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void GearIntake::GrabGears(){
    m_gearIntakeGrip->Set(false);
    m_gearIntakeState = GearIntake::GearIntakeState::grabbed;
  }

  void GearIntake::ReleaseGears(){
    m_gearIntakeGrip->Set(true);
    m_gearIntakeState  = GearIntake::GearIntakeState::released;
  }

  void GearIntake::FloatGears(){
    m_gearIntakeState = GearIntake::GearIntakeState::floating;
  }

  void GearIntake::SetIntakeUp(){
    m_gearIntakePos->Set(true);
    m_gearPosition = GearPosition::up;
  }

  void GearIntake::SetIntakeDown(){
    m_gearIntakePos->Set(false);
    m_gearPosition = GearPosition::down;
  }

  void GearIntake::SetIntakingMode(){
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_rightIndexer->Set(1.0);
    m_leftIndexer->Set(1.0);
    m_indexer = GearIntake::Indexer::intaking;
  }

  void GearIntake::SetHoldingMode(){
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_rightIndexer->Set(0.0);
    m_leftIndexer->Set(0.0);
    m_indexer = GearIntake::Indexer::holding;
  }

  void GearIntake::SetIndexingMode(){
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_rightIndexer->Set(RIGHT_INDEXER_POWER);
    m_leftIndexer->Set(LEFT_INDEXER_POWER);
    m_indexer = GearIntake::Indexer::indexing;
  }

  bool GearIntake::IsGearAligned(){
    if (true) {
      return true;
    }
    else {
      return false;
    }
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
  }
}
