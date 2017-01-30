#include "GearIntake.h"
#include "RobotInfo.h"

namespace frc973{
  static constexpr double RIGHT_INDEXER_POWER = 0.8;
  static constexpr double LEFT_INDEXER_POWER = -0.4;

  static constexpr double INTAKING_POWER = 1.0;
  static constexpr double HOLDING_POWER = 0.2;

  GearIntake::GearIntake(TaskMgr *scheduler) :
  m_scheduler(scheduler),
  m_gearIntakeGrip(new Solenoid(GEAR_INTAKE_GRIP)),
  m_gearIntakePos(new Solenoid(GEAR_INTAKE_POS)),
  m_rightIndexer(new CANTalon(RIGHT_INDEXER_CAN_ID)),
  m_leftIndexer(new CANTalon(LEFT_INDEXER_CAN_ID)),
  m_indexer(GearIntake::Indexer::holding),
  m_gearPosition(GearPosition::up),
  m_gearIntakeState(GearIntake::GearIntakeState::released),
  m_bannerSensor(new DigitalInput(GEAR_INTAKE_BANNER_DIN)),
  m_sensorValue(false)
  {
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    this->m_scheduler->RegisterTask("GearIntake", this, TASK_PERIODIC);
  }

  GearIntake::~GearIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void GearIntake::SetGearIntakeState(GearIntakeState gearIntakeState){
    switch (gearIntakeState){
      case released:
        m_gearIntakeGrip->Set(true);
        m_gearIntakeState  = GearIntake::GearIntakeState::released;
        break;
      case grabbed:
        m_gearIntakeGrip->Set(false);
        m_gearIntakeState = GearIntake::GearIntakeState::grabbed;
        break;
      case floating:
        m_gearIntakeState = GearIntake::GearIntakeState::floating;
        break;
    }
  }

  void GearIntake::SetGearPos(GearPosition gearPosition){
    switch (gearPosition){
      case up:
        m_gearIntakePos->Set(true);
        m_gearPosition = GearPosition::up;
        break;
      case down:
        m_gearIntakePos->Set(false);
        m_gearPosition = GearPosition::down;
        break;
    }
  }

  void GearIntake::SetIndexerMode(Indexer indexerMode){
    switch (indexerMode) {
      case intaking:
        m_rightIndexer->Set(INTAKING_POWER);
        m_leftIndexer->Set(INTAKING_POWER);
        m_indexer = GearIntake::Indexer::intaking;
        break;
      case indexing:
        m_rightIndexer->Set(RIGHT_INDEXER_POWER);
        m_leftIndexer->Set(LEFT_INDEXER_POWER);
        m_indexer = GearIntake::Indexer::indexing;
        if (m_sensorValue == true) {
            m_indexer = GearIntake::Indexer::indexing;
          }
        break;
      case holding:
        m_rightIndexer->Set(HOLDING_POWER);
        m_leftIndexer->Set(HOLDING_POWER);
        m_indexer = GearIntake::Indexer::holding;
        break;
    }
  }

  bool GearIntake::IsGearAligned(){
    if (m_sensorValue == true) {
      return true;
    }
    else {
      return false;
    }
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
    m_sensorValue = m_bannerSensor->Get();
  }
}
