#include "GearIntake.h"
#include "RobotInfo.h"

namespace frc973{
  static constexpr double RIGHT_INDEXER_POWER = 0.8;
  static constexpr double LEFT_INDEXER_POWER = -0.4;

  static constexpr double INTAKING_POWER = 1.0;
  static constexpr double HOLDING_POWER = 0.2;

  GearIntake::GearIntake(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_gearIntakeGrip(new DoubleSolenoid(GEAR_INTAKE_GRIP_OPEN, GEAR_INTAKE_GRIP_CLOSE)),
    m_gearIntakePos(new Solenoid(GEAR_INTAKE_POS)),
    m_rightIndexer(new CANTalon(RIGHT_INDEXER_CAN_ID)),
    m_leftIndexer(new CANTalon(LEFT_INDEXER_CAN_ID)),
    m_indexer(GearIntake::Indexer::holding),
    m_gearPosition(GearPosition::up),
    m_gearIntakeState(GearIntake::GearIntakeState::released),
    m_bannerSensor(new DigitalInput(GEAR_INTAKE_BANNER_DIN)),
    m_gearTimer(0),
    m_pickUpState(GearIntake::PickUp::vomiting)
  {
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->EnableCurrentLimit(true);
    m_rightIndexer->SetCurrentLimit(8);
    m_rightIndexer->EnableCurrentLimit(true);
    m_leftIndexer->SetCurrentLimit(8);
    this->m_scheduler->RegisterTask("GearIntake", this, TASK_PERIODIC);
  }

  GearIntake::~GearIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void GearIntake::SetGearIntakeState(GearIntakeState gearIntakeState){
    switch (gearIntakeState){
      case released:
        m_gearIntakeGrip->Set(DoubleSolenoid::Value::kReverse);
        m_gearIntakeState  = GearIntake::GearIntakeState::released;
        break;
      case grabbed:
        m_gearIntakeGrip->Set(DoubleSolenoid::Value::kForward);
        m_gearIntakeState = GearIntake::GearIntakeState::grabbed;
        break;
      case floating:
        m_gearIntakeGrip->Set(DoubleSolenoid::Value::kOff);
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
        if (m_bannerSensor->Get() == true) {
            m_indexer = GearIntake::Indexer::holding;
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
    return m_bannerSensor->Get();
  }

  void GearIntake::StartPickupSequence(){
    m_pickUpState = PickUp::seeking;
  }

  void GearIntake::ReleaseGear(){
    this->SetIndexerMode(GearIntake::Indexer::stop);
    this->SetGearPos(GearIntake::GearPosition::up);
    this->SetGearIntakeState(GearIntake::GearIntakeState::released);
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
    if (m_indexer == Indexer::indexing && m_bannerSensor->Get() == true){
      this->SetIndexerMode(Indexer::holding);
    }
    switch(m_pickUpState){
      case seeking:
        this->SetIndexerMode(GearIntake::Indexer::intaking);
        this->SetGearPos(GearIntake::GearPosition::down);
        this->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
        m_gearTimer = GetMsecTime();
        if (m_rightIndexer->GetOutputCurrent() >= 8 || m_leftIndexer->GetOutputCurrent() >= 8){
            m_pickUpState = GearIntake::PickUp::chewing;
          }
        break;
      case chewing:
        this->SetIndexerMode(GearIntake::Indexer::intaking);
        if (GetMsecTime() - m_gearTimer >= 500) {
            m_pickUpState = GearIntake::PickUp::digesting;
          }
        break;
      case digesting:
        this->SetIndexerMode(GearIntake::Indexer::indexing);
        this->SetGearPos(GearIntake::GearPosition::up);
        this->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
        break;
      case vomiting:
        this->ReleaseGear();
        break;
    }
  }
}
