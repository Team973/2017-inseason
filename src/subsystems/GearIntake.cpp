#include "GearIntake.h"
#include "RobotInfo.h"

namespace frc973{
  static constexpr double RIGHT_INDEXER_POWER = -0.08;
  static constexpr double LEFT_INDEXER_POWER = 0.04;

  static constexpr double INTAKING_POWER = -0.7;
  static constexpr double HOLDING_POWER = -0.0;

  GearIntake::GearIntake(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_gearIntakeState(GearIntake::GearIntakeState::grabbed),
    m_gearPosition(GearPosition::up),
    m_indexer(GearIntake::Indexer::holding),
    m_pickUpState(GearIntake::PickUp::vomiting),
    m_gearIntakeRelease(new Solenoid(GEAR_INTAKE_GRIP_OPEN)),
    m_gearIntakeGrab(new Solenoid(GEAR_INTAKE_GRIP_CLOSE)),
    m_gearIntakePos(new Solenoid(GEAR_INTAKE_POS)),
    m_pushTopLeft(new DigitalInput(PUSH_SENSOR_TOP_LEFT)),
    m_pushTopRight(new DigitalInput(PUSH_SENSOR_TOP_RIGHT)),
    m_pushBottom(new DigitalInput(PUSH_SENSOR_BOTTOM)),
    m_leftIndexer(new CANTalon(LEFT_INDEXER_CAN_ID)),
    m_rightIndexer(new CANTalon(RIGHT_INDEXER_CAN_ID)),
    m_gearTimer(0),
    m_driverReleased(false)
  {
    m_rightIndexer->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
    m_leftIndexer->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
    m_rightIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_leftIndexer->SetInverted(true);
    m_rightIndexer->SetInverted(false);
    m_leftIndexer->EnableCurrentLimit(true);
    m_rightIndexer->SetCurrentLimit(100);
    m_rightIndexer->EnableCurrentLimit(true);
    m_leftIndexer->SetCurrentLimit(100);
    this->SetGearIntakeState(GearIntakeState::grabbed);
    this->m_scheduler->RegisterTask("GearIntake", this, TASK_PERIODIC);
  }

  GearIntake::~GearIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void GearIntake::SetGearIntakeState(GearIntakeState gearIntakeState){
    m_pickUpState = PickUp::manual;
    switch (gearIntakeState){
      case released:
        m_gearIntakeGrab->Set(true);
        m_gearIntakeRelease->Set(true);
        m_gearIntakeState  = GearIntake::GearIntakeState::released;
        break;
      case grabbed:
        m_gearIntakeGrab->Set(false);
        m_gearIntakeRelease->Set(false);
        m_gearIntakeState = GearIntake::GearIntakeState::grabbed;
        break;
      case floating:
        m_gearIntakeGrab->Set(true);
        m_gearIntakeRelease->Set(false);
        m_gearIntakeState = GearIntake::GearIntakeState::floating;
        break;
    }
  }

  void GearIntake::SetGearPos(GearPosition gearPosition){
    m_pickUpState = PickUp::manual;
    switch (gearPosition){
      case up:
        m_gearIntakePos->Set(false);
        m_gearPosition = GearPosition::up;
        break;
      case down:
        m_gearIntakePos->Set(true);
        SetGearIntakeState(GearIntakeState::grabbed);
        m_gearPosition = GearPosition::down;
        break;
    }
  }

  void GearIntake::SetIndexerMode(Indexer indexerMode){
    m_pickUpState = PickUp::manual;
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
        if (IsGearReady() == true) {
            m_indexer = GearIntake::Indexer::holding;
          }
        break;
      case holding:
        m_rightIndexer->Set(HOLDING_POWER);
        m_leftIndexer->Set(HOLDING_POWER);
        m_indexer = GearIntake::Indexer::holding;
        break;
       case stop:
        m_rightIndexer->Set(0.0);
        m_leftIndexer->Set(0.0);
        m_indexer = GearIntake::Indexer::stop;
        break;
    }
  }

  bool GearIntake::IsGearReady(){
    if (m_pushTopLeft->Get() + m_pushTopRight->Get() + m_pushBottom->Get() < 2){
      return true;
    }
    else{
      return false;
    }
  }

  void GearIntake::StartPickupSequence(){
    m_pickUpState = PickUp::seeking;
  }

  void GearIntake::ReleaseGear(){
      m_pickUpState = PickUp::vomiting;
      this->SetIndexerMode(Indexer::stop);
      this->SetGearIntakeState(GearIntakeState::released);
  }

  void GearIntake::SetReleaseAutoEnable(bool driverInput){
    m_driverReleased = driverInput;
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
    if (m_indexer == Indexer::indexing && IsGearReady() == true){
      this->SetIndexerMode(Indexer::holding);
    }
    switch(m_pickUpState){
      case seeking:
        this->SetIndexerMode(GearIntake::Indexer::intaking);
        this->SetGearPos(GearIntake::GearPosition::down);
        this->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
        m_pickUpState = PickUp:: seeking;
        m_gearTimer = GetMsecTime();
        if (m_rightIndexer->GetOutputCurrent() >= 50 || m_leftIndexer->GetOutputCurrent() >= 50){
            m_pickUpState = GearIntake::PickUp::chewing;
          }
        break;
      case chewing:
        this->SetIndexerMode(GearIntake::Indexer::intaking);
        m_pickUpState = PickUp:: chewing;
        if (GetMsecTime() - m_gearTimer >= 200) {
            m_pickUpState = GearIntake::PickUp::digesting;
        }
        break;
      case digesting:
        this->SetIndexerMode(GearIntake::Indexer::holding);
        this->SetGearPos(GearIntake::GearPosition::up);
        this->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
        if (IsGearReady() == true && m_driverReleased == true) {
          m_pickUpState = PickUp::vomiting;
        }
        else{
          m_pickUpState = PickUp::digesting;
        }
        break;
      case vomiting:
        this->ReleaseGear();
        m_gearTimer = GetMsecTime();
        m_pickUpState = PickUp::postVomit;
        break;
      case postVomit:
        if (m_gearTimer - GetMsecTime() == 3000) {
          this->SetGearPos(GearPosition::down);
        }
        break;
      case manual:
        break;
    }
  }
}
