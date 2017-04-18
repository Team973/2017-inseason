#include "Robot.h"
#include "AutoCommon.h"

namespace frc973 {

/*
LiteralDrive: -0.38, 0.0, false, false, 2.0, 3.4, 3.0, 4.0
LiteralDrive: -1.3, -0.67, false, false, 2.0, 3.4, 3.0, 4.0
SpinupShooter
LiteralDrive: -0.66, -0.12, false, false, 2.0, 3.4, 3.0, 4.0
*/

void Robot::LiteralCitrusHopperAuto(){
  switch(m_autoState){
    case 0:
      m_drive
        ->TrapDrive(DriveBase::RelativeTo::Now, -14.0, 0.0 * m_autoDirection)
        ->SetHalt(true, false)
        ->SetConstraints(40.0, 48.0);
      m_gearIntake->SetPickUpManual();
      m_compressor->Disable();
      m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
      m_shooter->SetFlywheelSpeed(2950);
      m_shooter->StopAgitator();
      m_shooter->StartConveyor(0.0);
      m_autoTimer = GetMsecTime();
      m_autoState++;
      break;
    case 1:
      if (GetMsecTime() - m_autoTimer > 250) {
          m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
      }
      if(m_drive->OnTarget()){
        m_drive
          ->TrapDrive(DriveBase::RelativeTo::SetPoint, -51.0, -38.5 * m_autoDirection)
          ->SetHalt(false, true)
          ->SetConstraints(40.0, 48.0);
        m_autoTimer = GetMsecTime();
        m_autoState++;
      }
      break;
    case 2:
      if (m_drive->OnTarget()) {
        m_drive
          ->TrapDrive(DriveBase::RelativeTo::SetPoint, -26.0, -6.87 * m_autoDirection)
          ->SetHalt(true, true)
          ->SetConstraints(40.0, 48.0);
          m_autoState++;
        }
      break;
    case 3:
      if (m_drive->OnTarget()) {
        m_ballIntake->ExpandHopper();
        m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
        m_shooter->StartAgitator(1.0, Shooter::Side::right);
        m_shooter->StartAgitator(1.0, Shooter::Side::left);
        m_shooter->StartConveyor(0.7);
        m_autoTimer = GetMsecTime();
        m_autoState++;
      }
      break;
    case 4:
      if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1000) {
          double boilerOffset = m_boilerPixy->GetXOffset() *
              BoilerPixy::PIXY_OFFSET_CONSTANT;

          if (Util::abs(boilerOffset) >= 10.0) {
              m_autoState++;
          }
          else {
            m_drive
                ->PIDTurn(m_drive->GetAngle() - boilerOffset,
                           DriveBase::RelativeTo::Absolute, 1.0)
                ->SetDistTolerance(15.0, 25.0)
                ->SetAngleTolerance(30.0, 60.0);
            m_autoTimer = GetMsecTime();
            m_autoState++;
          }
      }
      break;
    default:
        break;
  }
}

void Robot::ModifiedCitrusHopperAuto(){
  switch(m_autoState){
    case 0:
      m_drive
        ->TrapDrive(DriveBase::RelativeTo::Now, -42.0, 45.0 * m_autoDirection)
        ->SetHalt(true, false)
        ->SetConstraints(40.0, 48.0);
      m_gearIntake->SetPickUpManual();
      m_compressor->Disable();
      m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
      m_shooter->SetFlywheelSpeed(2990);
      m_shooter->StopAgitator();
      m_shooter->StartConveyor(0.0);
      m_autoTimer = GetMsecTime();
      m_autoState++;
      break;
    case 1:
      if (GetMsecTime() - m_autoTimer > 250) {
          m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
      }
      if(m_drive->OnTarget()){
        m_drive
          ->TrapDrive(DriveBase::RelativeTo::SetPoint, -48.0, -45.0 * m_autoDirection)
          ->SetHalt(false, true)
          ->SetConstraints(40.0, 48.0);
        m_autoTimer = GetMsecTime();
        m_autoState++;
      }
      break;
    case 2:
      if (m_drive->OnTarget()) {
        m_drive
          ->TrapDrive(DriveBase::RelativeTo::SetPoint, -12.0, -5.0 * m_autoDirection)
          ->SetHalt(true, true)
          ->SetConstraints(40.0, 48.0);
          m_autoState++;
        }
      break;
    case 3:
      if (m_drive->OnTarget()) {
        m_ballIntake->ExpandHopper();
        m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
        m_shooter->StartAgitator(1.0, Shooter::Side::right);
        m_shooter->StartAgitator(1.0, Shooter::Side::left);
        m_shooter->StartConveyor(0.7);
        m_autoTimer = GetMsecTime();
        m_autoState++;
      }
      break;
    case 4:
      if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1000) {
          double boilerOffset = m_boilerPixy->GetXOffset() *
              BoilerPixy::PIXY_OFFSET_CONSTANT;

          if (Util::abs(boilerOffset) >= 10.0) {
              m_autoState++;
          }
          else {
            m_drive
                ->PIDTurn(m_drive->GetAngle() - boilerOffset,
                           DriveBase::RelativeTo::Absolute, 1.0)
                ->SetDistTolerance(15.0, 25.0)
                ->SetAngleTolerance(30.0, 60.0);
            m_autoTimer = GetMsecTime();
            m_autoState++;
          }
      }
      break;

    default:
        break;
  }
}

}
