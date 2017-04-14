#include "Robot.h"
#include "AutoCommon.h"

namespace frc973 {

double off;

void Robot::MidPegKpaAuto(){
  switch(m_autoState){
    case 0:
      m_boilerPixy->Enable();
      m_drive->DriveStraight(Drive::RelativeTo::Now, -0.5,
              0.0);
      m_gearIntake->SetPickUpManual();
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
      if(m_gearIntake->IsGearReady() || GetMsecTime() - m_autoTimer >= 3000){
          m_drive
              ->TrapDrive(DriveBase::RelativeTo::Now, 93.0, -78.0 * m_autoDirection)
              ->SetHalt(true, true)
              ->SetConstraints(70.0, 70.0);
          m_autoTimer = GetMsecTime();
          m_autoState++;
      }
      break;
    case 2:
        if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 4000) {
            double boilerOffset = m_boilerPixy->GetXOffset() *
                BoilerPixy::PIXY_OFFSET_CONSTANT;
            off = boilerOffset;
            if (Util::abs(boilerOffset) >= 10.0) {
                //it's too big so screw it
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
    case 3:
        if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                GetMsecTime() - m_autoTimer > 1500) {
            m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
            m_shooter->StartAgitator(1.0, Shooter::Side::right);
            m_shooter->StartAgitator(1.0, Shooter::Side::left);
            m_shooter->StartConveyor(1.0);
            m_autoTimer = GetMsecTime();
            m_autoState++;
        }
        break;
    default:
        break;
  }
  DBStringPrintf(DB_LINE2,"off %2.2lf state %d", off,
          m_autoState);
}

}
