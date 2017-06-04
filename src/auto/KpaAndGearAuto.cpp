/**
 * This auto routine must start with the hopper away from the wall.
 * Attempted to do this, did not use this for competition
 */

#include "Robot.h"
#include "AutoCommon.h"

namespace frc973 {

//double offgear;

void Robot::KpaAndGearAuto(){
  switch(m_autoState){
    case 0:
      m_boilerPixy->Enable();
      m_drive->PIDDrive(-55.5, 0.0, DriveBase::RelativeTo::Now, 1.0);
      m_gearIntake->SetPickUpManual();
      m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
      m_shooter->SetFlywheelSpeed(2920);
      m_shooter->StopAgitator();
      m_shooter->StartConveyor(0.0);
      m_autoTimer = GetMsecTime();
      m_autoState++;
      break;
    case 1:
      if (GetMsecTime() - m_autoTimer > 250) {
          m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
      }
      if(m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2000){
          m_drive->PIDTurn(-31.5 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
              ->SetAngleTolerance(10.0, 2.0);
          m_autoTimer = GetMsecTime();
          m_autoState++;
      }
      break;
    case 2:
        if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2000) {
            double boilerOffset = m_boilerPixy->GetXOffset() *
                BoilerPixy::PIXY_OFFSET_CONSTANT;
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
    case 4:
        if(GetMsecTime() - m_autoTimer >= 2500){
          m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
              ->SetAngleTolerance(10.0, 2.0);
          m_shooter->StopAgitator();
          m_shooter->StartConveyor(0.0);
          m_autoTimer = GetMsecTime();
          m_autoState++;
        }
        break;
    case 5:
      if(m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500){
        m_drive->PIDDrive(-50.0, 0.0, DriveBase::RelativeTo::Now, 1.0);
        m_autoTimer = GetMsecTime();
        m_autoState++;
      }
      break;
    case 6:
        if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500) {
            double gearOffset = m_pixyR->GetOffset() *
                PixyThread::GEAR_DEGREES_PER_PIXEL;
            //offgear = gearOffset;
            if (Util::abs(gearOffset) >= 10.0) {
                m_autoState++;
            }
            else {
              m_drive
                  ->PIDTurn(m_drive->GetAngle() - gearOffset,
                             DriveBase::RelativeTo::Absolute, 1.0)
                  ->SetDistTolerance(15.0, 25.0)
                  ->SetAngleTolerance(4.0, 4.0);
              m_autoTimer = GetMsecTime();
              m_autoState++;
            }
        }
        break;
    case 7:
        if (m_drive->OnTarget()) {
            m_autoTimer = GetMsecTime();
            m_drive->ArcadeDrive(-0.3, 0.0);
            m_autoState++;
        }
        break;
    case 8:
        if (m_gearIntake->IsGearReady()) {
            //hit the gear, continue normally
            m_drive->PIDDrive(30.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
            m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
            m_autoState++;
        }
        else if (GetMsecTime() - m_autoTimer > 3000) {
            //we did not hit it after 3 seconds so back up and try again
            m_drive->PIDDrive(20.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
            m_autoState = 6;
        }
        break;
    case 9:
        //should be done scoring gear... make hair merry red left
        if (m_drive->OnTarget()) {
            m_drive->PIDTurn(-90.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
            m_autoState++;
        }
        break;
    default:
        break;
  }
  //DBStringPrintf(DB_LINE2,"off %2.2lf state %d", offgear,
    //      m_autoState);
}

}
