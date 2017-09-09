/**
 * This auto routine must start with the hopper touching the wall.
 * Attempted to do this, failed miserably
 */

#include "Robot.h"
#include "AutoCommon.h"
#include "lib/TrapProfile.h"

namespace frc973 {

void Robot::SpartanHopperAuto(){
    double initial_dist = 6.0;

    if(m_alliance == Alliance::Red){
        initial_dist += 0.0;
    }
    else{
        initial_dist += 0.0;
    }

    switch (m_autoState){
        case 0:
            m_compressor->Disable();
            m_ballIntake->ExpandHopper();
            m_shooter->SetFlywheelSpeed(3030);
            m_gearIntake->SetPickUpManual();
            m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            m_shooter->StopAgitator();
            m_shooter->StartConveyor(0.0);
            m_drive
                ->TrapDrive(DriveBase::RelativeTo::SetPoint, -114.0,
                            m_autoDirection * 45.0)
                ->SetHalt(true, true)
                ->SetConstraints(70.0, 96.0);
            m_autoState++;
            break;
        case 1:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            }
            if (m_drive->OnTarget() || m_drive->GetDist() <= -110.0) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
                using namespace Profiler;
                TrapProfile<FakeFloat<4 * 12>, FakeFloat<90>,
                    FakeFloat<70>, FakeFloat<54>,
                    false, true>(0);
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::SetPoint, 30.0,
                                m_autoDirection * -80.0)
                    ->SetHalt(true, true)
                    ->SetConstraints(130.0, 96.0);
                m_autoState = 90;
            }
            break;
        case 2:
            if (m_drive->OnTarget()) {
                m_drive
                    ->PIDTurn(-60.0 * m_autoDirection,
                              DriveBase::RelativeTo::Now, 1.0);
                m_ballIntake->BallIntakeStart();
                m_autoTimer = GetMsecTime();
                m_autoState = 90;
            }
            break;
        case 3:
            if (GetMsecTime() - m_autoTimer > 2500) {
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::Now, -24.0,
                                m_autoDirection * 65.0)
                    ->SetHalt(true, true)
                    ->SetConstraints(60.0, 38.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 4:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 3000) {
                m_ballIntake->BallIntakeStop();
                double angleOffset = m_boilerPixy->GetXOffset() *
                    BoilerPixy::PIXY_OFFSET_CONSTANT;
                if (Util::abs(angleOffset) >= 10.0) {
                    m_autoState++;
                }
                else {
                    m_drive
                        ->PIDTurn(-angleOffset,
                                  DriveBase::RelativeTo::Now, 1.0)
                        ->SetDistTolerance(15.0, 25.0)
                        ->SetAngleTolerance(30.0, 60.0);
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                }
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_shooter->StartConveyor(0.9);
                m_shooter->StartAgitator(1.0, Shooter::Side::right);
                m_shooter->StartAgitator(1.0, Shooter::Side::left);
            }
            break;
        case 5:
            if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1000 )) {
              m_autoTimer = GetMsecTime();
              m_autoState++;
            }
            break;
        case 6:
              m_ballIntake->RetractHopper();
              m_autoState++;
            break;
        case 7:
            break;
    }
  }
}
