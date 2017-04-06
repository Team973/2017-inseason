#include "Robot.h"
#include "AutoCommon.h"
#include "lib/TrapProfile.h"

namespace frc973 {
    
/**
 * This auto routine must start with the hopper away from the wall
 * because we move forward.
 */
void Robot::SpartanHopperAuto(){
    double initial_dist = 48.0;

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
            m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            m_shooter->StopAgitator();
            m_shooter->StartConveyor(0.0);
            m_drive
                ->TrapDrive(DriveBase::RelativeTo::Now, initial_dist, 0.0)
                ->SetHalt(true, false)
                ->SetConstraints(80.0, 48.0);
            m_autoState++;
            break;
        case 1:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            }
            if (m_drive->OnTarget()) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::Now, 3.0 * 12.0,
                                m_autoDirection * -90.0)
                    ->SetHalt(false, false)
                    ->SetConstraints(80.0, 48.0);
                m_autoState++;
            }
            break;
        case 2:
            if (m_drive->OnTarget()) {
                /*
                 * the TrapProfile template is a compile-time hack to make
                 * sure the profile won't mess up at runtime.  We aren't
                 * going to follow the whole thing; we're going to switch to
                 * a regular drive after a few inches.  TrapDrive just makes
                 * absolute certain that we don't have angular momentum
                 * when we transition to that regular arcade drive.  The
                 * arcade drive will press against the hopper.
                 */
                using namespace Profiler;
                TrapProfile<FakeFloat<80>, FakeFloat<0>,
                    FakeFloat<80>, FakeFloat<48>,
                    false, true>(0);
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::Now, 80.0, 0.0)
                    ->SetHalt(false, true)
                    ->SetConstraints(80.0, 48.0);
                m_ballIntake->BallIntakeStart();
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 3:
            /*
             * Advance after 18 inches of trap driving.  We can generalize
             * the trap profiler in offseason to exit on arbitrary velocity
             * but for now we have to make an extra long profile and exit
             * early.
             */
            if (GetMsecTime() - m_autoTimer > 700 &&
                    m_drive->GetTrapDriveController()
                           ->DistFromStart() > 18.0) {
                m_drive->ArcadeDrive(0.1, 0.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 4:
            if (GetMsecTime() - m_autoTimer > 2500) {
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::Now, -24.0,
                                m_autoDirection * 90.0)
                    ->SetHalt(true, true)
                    ->SetConstraints(80.0, 48.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 5:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2000) {
                m_ballIntake->BallIntakeStop();
                double angleOffset = m_boilerPixy->GetXOffset() *
                    BoilerPixy::PIXY_OFFSET_CONSTANT;
                if (Util::abs(angleOffset) >= 10.0) {
                    m_autoState++;
                }
                else {
                    m_drive
                        ->PIDTurn(m_drive->GetAngle() - angleOffset,
                                   DriveBase::RelativeTo::Absolute, 1.0)
                        ->SetDistTolerance(15.0, 25.0)
                        ->SetAngleTolerance(30.0, 60.0);
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                }
            }
            break;
        case 6:
            if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1000 )) {
              m_autoTimer = GetMsecTime();
              m_autoState++;
            }
            break;
        case 7:
              m_ballIntake->RetractHopper();
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
              m_shooter->StartConveyor(0.9);
              m_shooter->StartAgitator(1.0, Shooter::Side::right);
              m_shooter->StartAgitator(1.0, Shooter::Side::left);
              m_autoState++;
            break;
        case 8:
            break;
    }
}

}
