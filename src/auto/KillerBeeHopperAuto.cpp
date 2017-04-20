#include "Robot.h"
#include "AutoCommon.h"
#include "lib/TrapProfile.h"

namespace frc973 {

/**
 * This auto routine must start with the hopper away from the wall
 * because we move forward.
 */
int didPixy = 0;
double angle = -99;
double startAngle = 0.0;

void Robot::KillerHopperAuto(){
    double initial_dist = 47.0;

    if(m_alliance == Alliance::Red){
        initial_dist += 0.0;
    }
    else{
        initial_dist += 0.0;
    }

    switch (m_autoState){
        case 0:
            startAngle = m_drive->GetAngle();
            m_compressor->Disable();
            m_ballIntake->ExpandHopper();
            m_shooter->SetFlywheelSpeed(3070);
            m_gearIntake->SetPickUpManual();
            m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            m_shooter->StopAgitator();
            m_shooter->StartConveyor(0.0);
            m_drive
                ->TrapDrive(DriveBase::RelativeTo::Now, initial_dist, 0.0)
                ->SetHalt(true, false)
                ->SetConstraints(70.0, 70.0);
            m_autoState++;
            break;
        case 1:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            }
            if (m_drive->OnTarget()) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
                using namespace Profiler;
                TrapProfile<FakeFloat<53>, FakeFloat<90>,
                    FakeFloat<70>, FakeFloat<70>,
                    false, true>(0);
                m_drive
                    ->TrapDrive(DriveBase::RelativeTo::SetPoint, 53.0,
                                m_autoDirection * 91.0)
                    ->SetHalt(false, true)
                    ->SetConstraints(70.0, 70.0);
                m_autoState++;
            }
            break;
        case 2:
            if (m_drive->OnTarget()) {
                m_ballIntake->BallIntakeStart();
                m_drive->DriveStraight(Drive::RelativeTo::Now, 0.6, 0.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
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
                angle = angleOffset;
                if (Util::abs(angleOffset) >= 10.0) {
                    m_autoState++;
                    didPixy = 1;
                }
                else {
                    m_drive
                        ->PIDTurn(-angleOffset,
                                  DriveBase::RelativeTo::Now, 1.0)
                        ->SetDistTolerance(15.0, 25.0)
                        ->SetAngleTolerance(30.0, 60.0);
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                    didPixy = 2;
                }
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_shooter->StartConveyor(0.9);
                m_shooter->StartAgitator(1.0, Shooter::Side::right);
                m_shooter->StartAgitator(1.0, Shooter::Side::left);
            }
            break;
        case 5:
            if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1000 )) {
              m_ballIntake->RetractHopper();
              m_autoState++;
            }
            break;
        default:
            break;
    }

    DBStringPrintf(DB_LINE2, "auto %d %d %lf",
            m_autoState, didPixy, angle);
}

}
