/**
 * This auto routine must start with the hopper away from the wall.
 * Most used in worlds, best auto ever 💯🔥
 */

#include "Robot.h"
#include "AutoCommon.h"
#include "lib/MotionProfile.h"

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
            m_shooter->SetFlywheelSpeed(3060);
            m_shooter->SetKickerRate(3060);
            m_gearIntake->SetPickUpManual();
            m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            m_shooter->StopAgitator();
            m_shooter->StartConveyor(0.0);
            m_drive
                ->SplineDrive(DriveBase::RelativeTo::Now, initial_dist, 0.0)
                ->SetMaxVelAccel(70.0, 70.0)
                ->SetStartEndVel(0.0, 70.0);
            m_autoState++;
            break;
        case 1:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            }
            if (m_drive->OnTarget()) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
                using namespace Profiler;
                /*MotionProfile<FakeFloat<53>, FakeFloat<90>,
                    FakeFloat<70>, FakeFloat<70>,
                    FakeFloat<0>, FakeFloat<60>>(0);*/
                m_drive
                    ->SplineDrive(DriveBase::RelativeTo::SetPoint, 53.0,
                                m_autoDirection * 91.0)
                                ->SetMaxVelAccel(70.0, 70.0)
                                ->SetStartEndVel(70.0, 60.0);
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
                    ->SplineDrive(DriveBase::RelativeTo::Now, -24.0,
                                m_autoDirection * 65.0)
                    ->SetMaxVelAccel(60.0, 38.0)
                    ->SetStartEndVel(0.0, 0.0);
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
                m_autoTimer = GetMsecTime();
                m_autoState++;
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
