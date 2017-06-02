#include "Robot.h"
#include "AutoCommon.h"

namespace frc973 {

void Robot::MadtownHopperThenShoot(){
    printf("MadtownHopperThenShoot auto\n");
    switch (m_autoState){
        case 0:
            if (GetMsecTime() - m_autoTimer <= 2000) {
                /* Two second wait at beginning */
                break;
            }
            printf("gonna piddrive\n");
            m_compressor->Disable();
            m_shooter->SetFlywheelSpeed(3030);
            m_drive
                ->PIDDrive(-(DRIVER_STATION_BASE_LINE_DIST - 18.0) - 4.0, 0.0,
                           DriveBase::RelativeTo::Now, 0.9)
                ->SetDistTolerance(4.0, 9000.0)
                ->SetAngleTolerance(9909.0, 9099.0);
            m_gearIntake->SetPickUpManual();
            m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            m_shooter->StopAgitator();
            m_shooter->StartConveyor(0.0);
            printf("piddrived\n");
            m_autoState++;
            break;
        case 1:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
            }
            printf("waiting for pid on target\n");
            if (m_drive->OnTarget()) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
                printf("pid on target moving on\n");
                m_drive
                    ->PIDTurn(-90.0 * m_autoDirection,
                              DriveBase::RelativeTo::SetPoint, 1.0)
                    ->SetDistTolerance(10.0, 90.0)
                    ->SetAngleTolerance(5.0, 10.0);
                m_autoState++;
            }
            break;
        case 2:
            if (m_drive->OnTarget()) {
                m_drive->ArcadeDrive(0.6, 0.0);
                m_autoTimer = GetMsecTime();
                /*
                m_drive->PIDDrive(26.0, 0.0,
                        DriveBase::RelativeTo::SetPoint, 0.6);
                        */
                m_autoState++;
            }
            break;
        case 3:
            if (GetMsecTime() - m_autoTimer > 700 &&
                    m_drive->GetDriveCurrent() > 18.0) {
                m_drive->ArcadeDrive(0.1, 0.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 4:
            if (GetMsecTime() - m_autoTimer > 2500) {
                m_drive
                    ->PIDDrive(-18.0, 0.0,
                               DriveBase::RelativeTo::Now, 1.0)
                    ->SetDistTolerance(10.0, 10.0)
                    ->SetAngleTolerance(10.0, 60.0);
                m_autoState++;
            }
            break;
        case 5:
            if (m_drive->OnTarget()) {
                m_drive
                    ->PIDTurn(-21.0 * m_autoDirection,
                               DriveBase::RelativeTo::Absolute, 1.0)
                    ->SetDistTolerance(15.0, 25.0)
                    ->SetAngleTolerance(30.0, 60.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 6:
            if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1500)) {
              //  m_drive->PIDDrive(0.0, 0.0 * m_autoDirection,
                //        DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
            }
            break;
        case 7:
            if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1500)) {
                //m_drive->SetBoilerPixyTargeting();
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 8:
            if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                    GetMsecTime() - m_autoTimer > 4000) {
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_drive->ArcadeDrive(0.0, 0.0);
                m_shooter->StartAgitator(1.0, Shooter::Side::right);
                m_shooter->StartAgitator(1.0, Shooter::Side::left);
                m_shooter->StartConveyor(0.9);
                m_autoState++;
            }
            break;
        case 9:
            break;
    }
}

}
