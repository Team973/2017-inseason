#include "Robot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/GearIntake.h"
#include "lib/GreyCompressor.h"
#include "subsystems/BallIntake.h"
#include "controllers/PIDDrive.h"

using namespace frc;

namespace frc973 {

    static constexpr double DRIVER_STATION_BASE_LINE_DIST = 87.0;
    static constexpr double DRIVER_STATION_LAUNCHPAD_DIST = 185.3;
    static constexpr double KEY_DIST = 52.0;
    static constexpr double SHOOTER_RPM = 2960.0;
    //8.6 feet from side of field to side of airship

    void Robot::AutonomousStart(void) {
        printf("***auto start\n");
        m_drive->Zero();

        m_shooter->SetFlywheelStop();
        m_ballIntake->BallIntakeStop();
        m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
        m_autoTimer = GetMsecTime();

        m_autoState = 0;
    }

    void Robot::AutonomousStop(void) {
        printf("***auto stop\n");
    }

    void Robot::AutonomousContinuous(void) {
        DBStringPrintf(DB_LINE0, "AutoState %d", m_autoState);
        CitrusKpaAndGearAuto();
        /*switch (m_autoRoutine){
            case AutonomousRoutine::MadtownHopperThenShootFuel:
                MadtownHopperThenShoot();
                break;
            case AutonomousRoutine::HopperThenShootFuel:
                HopperThenShoot();
                break;
            case AutonomousRoutine::KpaGearAuto:
                KpaAndGearAuto();
                break;
            case AutonomousRoutine::NoAuto:
                //Don't do any auto
                break;
            case AutonomousRoutine::CitrusKpaGearAuto:
                CitrusKpaAndGearAuto();
                break;

        }*/
    }

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

    void Robot::HopperThenShoot(){
        printf("HopperThenShoot auto\n");
        switch (m_autoState){
            case 0:
                printf("gonna piddrive\n");
                m_compressor->Disable();
                m_ballIntake->ExpandHopper();
                m_shooter->SetFlywheelSpeed(3030);
                m_gearIntake->SetPickUpManual();
                m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
                m_shooter->StopAgitator();
                m_shooter->StartConveyor(0.0);
                if(m_alliance == Alliance::Red){
                  m_drive
                      ->PIDDrive(-63.0, 0.0,
                                 DriveBase::RelativeTo::Now, 0.9)
                      ->SetDistTolerance(2.0, 5.0)
                      ->SetAngleTolerance(9909.0, 9099.0);
                  }
                else{
                  m_drive
                      ->PIDDrive(-64.0, 0.0,
                                 DriveBase::RelativeTo::Now, 0.9)
                      ->SetDistTolerance(2.0, 5.0)
                      ->SetAngleTolerance(9909.0, 9099.0);
                }
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
                    m_ballIntake->BallIntakeStart();
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
                if (GetMsecTime() - m_autoTimer > 2500 ) {
                    m_drive
                        ->PIDDrive(-18.0, 0.0,
                                   DriveBase::RelativeTo::Now, 1.0)
                        ->SetDistTolerance(10.0, 10.0)
                        ->SetAngleTolerance(10.0, 60.0);
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                }
                break;
            case 5:
                if(m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500){
                    m_drive->PIDTurn(-21.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
                    m_ballIntake->BallIntakeStop();
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                }
                break;
            case 6:
                if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1200) {
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
            case 7:
                if (m_drive->OnTarget() || (GetMsecTime() - m_autoTimer >= 1000 )) {
                  m_autoTimer = GetMsecTime();
                  m_autoState++;
                }
                break;
            case 8:
                  m_ballIntake->RetractHopper();
                  m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                  m_shooter->StartConveyor(0.9);
                  m_shooter->StartAgitator(1.0, Shooter::Side::right);
                  m_shooter->StartAgitator(1.0, Shooter::Side::left);
                  m_autoState++;
                break;
            case 9:
                break;
        }
    }

    void Robot::CitrusKpaAndGearAuto(){
      switch(m_autoState){
        case 0:
          m_drive->PIDDrive(-15.0, 0.0, DriveBase::RelativeTo::Now, 1.0);
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
          if(m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2500){
            m_drive->PIDTurn(-67.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
                ->SetAngleTolerance(15.0, 4.0);
            m_autoTimer = GetMsecTime();
            m_autoState++;
          }
          break;
        case 2:
          if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500) {
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
        case 3:
            if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                    GetMsecTime() - m_autoTimer > 1500) {
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_drive->ArcadeDrive(0.0, 0.0);
                m_shooter->StartAgitator(1.0, Shooter::Side::right);
                m_shooter->StartAgitator(1.0, Shooter::Side::left);
                m_shooter->StartConveyor(0.7);
                m_autoTimer = GetMsecTime();
                m_autoState++;
              }
        case 4:
            if(GetMsecTime() - m_autoTimer >= 1500){
              m_drive->PIDTurn(0.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
                  ->SetAngleTolerance(10.0, 3.0);
              m_shooter->StopAgitator();
              m_shooter->StartConveyor(0.0);
              m_shooter->SetFlywheelStop();
              m_autoTimer = GetMsecTime();
              m_autoState++;
            }
            break;
        case 5:
          if(m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500){
            m_drive->PIDDrive(-79.0, 0.0, DriveBase::RelativeTo::SetPoint, 1.0);
            m_autoTimer = GetMsecTime();
            m_compressor->Enable();
            m_autoState++;
          }
          break;
        case 6:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2000) {
              m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
                  ->SetAngleTolerance(10.0, 3.0);
              m_autoTimer = GetMsecTime();
              m_autoState++;
            }
            break;
        case 7:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500) {
                double gearOffset = m_pixyR->GetOffset() *
                    PixyThread::GEAR_MULTIPLIER;

                if (Util::abs(gearOffset) >= 10.0) {
                    m_autoState++;
                }
                else {
                  m_drive
                      ->PIDTurn(m_drive->GetAngle() - gearOffset,
                                 DriveBase::RelativeTo::Absolute, 1.0)
                      ->SetDistTolerance(15.0, 25.0)
                      ->SetAngleTolerance(30.0, 60.0);
                  m_autoTimer = GetMsecTime();
                  m_autoState++;
                }
            }
            break;
        case 8:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500) {
                m_autoTimer = GetMsecTime();
                m_drive->ArcadeDrive(-0.3, 0.0);
                m_autoState++;
            }
            break;
        case 9:
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
        case 10:
            //should be done scoring gear... make hair merry red left
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(-90.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
            }
            break;
        default:
            break;
      }
    }

    void Robot::KpaAndGearAuto(){
      switch(m_autoState){
        case 0:
          m_boilerPixy->Enable();
          m_drive->PIDDrive(-55.5, 0.0, DriveBase::RelativeTo::Now, 1.0);
          m_gearIntake->SetPickUpManual();
          m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
          m_shooter->SetFlywheelSpeed(SHOOTER_RPM);
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
                        ->SetAngleTolerance(0.0, 0.0);
                    m_autoTimer = GetMsecTime();
                    m_autoState++;
                }
              }
            break;
        case 3:
            if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                    GetMsecTime() - m_autoTimer > 3000) {
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
            m_autoState++;
          }
          break;
        case 6:
            if (m_drive->OnTarget()) {
              m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
              m_autoState++;
            }
            break;
        case 7:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 1500) {
                double gearOffset = m_pixyR->GetOffset() *
                    PixyThread::GEAR_MULTIPLIER;

                if (Util::abs(gearOffset) >= 10.0) {
                    m_autoState++;
                }
                else {
                  m_drive
                      ->PIDTurn(m_drive->GetAngle() - gearOffset,
                                 DriveBase::RelativeTo::Absolute, 1.0)
                      ->SetDistTolerance(15.0, 25.0)
                      ->SetAngleTolerance(30.0, 60.0);
                  m_autoTimer = GetMsecTime();
                  m_autoState++;
                }
            }
            break;
        case 8:
            if (m_drive->OnTarget()) {
                m_autoTimer = GetMsecTime();
                m_drive->ArcadeDrive(-0.3, 0.0);
                m_autoState++;
            }
            break;
        case 9:
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
        case 10:
            //should be done scoring gear... make hair merry red left
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(-90.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
            }
            break;
        default:
            break;
      }
    }
}
