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

    void Robot::AutonomousStart(void) {
        printf("***auto start\n");
        m_drive->Zero();

        m_shooter->SetFlywheelStop();
        m_ballIntake->BallIntakeStop();
        m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);

        m_autoState = 0;
    }

    void Robot::AutonomousStop(void) {
        printf("***auto stop\n");
    }

    void Robot::AutonomousContinuous(void) {
        DBStringPrintf(DB_LINE0, "AutoState %d", m_autoState);
        switch (m_autoRoutine){
            case AutonomousRoutine::GearLeftPeg:
                GearLtPeg();
                break;
            case AutonomousRoutine::GearMiddlePeg:
                GearMidPeg();
                break;
            case AutonomousRoutine::GearRightPeg:
                GearRtPeg();
                break;
            case AutonomousRoutine::FuelBallToBoiler:
                FuelToBoiler();
                break;
            case AutonomousRoutine::HopperThenShootFuel:
                HopperThenShoot();
                break;
            case AutonomousRoutine::ShootFuelThenHopper:
                ShootThenHopper();
                break;
            case AutonomousRoutine::KpaGearAuto:
                KpaAndGearAuto();
                break;
            case AutonomousRoutine::AimedAtBoilerAuto:
                AimedBoilerAuto();
                break;
            case AutonomousRoutine::CitrusKpaGearAuto:
                CitrusKpaAndGearAuto();
                break;
            case AutonomousRoutine::NoAuto:
                //Don't do any auto
                break;
        }
    }

    void Robot::GearLtPeg(){
      //start at left side
      switch (m_autoState) {
          case 0:
              m_drive->PIDDrive(-72.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
              m_autoState++;
              break;
          case 1:
              if (m_drive->OnTarget()){
                m_drive->PIDDrive(0.0, -60.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
              }
              break;
          case 2:
              if (m_drive->OnTarget()) {
                  m_drive->SetGearPixyTargeting();
                  m_autoState++;
              }
              break;
          case 3:
              if (m_drive->OnTarget()) {
                  m_autoTimer = GetMsecTime();
                  m_drive->ArcadeDrive(-0.3, 0.0);
                  m_autoState++;
              }
              break;
          case 4:
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
                  m_autoState = 2;
              }
              break;
          //READY FOR teleop
          case 5:
              if (m_drive->OnTarget()) {
                  m_drive->PIDDrive(0.0, -30.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                  m_autoState++;
              }
              break;
          default:
              break;
      }
    }

    void Robot::GearMidPeg(){
        //Start facing the wall
        switch (m_autoState) {
            case 0:
                m_drive->PIDDrive(-30.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
                m_autoState++;
                break;
            case 1:
                if (m_drive->OnTarget()) {
                    m_drive->SetGearPixyTargeting();
                    m_autoState++;
                }
                break;
            case 2:
                if (m_drive->OnTarget()) {
                    m_autoTimer = GetMsecTime();
                    m_drive->ArcadeDrive(-0.3, 0.0);
                    m_autoState++;
                }
                break;
            case 3:
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
                    m_autoState = 1;
                }
                break;
            case 4:
                //should be done scoring gear... make hair merry red left
                if (m_drive->OnTarget()) {
                    m_drive->PIDDrive(0.0, 90.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                    m_autoState++;
                }
                break;
            default:
                break;
        }
    }

    void Robot::GearRtPeg(){
      switch (m_autoState) {
          case 0:
              m_drive->PIDDrive(-72.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
              m_autoState++;
              break;
          case 1:
              if (m_drive->OnTarget()){
                m_drive->PIDDrive(0.0, 60.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
              }
              break;
          case 2:
              if (m_drive->OnTarget()) {
                  m_drive->SetGearPixyTargeting();
                  m_autoState++;
              }
              break;
          case 3:
              if (m_drive->OnTarget()) {
                  m_autoTimer = GetMsecTime();
                  m_drive->ArcadeDrive(-0.3, 0.0);
                  m_autoState++;
              }
              break;
          case 4:
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
                  m_autoState = 2;
              }
              break;
          case 5:
              //ready for teleop
              if (m_drive->OnTarget()) {
                  m_drive->PIDDrive(0.0, 32.0 * m_autoDirection, DriveBase::RelativeTo::Now, 0.8);
                  m_autoState++;
              }
              break;
          default:
            break;
        }
    }

    void Robot::FuelToBoiler(){
        switch (m_autoState){
            case 0:
                m_drive->PIDDrive(87.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 1:
                if (m_drive->OnTarget()) {
                  m_drive->PIDDrive(0.0, 90.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                  m_autoState++;
                  }
                break;
            case 2:
                if (m_drive->OnTarget()) {
                  m_drive->PIDDrive(26.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.4);
                  m_autoState++;
                }
                break;
            case 3:
                if (m_drive->OnTarget()) {
                    m_drive->PIDDrive(-6.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.4);
                    m_autoState++;
                  }
                break;
            case 4:
                if (m_drive->OnTarget()) {
                    m_drive->PIDDrive(0.0, 180.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                    m_autoState++;
                  }
                break;
            case 5:
                if (m_drive->OnTarget()) {
                    m_drive->PIDDrive(0.0, 67.0 * m_autoDirection, DriveBase::RelativeTo::SetPoint, 0.8);
                    m_autoState++;
                  }
                break;
            case 6:
                if (m_drive->OnTarget()) {
                    m_autoTimer = GetMsecTime();
                    m_shooter->SetFlywheelPow(1.0);
                    //boiler-pixy cam aim X-AXIS
                    //adjust RPM thorugh Y-AXIS
                    m_autoState++;
                  }
                break;
            case 7:
                if (m_autoTimer - GetMsecTime() >= 2000){
                    m_shooter->StartConveyor(1.0);
                    m_shooter->StartAgitator(1.0, true);
                    m_shooter->StartAgitator(1.0, false);
                  }
                break;
        }
    }

    void Robot::HopperThenShoot(){
        printf("HopperThenShoot auto\n");
        switch (m_autoState){
            case 0:
                printf("gonna piddrive\n");
                m_compressor->Disable();
                m_shooter->SetFlywheelSpeed(3030);
                m_drive
                    ->PIDDrive(-(DRIVER_STATION_BASE_LINE_DIST - 18.0) - 4.0, 0.0,
                               DriveBase::RelativeTo::Now, 0.9)
                    ->SetDistTolerance(4.0, 900.0)
                    ->SetAngleTolerance(999.0, 999.0);
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
                if (GetMsecTime() - m_autoTimer > 3000) {
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
                    m_shooter->StartAgitator(1.0, true);
                    m_shooter->StartAgitator(1.0, false);
                    m_shooter->StartConveyor(1.0);
                    m_autoState++;
                }
                break;
            case 9:
                break;
        }
    }

    void Robot::ShootThenHopper(){
        switch (m_autoState){
            case 0:
                m_drive->PIDDrive(0.0, -90.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 1:
                m_drive->PIDDrive(40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 2:
                m_shooter->SetFlywheelPow(1.0);
                m_autoState++;
                break;
            case 3:
                m_ballIntake->BallIntakeStart();
                m_autoState++;
                break;
            case 4:
                m_drive->PIDDrive(0.0, 90.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 5:
                m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 6:
                m_drive->PIDDrive(0.0, 180.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
                break;
            case 7:
                m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
                m_autoState++;
            case 8:
                m_shooter->SetFlywheelPow(1.0);
                m_autoState++;
                break;
            case 9:
                m_ballIntake->BallIntakeStart();
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
          if(m_drive->OnTarget()){
            m_drive->PIDTurn(-65.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0)
                ->SetAngleTolerance(8.0, 4.0);
            m_autoState++;
          }
          break;
        case 2:
          if (m_drive->OnTarget()) {
             // m_drive->SetBoilerPixyTargeting();
              m_autoTimer = GetMsecTime();
              m_autoState++;
          }
          break;
        case 3:
            if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                    GetMsecTime() - m_autoTimer > 1500) {
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_drive->ArcadeDrive(0.0, 0.0);
                m_shooter->StartAgitator(1.0, true);
                m_shooter->StartAgitator(1.0, false);
                m_shooter->StartConveyor(0.7);
                m_autoTimer = GetMsecTime();
                m_autoState++;
              }
        case 4:
            if(GetMsecTime() - m_autoTimer >= 1500){
              m_drive->PIDTurn(0.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
              m_shooter->StopAgitator();
              m_shooter->StartConveyor(0.0);
              m_shooter->SetFlywheelStop();
              m_autoState++;
            }
            break;
        case 5:
          if(m_drive->OnTarget()){
            m_drive->PIDDrive(-76.0, 0.0, DriveBase::RelativeTo::SetPoint, 1.0);
            m_compressor->Enable();
            m_autoState++;
          }
          break;
        case 6:
            if (m_drive->OnTarget()) {
              m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
           // m_drive->SetGearPixyTargeting();
              m_autoState++;
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
    }

    void Robot::KpaAndGearAuto(){
      switch(m_autoState){
        case 0:
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
              m_drive->PIDTurn(-31.5 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
              m_autoTimer = GetMsecTime();
              m_autoState++;
          }
          break;
        case 2:
            if (m_drive->OnTarget() || GetMsecTime() - m_autoTimer >= 2000) {
                // m_drive->SetBoilerPixyTargeting();
                 m_autoTimer = GetMsecTime();
                 m_autoState++;
            }
            break;
        case 3:
            if ((m_drive->OnTarget() && m_shooter->OnTarget()) ||
                    GetMsecTime() - m_autoTimer > 3000) {
                m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                m_drive->ArcadeDrive(0.0, 0.0);
                m_shooter->StartAgitator(1.0, true);
                m_shooter->StartAgitator(1.0, false);
                m_shooter->StartConveyor(1.0);
                m_autoTimer = GetMsecTime();
                m_autoState++;
            }
            break;
        case 4:
            if(GetMsecTime() - m_autoTimer >= 2500){
              m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
              m_shooter->StopAgitator();
              m_shooter->StartConveyor(0.0);
              m_autoState++;
            }
            break;
        case 5:
          if(m_drive->OnTarget()){
            m_drive->PIDDrive(-50.0, 0.0, DriveBase::RelativeTo::Now, 1.0);
            m_autoState++;
          }
          break;
        case 6:
            if (m_drive->OnTarget()) {
              m_drive->PIDTurn(-60.0 * m_autoDirection, DriveBase::RelativeTo::Absolute, 1.0);
           // m_drive->SetGearPixyTargeting();
              m_autoState++;
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
    }

    void Robot::AimedBoilerAuto(){
      switch (m_autoState) {
        case 0:
          m_shooter->SetFlywheelSpeed(SHOOTER_RPM);
          m_autoState++;
          break;
        case 1:
          if(m_shooter->OnTarget() || GetMsecTime() - m_autoTimer >= 3000){
            m_drive->ArcadeDrive(0.0, 0.0);
            m_shooter->StartAgitator(1.0, true);
            m_shooter->StartAgitator(1.0, false);
            m_shooter->StartConveyor(1.0);
            m_autoTimer = GetMsecTime();
            m_autoState++;
          }
          break;
        case 2:

        default:
          break;
      }
    }
}
