#include "Robot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/GearIntake.h"
#include "subsystems/Hanger.h"
#include "lib/GreyCompressor.h"
#include "subsystems/BallIntake.h"
#include "controllers/PIDDrive.h"
#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973 {

void Robot::TeleopStart(void) {
    m_drive->ArcadeDrive(0.0, 0.0);
    m_teleopTimer = GetMsecTime();
}

void Robot::TeleopStop(void) {
}

static bool g_hangSignalSent = false;
static bool g_manualDriveControl = true;
static bool g_manualConveyorControl = true;

void Robot::TeleopContinuous(void) {
    double y = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    double x = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis)
        + -m_tuningJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

    if(m_driverJoystick->GetRawButton(DualAction::RightBumper && m_bumperMode == BumperMode::BoilerVision)){
      m_drive->SetBoilerJoystickTerm(y, x);
      m_drive->SetBoilerPixyTargeting();
    }
    else if(g_manualDriveControl){
      if(m_driverJoystick->GetRawButton(DualAction::RightBumper) && m_bumperMode == BumperMode::LowGear){
        x /= 3.0;
        y /= 3.0;
      }

      if(m_driveMode == DriveMode::OpenLoop){
        m_drive->OpenloopArcadeDrive(y, x);
      }
      else if(m_driveMode == DriveMode::AssistedArcade){
        m_drive->AssistedArcadeDrive(y, x);
      }
    }
    /*
    else if(m_drive->OnTarget()){
      m_lights->NotifyFlash(1);
      g_manualDriveControl = true;
    }
    */

    if (Util::abs(m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis)) > 0.5 ||
        Util::abs(m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis)) > 0.5) {
      g_manualConveyorControl = true;
    }

    if (g_manualConveyorControl){
      double c = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

      m_shooter->StartConveyor(c);

      double l = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
      double r = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightYAxis);

      m_shooter->Shooter::StartAgitator(l, Shooter::Side::left);
      m_shooter->Shooter::StartAgitator(r, Shooter::Side::right);
    }

    if (g_hangSignalSent == false && GetMsecTime() - m_teleopTimer > 90000) {
        m_lights->NotifyFlash(10, 250);
        g_hangSignalSent = true;
    }
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
        bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
              m_bumperMode = BumperMode::BoilerVision;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
              m_bumperMode = BumperMode::LowGear;
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
              m_driveMode = DriveMode::OpenLoop;
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
              m_driveMode = DriveMode::AssistedArcade;
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
              //sw lowgear
              m_gearIntake->SetReleaseManualEnable(true);
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
            }
            else {
              m_gearIntake->SetReleaseManualEnable(false);
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
                m_gearIntake->SetReleaseAutoEnable(true);
                m_compressor->Enable();
                m_shooter->SetFlywheelStop();
            }
            else{
                m_gearIntake->SetReleaseAutoEnable(false);
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
              //m_shooter->SetShooterState(Shooter::ShootingSequenceState::shooting);
              m_compressor->Disable();
            }
            else{
              //m_shooter->SetShooterState(Shooter::ShootingSequenceState::idle);
              m_compressor->Enable();
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
              g_manualConveyorControl = false;
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::shooting);
              m_compressor->Disable();
              g_manualDriveControl = false;
            }
            else{
              g_manualDriveControl = true;
              g_manualConveyorControl = false;
              m_compressor->Enable();
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::idle);
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
            }
            else{
            }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP){
              m_buttonPresses->LogPrintf("button down drive %d", 1);
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP){
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {
                }
            break;
        case DualAction::Start:
            if (pressedP) {
            }
            break;
        case DualAction::Back:

            break;
        }
    }
    else if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnY:
            if (pressedP) {
              m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
              m_gearIntake->SetPickUpManual();
            }
            break;
        case DualAction::BtnA:
            if (pressedP) {
              m_hanger->SetHangerState(Hanger::HangerState::armed);
              m_compressor->Disable();
            }
            else{
              m_hanger->SetHangerState(Hanger::HangerState::start);
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
              m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
              m_gearIntake->SetPickUpManual();
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
            }
            else{
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
              m_shooter->SetFlywheelSpeed(m_speedSetpt);
              m_compressor->Disable();
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP){
              m_shooter->SetFlywheelStop();
              m_compressor->Enable();
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
                m_ballIntake->SetIntakePower(-1.0);
            }
            else {
                m_ballIntake->BallIntakeStop();
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
                m_ballIntake->BallIntakeStart();
            }
            else{
                m_ballIntake->BallIntakeStop();
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
              m_gearIntake->SetSeeking(true);
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
            }
            else{
              m_gearIntake->SetSeeking(false);
            }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
              m_buttonPresses->LogPrintf("button down co %d", 1);
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP){
              m_ballIntake->ExpandHopper();
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {
              m_ballIntake->RetractHopper();
                }
            break;
        case DualAction::Back:
            if (pressedP) {
              m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
              m_gearIntake->SetPickUpManual();
            }
            break;
        case DualAction::Start:
            if (pressedP) {
              m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
              m_gearIntake->SetPickUpManual();
              m_compressor->Enable();
              m_shooter->SetFlywheelStop();
            }
            break;
        }
    }
    else if (port == TUNING_JOYSTICK_PORT){
        switch (button) {
            case DualAction::DPadUpVirtBtn:
                if (pressedP) {
                    /*
                    g_manualDriveControl = false;
                    m_drive->SetBoilerPixyTargeting();
                    */
                    m_conveyorSetpt += 0.1;
                }
                break;
            case DualAction::DPadDownVirtBtn:
                if (pressedP) {

                    g_manualDriveControl = false;
                    m_drive->SetGearPixyTargeting();

                   // m_conveyorSetpt -= 0.1;
                }
                break;
            case DualAction::DPadRightVirtBtn:
                if (pressedP) {
                  g_manualDriveControl = false;
                  m_drive
                      ->PIDTurn(m_drive->GetAngle() + m_boilerPixy->GetXOffset() * BoilerPixy::PIXY_OFFSET_CONSTANT,
                                 DriveBase::RelativeTo::Absolute, 1.0)
                      ->SetDistTolerance(15.0, 25.0)
                      ->SetAngleTolerance(30.0, 60.0);
                }
                break;
            case DualAction::DPadLeftVirtBtn:
                if (pressedP) {
                    m_flailSetpt -= 0.1;
                }
                break;
            case DualAction::RightTrigger:
                if (pressedP) {
                  m_compressor->Enable();
                    //m_conveyorSetpt += 0.1;
                }
                break;
            case DualAction::RightBumper:
                if (pressedP) {
                    m_shooter->SetShooterState(Shooter::ShootingSequenceState::manual);
                    g_manualConveyorControl = false;
                    printf("Start right bumper things\n");
                    m_shooter->StartAgitator(m_flailSetpt, Shooter::Side::right);
                    m_shooter->StartAgitator(m_flailSetpt, Shooter::Side::left);
                    m_shooter->StartConveyor(m_conveyorSetpt);
                }
                else {
                    g_manualConveyorControl = true;
                    printf("end right bumper things\n");
                    m_shooter->StopAgitator();
                    m_shooter->StopConveyor();
                }
                break;
            case DualAction::LeftBumper:
                if (pressedP) {
                    m_speedSetpt += 10;
                    m_shooter->SetFlywheelSpeed(m_speedSetpt);
                    m_compressor->Disable();
                }
                break;
            case DualAction::LeftTrigger:
                if (pressedP) {
                    m_speedSetpt -= 10;
                    m_shooter->SetFlywheelSpeed(m_speedSetpt);
                    m_compressor->Disable();
                }
                break;
            case DualAction::BtnA:
                if (pressedP) {
                    g_manualDriveControl = false;
                    m_drive->PIDDrive(12 * 2, 0,
                            Drive::RelativeTo::Now, 1.0);
                }
                break;
            case DualAction::BtnB:
                if (pressedP) {
                    g_manualDriveControl = true;
                    m_drive->ArcadeDrive(0.0, 0.0);
                }
                break;
            case DualAction::BtnX:
                if (pressedP) {
                    g_manualDriveControl = false;
                    m_drive->PIDDrive(0.0, 45,
                            Drive::RelativeTo::Now, 1.0);
                }
                break;
            case DualAction::BtnY:
                if (pressedP) {
                    g_manualDriveControl = false;
                    m_lights->EnableLights();
                    /*
                    g_manualDriveControl = false;
                    m_drive->PIDDrive(120, 0,
                            Drive::RelativeTo::Now, 0.1);
                            */
                }
                else {
                    g_manualDriveControl = true;
                    m_lights->DisableLights();
                }
                break;
            case DualAction::Start:
              if (pressedP) {
                  g_manualDriveControl = false;
                  m_drive->PIDDrive(-12 * 2, 0,
                          Drive::RelativeTo::Now, 1.0);
              }
              break;
        }
    }
}

}
