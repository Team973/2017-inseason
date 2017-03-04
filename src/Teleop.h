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
static bool g_manualControl = true;
static bool g_manualConveyorControl = true;
static bool g_driveArcade = true;

void Robot::TeleopContinuous(void) {
    double y = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    double x = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis)
        + -m_tuningJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

    if (g_manualControl) {
        m_drive->AssistedArcadeDrive(y, x);
    }
    else if(m_drive->OnTarget()){
      m_lights->NotifyFlash(3);
      g_manualControl = true;
    }
    if (Util::abs(m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis)) > 0.5 ||
        Util::abs(m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis)) > 0.5) {
      g_manualConveyorControl = true;
    }

    if (g_manualConveyorControl){
      double c = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

      m_shooter->StartConveyor(c);

      double l = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
      double r = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightYAxis);

      m_shooter->Shooter::StartAgitator(l, false);
      m_shooter->Shooter::StartAgitator(r, true);
    }

    if (g_hangSignalSent == false && GetMsecTime() - m_teleopTimer > 90000) {
        m_lights->NotifyFlash(10);
    }
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
        bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
              g_driveArcade = false;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
              g_driveArcade = true;
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                //m_hanger->SetHangerState(Hanger::HangerState::autoHang);
            }
            else{
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
              //sw lowgear
              m_gearIntake->SetReleaseManualEnable(true);
            }
            else {
              m_gearIntake->SetReleaseManualEnable(false);
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
                m_gearIntake->SetReleaseAutoEnable(true);
            }
            else{
                m_gearIntake->SetReleaseAutoEnable(false);
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::shooting);
            }
            else{
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::idle);
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
              m_shooter->SetShooterState(Shooter::ShootingSequenceState::targeting);
              g_manualControl = false;
            }
            else{
              g_manualControl = true;
              g_manualConveyorControl = false;
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
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {

                }
            break;
        case DualAction::Back:
            if (pressedP) {
              m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
              m_gearIntake->SetPickUpManual();
            }
            break;
        case DualAction::Start:
            if (pressedP) {
              m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
              m_gearIntake->SetPickUpManual();
            }
            break;
        }
    }
    else if (port == TUNING_JOYSTICK_PORT){
        switch (button) {
            case DualAction::DPadUpVirtBtn:
                if (pressedP) {
                    /*
                    g_manualControl = false;
                    m_drive->SetBoilerPixyTargeting();
                    */
                    m_conveyorSetpt += 0.1;
                }
                break;
            case DualAction::DPadDownVirtBtn:
                if (pressedP) {
                    
                    g_manualControl = false;
                    m_drive->SetGearPixyTargeting();
                    
                   // m_conveyorSetpt -= 0.1;
                }
                break;
            case DualAction::DPadRightVirtBtn:
                if (pressedP) {
                    m_flailSetpt += 0.1;
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
                    m_shooter->StartAgitator(m_flailSetpt, true);
                    m_shooter->StartAgitator(m_flailSetpt, false);
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
                    g_manualControl = false;
                    m_drive->PIDDrive(12 * 8, 0,
                            Drive::RelativeTo::Now, 1.0);
                }
                break;
            case DualAction::BtnB:
                if (pressedP) {
                    g_manualControl = true;
                    m_drive->ArcadeDrive(0.0, 0.0);
                }
                break;
            case DualAction::BtnX:
                if (pressedP) {
                    g_manualControl = false;
                    m_drive->PIDTurn(90,
                            Drive::RelativeTo::Now, 1.0);
                }
                break;
            case DualAction::BtnY:
                if (pressedP) {
                    m_lights->EnableLights();
                    /*
                    g_manualControl = false;
                    m_drive->PIDDrive(120, 0,
                            Drive::RelativeTo::Now, 0.1);
                            */
                }
                else {
                    m_lights->DisableLights();
                }
                break;
            case DualAction::Start:
              if (pressedP) {
                  g_manualControl = false;
                  m_drive->PIDDrive(-12 * 8, 0,
                          Drive::RelativeTo::Now, 1.0);
              }
              break;
        }
    }
}

}
