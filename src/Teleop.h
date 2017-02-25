#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973 {

void Robot::TeleopStart(void) {
    m_drive->ArcadeDrive(0.0, 0.0);
}

void Robot::TeleopStop(void) {
}

static bool g_manualControl = true;
static bool g_manualConveyorControl = true;

void Robot::TeleopContinuous(void) {
    double y = -m_driverJoystick->GetRawAxis(DualAction::LeftYAxis);
    double x = -m_driverJoystick->GetRawAxis(DualAction::RightXAxis)
        + -m_tuningJoystick->GetRawAxis(DualAction::RightXAxis);
//  printf("throttle  %lf, turn  %lf\n", y, x);

    if (m_driverJoystick->GetRawButton(DualAction::LeftBumper)) {
        y *= 0.4;
        x *= 0.4;
    }
    if (g_manualControl) {
        m_drive->ArcadeDrive(y, x);
    }

    if (g_manualConveyorControl == false){
      double c = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

      m_shooter->StartConveyor(c);

      double l = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
      double r = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightYAxis);

      m_shooter->Shooter::StartAgitator(l, false);
      m_shooter->Shooter::StartAgitator(r, true);
    }
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
        bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
              m_gearIntake->SetSeeking(true);
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                m_hanger->SetHangerState(Hanger::HangerState::armed);
                //m_hanger->SetHangerState(Hanger::HangerState::autoHang);
            }
            else{
                m_hanger->SetHangerState(Hanger::HangerState::start);
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
                //sw lowgear
                m_gearIntake->ReleaseGear();
            }
            else {
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
            if (pressedP && m_shooter->OnTarget()) {
              g_manualConveyorControl = true;
              m_shooter->StartConveyor(m_conveyorSetpt);
              m_shooter->StartAgitator(m_flailSetpt, true);
              m_shooter->StartAgitator(m_flailSetpt, false);
            }
            else{
              g_manualConveyorControl = false;
              m_shooter->StopConveyor();
              m_shooter->StopAgitator();
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
              m_drive->SetBoilerPixyTargeting();
              if (m_drive->OnTarget() && m_shooter->OnTarget()) {
                g_manualConveyorControl = true;
                m_shooter->StartConveyor(m_conveyorSetpt);
                m_shooter->StartAgitator(m_flailSetpt, true);
                m_shooter->StartAgitator(m_flailSetpt, false);
            }
            else{
              g_manualConveyorControl = false;
              m_shooter->StopConveyor();
              m_shooter->StopAgitator();
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
            if (pressedP) {
            }
            break;
        }
    }
    else if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnY:
            if (pressedP) {
              m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
              }
            break;
        case DualAction::BtnA:
            if (pressedP) {
                m_gearIntake->SetReleaseAutoEnable(true);
            }
            else{
                m_gearIntake->SetReleaseAutoEnable(false);
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
              m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
              m_gearIntake->ReleaseGear();
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

            break;
        case DualAction::Start:

            break;
        }
    }
    else if (port == TUNING_JOYSTICK_PORT){
        switch (button) {
            case DualAction::DPadUpVirtBtn:
                if (pressedP) {
                    g_manualControl = false;
                    m_drive->SetBoilerPixyTargeting();
                }
                break;
            case DualAction::DPadDownVirtBtn:
                if (pressedP) {
                    g_manualControl = false;
                    m_drive->SetGearPixyTargeting();
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
                    //m_conveyorSetpt += 0.1;
                }
                break;
            case DualAction::RightBumper:
                if (pressedP) {
                    g_manualConveyorControl = true;
                    printf("Start right bumper things\n");
                    m_shooter->StartAgitator(m_flailSetpt, true);
                    m_shooter->StartAgitator(m_flailSetpt, false);
                    m_shooter->StartConveyor(0.5);
                }
                else {
                    g_manualConveyorControl = false;
                    printf("end right bumper things\n");
                    m_shooter->StopAgitator();
                    m_shooter->StopConveyor();
                }
                break;
            case DualAction::LeftBumper:
                if (pressedP) {
                    m_speedSetpt += 10;
                    m_shooter->SetFlywheelSpeed(m_speedSetpt);
                }
                break;
            case DualAction::LeftTrigger:
                if (pressedP) {
                    m_speedSetpt -= 10;
                    m_shooter->SetFlywheelSpeed(m_speedSetpt);
                }
                break;
            case DualAction::BtnA:
                if (pressedP) {
                    g_manualControl = false;
                    m_drive->PIDDrive(120, 0,
                            Drive::RelativeTo::Now, 0.5);
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
        }
    }
}

}
