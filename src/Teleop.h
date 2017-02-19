#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973 {

void Robot::TeleopStart(void) {
    m_teleopTimeSec = GetSecTime();
    m_drive->ArcadeDrive(0.0, 0.0);
}

void Robot::TeleopStop(void) {
}

static bool g_manualControl = true;

void Robot::TeleopContinuous(void) {
    double y = m_driverJoystick->GetRawAxis(DualAction::LeftYAxis);
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

    double c = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);

    m_shooter->StartConveyor(c);

    double l = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    double r = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightYAxis);

    m_shooter->Shooter::StartAgitator(l, false);
    m_shooter->Shooter::StartAgitator(r, true);
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
        bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
                m_hanger->SetHangerState(Hanger::HangerState::armed);
            }
            else{
                m_hanger->SetHangerState(Hanger::HangerState::start);
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::floating);
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
                m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
                //sw lowgear
                m_boilerPixy->Enable();
            }
            else {
                m_boilerPixy->Disable();
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
                m_hanger->SetHangerState(Hanger::HangerState::autoHang);
            }
            else{
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
                m_gearIntake->ReleaseGear();
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
        }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
              m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP){
                m_gearIntake->SetIndexerMode(GearIntake::Indexer::stop);
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
            }
            break;
        case DualAction::BtnA:
            if (pressedP) {
                m_shooter->SetFlywheelSpeed(m_speedSetpt);
                m_compressor->Disable();
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                m_boilerPixy->Enable();
                m_ballIntake->BallIntakeStartReverse();
            }
            else{
                m_boilerPixy->Disable();
                m_ballIntake->BallIntakeStop();
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_shooter->SetFlywheelStop();
                m_compressor->Enable();
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
                m_gearIntake->StartPickupSequence();
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP){
                m_shooter->StartConveyor(m_conveyorSetpt);
            }
            else{
                m_shooter->StopConveyor();
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
        case DualAction::RightTrigger:
            if (pressedP){
                m_shooter->StartAgitator(m_flailSetpt, true);
                m_shooter->StartAgitator(m_flailSetpt, false);
            }
            else{
                m_shooter->StopAgitator();
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
          }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
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
                    m_shooter->StartAgitator(m_flailSetpt, true);
                    m_shooter->StartAgitator(m_flailSetpt, false);
                    m_shooter->StartConveyor(1.0);
                }
                else {
                    m_shooter->StopAgitator();
                    m_shooter->StopConveyor();
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
                    m_conveyorSetpt += 0.1;
                }
                break;
            case DualAction::RightBumper:
                if (pressedP) {
                    printf("Start right bumper things\n");
                    m_shooter->StartAgitator(m_flailSetpt, true);
                    m_shooter->StartAgitator(m_flailSetpt, false);
                    m_shooter->StartConveyor(1.0);
                }
                else {
                    printf("end right bumper things\n");
                    m_shooter->StopAgitator();
                    m_shooter->StopConveyor();
                }
                break;
            case DualAction::LeftBumper:
                if (pressedP) {
                    m_speedSetpt += 50;
                    m_shooter->SetFlywheelSpeed(m_speedSetpt);
                }
                break;
            case DualAction::LeftTrigger:
                if (pressedP) {
                    m_speedSetpt -= 50;
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
