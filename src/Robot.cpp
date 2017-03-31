#include "RobotInfo.h"
#include "Robot.h"

#include "WPILib.h"

#include "lib/GreyCompressor.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/WrapDash.h"
#include "subsystems/Drive.h"
#include "subsystems/Hanger.h"
#include "subsystems/BallIntake.h"
#include "subsystems/GearIntake.h"
#include "subsystems/Shooter.h"
#include "subsystems/BoilerPixy.h"
#include "subsystems/Lights.h"

#include "CANTalon.h"

#include "subsystems/PixyThread.h"
using namespace frc;

namespace frc973 {

Robot::Robot(void
    ) :
    CoopMTRobot(),
    JoystickObserver(),
    m_pdp(new PowerDistributionPanel()),
    m_autoDirection(0.0),
    m_autoState(0),
    m_autoTimer(0),
    m_teleopTimer(0),
    m_autoRoutine(AutonomousRoutine::HopperThenShootFuel),
    m_speedSetpt(2900),
    m_flailSetpt(1.0),
    m_alliance(Alliance::Red),
    m_conveyorSetpt(1.0),
    m_driveMode(DriveMode::AssistedArcade),
    m_bumperMode(BumperMode::LowGear)
{
    CameraServer::GetInstance()->StartAutomaticCapture();
    m_driverJoystick = new ObservableJoystick(DRIVER_JOYSTICK_PORT,
            this, this);
    m_operatorJoystick = new ObservableJoystick(OPERATOR_JOYSTICK_PORT,
            this, this);
    m_tuningJoystick = new ObservableJoystick(2, this, this);
    fprintf(stderr, "Joystick Initialized...\n");

    m_leftDriveTalonA = new CANTalon(DRIVE_LEFT_A_CAN);
    m_leftDriveTalonB = new CANTalon(DRIVE_LEFT_B_CAN);
    m_rightDriveTalonA = new CANTalon(DRIVE_RIGHT_A_CAN);
    m_rightDriveTalonB = new CANTalon(DRIVE_RIGHT_B_CAN);

    m_leftDriveTalonA->SetControlMode(
            CANSpeedController::ControlMode::kPercentVbus);
    m_leftDriveTalonA->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_leftDriveTalonA->SetFeedbackDevice(
            CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_leftDriveTalonA->SetClosedLoopOutputDirection(true);
    m_leftDriveTalonA->ConfigNominalOutputVoltage(0, 0);
    //m_leftDriveTalonA->SetNominalClosedLoopVoltage(12.0);
    m_leftDriveTalonA->DisableNominalClosedLoopVoltage();
    m_leftDriveTalonA->ConfigPeakOutputVoltage(12, -12);
    m_leftDriveTalonA->SetSensorDirection(true);
    m_leftDriveTalonA->SelectProfileSlot(0);
    m_leftDriveTalonA->SetP(0.7);
    m_leftDriveTalonA->SetI(0);
    m_leftDriveTalonA->SetD(0.7);
    m_leftDriveTalonA->SetF(0.2);
    m_leftDriveTalonA->EnableCurrentLimit(true);
    m_leftDriveTalonA->SetCurrentLimit(50);
    m_leftDriveTalonA->SetVoltageRampRate(95.0);

    m_leftDriveTalonB->SetControlMode(
            CANSpeedController::ControlMode::kFollower);
    m_leftDriveTalonB->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_leftDriveTalonB->Set(m_leftDriveTalonA->GetDeviceID());
    m_leftDriveTalonB->SetClosedLoopOutputDirection(false);
    //m_leftDriveTalonB->SetNominalClosedLoopVoltage(12.0);
    m_leftDriveTalonB->DisableNominalClosedLoopVoltage();
    m_leftDriveTalonB->ConfigPeakOutputVoltage(12, -12);
    m_leftDriveTalonB->EnableCurrentLimit(true);
    m_leftDriveTalonB->SetCurrentLimit(50);
    m_leftDriveTalonB->SetVoltageRampRate(95.0);

    m_rightDriveTalonA->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_rightDriveTalonA->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_rightDriveTalonA->SetFeedbackDevice(
            CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_rightDriveTalonA->SetClosedLoopOutputDirection(true);
    m_rightDriveTalonA->ConfigNominalOutputVoltage(0, 0);
    //m_rightDriveTalonA->SetNominalClosedLoopVoltage(12.0);
    m_rightDriveTalonA->ConfigPeakOutputVoltage(12, -12);
    m_rightDriveTalonA->DisableNominalClosedLoopVoltage();
    m_rightDriveTalonA->SetSensorDirection(true);
    m_rightDriveTalonA->SelectProfileSlot(0);
    m_rightDriveTalonA->SetP(0.7);
    m_rightDriveTalonA->SetI(0);
    m_rightDriveTalonA->SetD(0.7);
    m_rightDriveTalonA->SetF(0.2);
    m_rightDriveTalonA->EnableCurrentLimit(true);
    m_rightDriveTalonA->SetCurrentLimit(50);
    m_rightDriveTalonA->SetVoltageRampRate(95.0);

    m_rightDriveTalonB->SetControlMode(CANSpeedController::ControlMode::kFollower);
    m_rightDriveTalonB->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_rightDriveTalonB->Set(m_rightDriveTalonA->GetDeviceID());
    m_rightDriveTalonB->ConfigPeakOutputVoltage(12, -12);
    //m_rightDriveTalonB->SetNominalClosedLoopVoltage(12.0);
    m_rightDriveTalonB->DisableNominalClosedLoopVoltage();
    m_rightDriveTalonB->SetClosedLoopOutputDirection(false);
    m_rightDriveTalonB->EnableCurrentLimit(true);
    m_rightDriveTalonB->SetCurrentLimit(50);
    m_rightDriveTalonB->SetVoltageRampRate(95.0);

    m_leftAgitatorTalon = new CANTalon(LEFT_AGITATOR_CAN_ID, 50);
    fprintf(stderr, "Initialized drive controllers\n");

    m_logger = new LogSpreadsheet(this);
    m_time = new LogCell("Time");
    m_logger->RegisterCell(m_time);
    m_driverJoystick->RegisterLog(m_logger);
    m_operatorJoystick->RegisterLog(m_logger);
    m_lights = new Lights(this);
    m_boilerPixy = new BoilerPixy(this, m_lights, m_logger);
    m_pixyR = new PixyThread(*this);
    m_drive = new Drive(this,
            m_leftDriveTalonA, m_rightDriveTalonA, m_leftAgitatorTalon,
            m_logger, m_boilerPixy, m_pixyR);

    m_battery = new LogCell("Battery voltage");
    m_state = new LogCell("Game State");
    m_messages = new LogCell("Robot messages", 100, true);
    m_buttonPresses = new LogCell("Button Presses", 100, true);
    m_xAccel = new LogCell("X acceleration", 32, true);
    m_yAccel = new LogCell("Y acceleration", 32, true);
    m_zAccel = new LogCell("Z acceleration", 32, true);
    m_autoStateLog = new LogCell("Auto state", 32, true);
    m_autoSelectLog = new LogCell("Selected auto routine", 32, true);
    m_boilerOffset = new LogCell("AngleOffset", 32, true);
    m_gearOffset = new LogCell("GearOffset", 32, true);

    m_logger->RegisterCell(m_battery);
    m_logger->RegisterCell(m_state);
    m_logger->RegisterCell(m_messages);
    m_logger->RegisterCell(m_buttonPresses);
    m_logger->RegisterCell(m_xAccel);
    m_logger->RegisterCell(m_yAccel);
    m_logger->RegisterCell(m_zAccel);
    m_logger->RegisterCell(m_autoStateLog);
    m_logger->RegisterCell(m_autoSelectLog);
    m_logger->RegisterCell(m_boilerOffset);
    m_logger->RegisterCell(m_gearOffset);

    m_hanger = new Hanger(this, m_logger);
    m_ballIntake = new BallIntake(this, m_logger);
    m_gearIntake = new GearIntake(this, m_lights, m_logger);
    m_shooter = new Shooter(this, m_logger, m_leftAgitatorTalon, m_drive, m_boilerPixy);

    m_airPressureSwitch = new DigitalInput(AIR_PRESSURE_DIN);
    m_compressorRelay = new Relay(COMPRESSOR_RELAY, Relay::kForwardOnly);
    m_compressor = new GreyCompressor(m_airPressureSwitch, m_compressorRelay, this);

    fprintf(stderr, "initializing aliance\n");
    fprintf(stderr, "done w/ constructor\n");

}

Robot::~Robot(void) {
}

void Robot::Initialize(void) {
    printf("gonna initialize logger\n");
    m_logger->InitializeTable();
    printf("initialized\n");
 }

void Robot::AllStateContinuous(void) {
    m_battery->LogPrintf("%f",
            DriverStation::GetInstance().GetBatteryVoltage());
    m_time->LogDouble(GetSecTime());
    m_state->LogPrintf("%s", GetRobotModeString());

    m_xAccel->LogDouble(m_accel.GetX());
    m_yAccel->LogDouble(m_accel.GetY());
    m_zAccel->LogDouble(m_accel.GetZ());
    m_boilerOffset->LogDouble(m_boilerPixy->GetXOffset() *
        BoilerPixy::PIXY_OFFSET_CONSTANT);
    m_gearOffset->LogDouble(m_pixyR->GetOffset() *
        PixyThread::GEAR_DEGREES_PER_PIXEL);
    m_autoStateLog->LogInt(m_autoState);

    DBStringPrintf(DB_LINE1,
            "angle %.2lf rate %.2lf",
            m_drive->GetAngle(), m_drive->GetAngularRate());
    /*
    DBStringPrintf(DB_LINE2, "drive cur %lf",
                   m_drive->GetDriveCurrent());
                   */
    DBStringPrintf(DB_LINE8,
            "g %d %lf %d",
            m_pixyR->GetDataFresh(), m_pixyR->GetOffset() * PixyThread::GEAR_DEGREES_PER_PIXEL, m_gearIntake->IsGearReady());
}

void Robot::ObserveJoystickStateChange(uint32_t port, uint32_t button,
            bool pressedP) {
    fprintf(stderr, "joystick state change port %d button %d state %d\n",
            port, button, pressedP);
    if (this->IsOperatorControl()){
        HandleTeleopButton(port, button, pressedP);
    }
    else if (this->IsDisabled()){
        HandleDisabledButton(port, button, pressedP);
    }
}

}

START_ROBOT_CLASS(frc973::Robot);
