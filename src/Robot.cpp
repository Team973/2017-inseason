#include "RobotInfo.h"
#include "Robot.h"

#include "WPILib.h"

#include "lib/GreyCompressor.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/WrapDash.h"
#include "lib/SingleThreadTaskMgr.h"
#include "subsystems/GearIntake.h"
#include "subsystems/BallIntake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/Hanger.h"

#include "CANTalon.h"

#include "subsystems/PixyThread.h"
using namespace frc;

namespace frc973 {

Robot::Robot(void
	) :
	CoopMTRobot(),
	JoystickObserver(),
	m_pdp(new PowerDistributionPanel()),
    m_spiGyro(nullptr), //new SPIGyro()),
	m_autoDirection(0.0),
	m_autoState(0),
	m_autoRoutine(AutonomousRoutine::NoAuto),
	m_autoTimer(0),
	m_speedSetpt(2000)
{
    SingleThreadTaskMgr *sepTask =
        new SingleThreadTaskMgr(*this, 1.0 / 50.0);
	m_driverJoystick = new ObservableJoystick(DRIVER_JOYSTICK_PORT, this, this);
	m_operatorJoystick = new ObservableJoystick(OPERATOR_JOYSTICK_PORT, this, this);
	m_tuningJoystick = new ObservableJoystick(2, this, this);
	fprintf(stderr, "Joystick Initialized...\n");

	m_leftDriveTalonA = new CANTalon(DRIVE_LEFT_A_CAN);
	m_leftDriveTalonB = new CANTalon(DRIVE_LEFT_B_CAN);
	m_rightDriveTalonA = new CANTalon(DRIVE_RIGHT_A_CAN);
	m_rightDriveTalonB = new CANTalon(DRIVE_RIGHT_B_CAN);

    m_leftDriveTalonA->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
	m_leftDriveTalonA->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_leftDriveTalonB->SetControlMode(CANSpeedController::ControlMode::kFollower);
    m_leftDriveTalonB->Set(m_leftDriveTalonA->GetDeviceID());

    m_rightDriveTalonA->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
	m_rightDriveTalonA->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_rightDriveTalonB->SetControlMode(CANSpeedController::ControlMode::kFollower);
    m_rightDriveTalonB->Set(m_rightDriveTalonA->GetDeviceID());

    m_leftAgitatorTalon = new CANTalon(LEFT_AGITATOR_CAN_ID, 50);
	fprintf(stderr, "Initialized drive controllers\n");

	m_logger = new LogSpreadsheet(this);
	m_drive = new Drive(this,
            m_leftDriveTalonA, m_rightDriveTalonA, m_leftAgitatorTalon, m_logger);

	m_battery = new LogCell("Battery voltage");

	m_time = new LogCell("Time (ms)");
	m_state = new LogCell("Game State");
	m_messages = new LogCell("Robot messages", 100, true);
	m_buttonPresses = new LogCell("Button Presses (disabled only)", 100, true);

	m_logger->RegisterCell(m_battery);
	m_logger->RegisterCell(m_time);
	m_logger->RegisterCell(m_buttonPresses);

	m_shooter = new Shooter(this, m_logger, m_leftAgitatorTalon);
	m_hanger = new Hanger(this);
	m_ballIntake = new BallIntake(this);
	m_gearIntake = new GearIntake(this);

	m_airPressureSwitch = new DigitalInput(AIR_PRESSURE_DIN);
	m_compressorRelay = new Relay(COMPRESSOR_RELAY, Relay::kForwardOnly);
	m_compressor = new GreyCompressor(m_airPressureSwitch, m_compressorRelay, this);
    sepTask->Start();

    fprintf(stderr, "initializing aliance\n");
	if(DriverStation::GetInstance().GetAlliance() == DriverStation::Alliance::kRed){
		m_autoDirection = 1.0;
	}
	else{
		m_autoDirection = -1.0;
	}
    fprintf(stderr, "done w/ constructor\n");

    SingleThreadTaskMgr *pixyThread = new SingleThreadTaskMgr(*this, 0.02, false);
    m_pixyR = new PixyThread(pixyThread);
    pixyThread->Start();
}

Robot::~Robot(void) {
}

void Robot::Initialize(void) {
    printf("gonna initialize logger\n");
	m_logger->InitializeTable();
    printf("initialized\n");
}

void Robot::AllStateContinuous(void) {
	m_battery->LogPrintf("%f", DriverStation::GetInstance().GetBatteryVoltage());
	m_time->LogDouble(GetSecTime());
	m_state->LogPrintf("%s", GetRobotModeString());

    DBStringPrintf(DB_LINE7, 
                   "pixy o%2.2lf %d",
                   m_pixyR->GetOffset(),
                   m_pixyR->GetDataFresh());
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

#include "Disabled.h"
#include "Autonomous.h"
#include "Teleop.h"
#include "Test.h"

START_ROBOT_CLASS(frc973::Robot);
