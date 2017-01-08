#include "RobotInfo.h"
#include "Robot.h"

#include "lib/GreyCompressor.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/SingleThreadTaskMgr.h"
#include "lib/SPIGyro.h"
#include "lib/WrapDash.h"

#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/Hanger.h"

#include "lib/filters/Debouncer.h"

#include "PoseManager.h"

namespace frc973 {

constexpr int ONE_BALL_AUTO = 0;
constexpr int TWO_BALL_AUTO = 1;

Robot::Robot(void
	) :
	CoopMTRobot(),
	JoystickObserver(),
	m_hiFreq(nullptr),
	m_logger(nullptr),
	m_pdp(new PowerDistributionPanel()),
	m_driverJoystick(nullptr),
	m_operatorJoystick(nullptr),
	m_tuningJoystick(nullptr),
#ifdef PROTO_BOT_PINOUT
	m_collinGyro(new Encoder(COLLIN_GYRO_A_DIN, COLLIN_GYRO_B_DIN,
			false, CounterBase::k4X)),
#else
	m_austinGyro(new SPIGyro()),
#endif
	m_leftDriveVictor(nullptr),
	m_rightDriveVictor(nullptr),
	m_leftDriveEncoder(nullptr),
	m_gyroEncoder(nullptr),
	m_drive(nullptr),
	m_sharedConveyorMotor(new VictorSP(SHOOTER_CONVEYER_MOTOR_PWM)),
	m_intake(nullptr),
	m_shooter(nullptr),
	m_shooterStallFilter(new Debouncer(1.5)),
	m_hanger(nullptr),
	m_airPressureSwitch(nullptr),
	m_compressorRelay(nullptr),
	m_compressor(nullptr),
	m_autoState(0),
	m_autoTimer(0),
	m_selectedAutoRoutine(TWO_BALL_AUTO),
	m_battery(nullptr),
	m_time(nullptr),
	m_state(nullptr),
	m_messages(nullptr),
	m_buttonPresses(nullptr),
	m_poseManager(nullptr),
	m_selectedRoutine(AutoRoutine::Go),
	m_selectedDirection(AutoStartPosition::NoVision),
	m_goBack(false),
	m_ballSnatch(false)
{
	printf("Starting robot init\n");fflush(stdout);
	//m_hiFreq = new SingleThreadTaskMgr(*this, 1.0 / 2.0);

	m_driverJoystick = new ObservableJoystick(DRIVER_JOYSTICK_PORT, this, this);
	m_operatorJoystick = new ObservableJoystick(OPERATOR_JOYSTICK_PORT, this, this);
	m_tuningJoystick = new ObservableJoystick(2, this, this);

	fprintf(stderr, "Initialized joysticks\n");

	m_leftDriveVictor = new VictorSP(DRIVE_LEFT_PWM);
	m_rightDriveVictor = new VictorSP(DRIVE_RIGHT_PWM);
	fprintf(stderr, "Initialized drive victors\n");

	m_leftDriveEncoder = new Encoder(LEFT_DRIVE_ENCODER_A_DIN,
			LEFT_DRIVE_ENCODER_B_DIN, false, CounterBase::k4X);
	fprintf(stderr, "Initilalized encoder\n");


	m_logger = new LogSpreadsheet(this);
	fprintf(stderr, "Starting drive init\n");
#ifdef PROTO_BOT_PINOUT
	m_drive = new Drive(this, m_leftDriveVictor, m_rightDriveVictor,
			m_leftDriveEncoder, nullptr, m_collinGyro, m_logger);
#else
	m_drive = new Drive(this, m_leftDriveVictor, m_rightDriveVictor,
			m_leftDriveEncoder, nullptr, m_austinGyro, m_logger);
#endif
	fprintf(stderr, "Finished drive init\n");

	m_intake = new Intake(this);

	fprintf(stderr, "Finished intake init\n");
	m_airPressureSwitch = new DigitalInput(AIR_PRESSURE_DIN);
	m_compressorRelay = new Relay(COMPRESSOR_RELAY, Relay::kForwardOnly);
	m_compressor = new GreyCompressor(m_airPressureSwitch, m_compressorRelay, this);
	fprintf(stderr, "Finished compressor init\n");

	m_battery = new LogCell("Battery voltage");

	m_time = new LogCell("Time (ms)");
	m_state = new LogCell("Game State");
	m_messages = new LogCell("Robot messages", 100, true);
	m_buttonPresses = new LogCell("Button Presses (disabled only)", 100, true);

	m_logger->RegisterCell(m_battery);
	m_logger->RegisterCell(m_time);
	m_logger->RegisterCell(m_buttonPresses);
	fprintf(stderr, "Finished logger init\n");

	m_shooter = new Shooter(this, m_logger, m_sharedConveyorMotor);
	fprintf(stderr, "Finished shooter init\n");

	m_hanger = new Hanger(this, m_drive, m_sharedConveyorMotor, m_shooter);

    m_poseManager = new PoseManager(m_shooter, m_intake);
	fprintf(stderr, "Finished poses init\n");
}

Robot::~Robot(void) {
}

void Robot::Initialize(void) {
	printf("initializing the roboto\n");

	m_compressor->Enable();
	fprintf(stderr, "Finished compressor enable\n");

	m_logger->InitializeTable();
	fprintf(stderr, "Finished logger init\n");

	//SmartDashboard::init();
	fprintf(stderr, "Finished smart-dash\n");

	//m_hiFreq->Start();
	//m_hiFreq->SetHighPriority();

	DBStringPrintf(DBStringPos::DB_LINE0, "shooter disabled");
	DBStringPrintf(DBStringPos::DB_LINE6, "Forward Auto");
	DBStringPrintf(DBStringPos::DB_LINE9, "No vision (still drive tho)");
	fprintf(stderr, "Finished initialization of robot\n");
}

void Robot::AllStateContinuous(void) {
#ifdef PROTO_BOT_PINOUT
	DBStringPrintf(DBStringPos::DB_LINE8, "cgyro %lf", m_collinGyro->GetDistance());
#else
	DBStringPrintf(DBStringPos::DB_LINE8, "agyro %lf", m_austinGyro->GetDegrees());
#endif

	//printf("gyro angle: %lf\n", m_austinGyro->GetDegreesPerSec());
	DBStringPrintf(DBStringPos::DB_LINE5, "dist %lf", m_drive->GetLeftDist());

	//DBStringPrintf(DBStringPos::DB_LINE8, "port 10 cur %lf", m_pdp->GetCurrent(10));

	m_battery->LogPrintf("%f", DriverStation::GetInstance().GetBatteryVoltage());

	m_time->LogDouble(GetSecTime());
	m_state->LogPrintf("%s", GetRobotModeString());

	double backFlywheelCurrent = m_pdp->GetCurrent(BACK_FLYWHEEL_PDB);

	if (m_shooterStallFilter->Update(
			backFlywheelCurrent > BACK_FLYWHEEL_STALL_CURRENT)) {
		m_shooter->SetFlywheelEnabled(false);
		DBStringPrintf(DBStringPos::DB_LINE0, "!STALL DETECTED!");
	}

	DBStringPrintf(DBStringPos::DB_LINE0, "bf %2.2lf",
			backFlywheelCurrent);
}
void Robot::ObserveJoystickStateChange(uint32_t port, uint32_t button,
			bool pressedP) {
	fprintf(stderr, "joystick state change port %d button %d state %d\n", port, button, pressedP);
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
