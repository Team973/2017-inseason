#pragma once

#include "lib/CoopMTRobot.h"
#include "lib/JoystickHelper.h"
#include "RobotInfo.h"
#include "stdio.h"
#include "lib/WrapDash.h"

using namespace frc;
#include "WPILib.h"
#include "CANTalon.h"


namespace frc973 {

class LogSpreadsheet;
class Drive;
class GearIntake;
class Shooter;
class GreyCompressor;
class LogCell;
class PoseManager;
class Debouncer;
class Hanger;
class PixyThread;
class BallIntake;
class GreyCompressor;
class BoilerPixy;
class Lights;
class SPIGyro;

class Robot:
        public CoopMTRobot,
        public JoystickObserver
{
private:
    enum AutonomousRoutine {
        HopperThenShootFuel,
        MadtownHopperThenShootFuel,
        KpaGearAuto,
        NoAuto,
        CitrusKpaGearAuto,
        CitrusHopper,
        SpartanHopper,
        KillerHopper
    };

    const char *GetAutoName(AutonomousRoutine routine);

    enum Alliance{
      Red,
      Blue
    };

    enum DriveMode{
      OpenLoop,
      AssistedArcade,
      PixyDrive
    };

    enum BumperMode{
      BoilerVision,
      LowGear
    };

    LogSpreadsheet *m_logger;

    PowerDistributionPanel *m_pdp;

    /**
     * Inputs (joysticks, sensors, etc...)
     */
    ObservableJoystick		*m_driverJoystick;
    ObservableJoystick		*m_operatorJoystick;
    ObservableJoystick		*m_tuningJoystick;

    /**
     * Outputs (motors, solenoids, etc...)
     */
    CANTalon		*m_leftDriveTalonA;
    CANTalon		*m_leftDriveTalonB;
    CANTalon		*m_rightDriveTalonA;
    CANTalon		*m_rightDriveTalonB;
    CANTalon        *m_leftAgitatorTalon;
    ADXRS450_Gyro        *m_austinGyro;
    Drive			*m_drive;

    /**
     * Subsystems
     */
    Hanger			*m_hanger;
    BallIntake			*m_ballIntake;
    Shooter			*m_shooter;
    Lights      *m_lights;
    BoilerPixy      *m_boilerPixy;
    GearIntake	*m_gearIntake;
    /*
     * Compressor
     */
    DigitalInput	*m_airPressureSwitch;
    Relay			*m_compressorRelay;
    GreyCompressor  *m_compressor;

    /**
     * Auto
     */
    double 						m_autoDirection;
    int 							m_autoState;
    uint32_t 					m_autoTimer;
    uint32_t 					m_teleopTimer;
    AutonomousRoutine m_autoRoutine;
    int						    m_speedSetpt;
    double						m_flailSetpt;
    Alliance          m_alliance;
    double						m_conveyorSetpt;
    int               m_kickerSetpt;
    DriveMode         m_driveMode;
    BumperMode        m_bumperMode;

    /**
     * Logging
     */
    BuiltInAccelerometer m_accel;//for testing the logger only

    LogCell *m_battery;
    LogCell *m_time;
    LogCell *m_state;
    LogCell *m_messages;
    LogCell *m_buttonPresses;
    LogCell *m_xAccel;
    LogCell *m_yAccel;
    LogCell *m_zAccel;
    LogCell *m_autoStateLog;
    LogCell *m_autoSelectLog;
    LogCell *m_boilerOffset;
    LogCell *m_gearOffset;
    LogCell *m_austinGyroLog;
    LogCell *m_austinGyroRateLog;

    PixyThread *m_pixyR;
public:
    /**
     * Defined in Robot.cpp
     */
    Robot(void);
    ~Robot(void);
    void Initialize(void) override;

    /**
     * Defined in Disabled.h
     */
    void DisabledStart(void) override;
    void DisabledStop(void) override;
    void DisabledContinuous(void) override;

    /**
     * Defined in Autonomous.h
     */
    void AutonomousStart(void) override;
    void AutonomousStop(void) override;
    void AutonomousContinuous(void) override;

    void HopperThenShoot(void);
    void MadtownHopperThenShoot(void);
    void KpaAndGearAuto(void);
    void HaltAuto(void);
    void CitrusKpaAndGearAuto(void);
    void CitrusHopperAuto(void);
    void SpartanHopperAuto(void);
    void KillerHopperAuto(void);
    /**
     * Defined in Teleop.h
     */
    void TeleopStart(void) override;
    void TeleopStop(void) override;
    void TeleopContinuous(void) override;

    /**
     * Function called by the observable joystick whenever a joystick
     * button is pressed or released
     */
    void ObserveJoystickStateChange(uint32_t port, uint32_t button,
            bool newState) override;
    void HandleTeleopButton(uint32_t port, uint32_t button,
            bool newState);
    void HandleDisabledButton(uint32_t port, uint32_t button,
            bool newState);

    /**
     * Defined in Test.h
     */
    void TestStart(void) override;
    void TestStop(void) override;
    void TestContinuous(void) override;

    /**
     * Defined in Robot.cpp
     */
    void AllStateContinuous(void) override;

    void PrintState();
};

}
