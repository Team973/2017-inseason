#include "Robot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/GearIntake.h"
#include "lib/GreyCompressor.h"
#include "subsystems/BallIntake.h"
#include "controllers/PIDDrive.h"

using namespace frc;

namespace frc973 {
    const char *Robot::GetAutoName(AutonomousRoutine routine) {
        switch (routine) {
            case AutonomousRoutine::HopperThenShootFuel:
                return "HopperThenShoot";
            case AutonomousRoutine::MadtownHopperThenShootFuel:
                return "MadtownHopper";
            case AutonomousRoutine::KpaGearAuto:
                return "KpaGear";
            case AutonomousRoutine::NoAuto:
                return "No auto";
            case AutonomousRoutine::CitrusKpaGearAuto:
                return "CitrusKpaGear";
            case AutonomousRoutine::CitrusHopper:
                return "CitrusHopper";
            case AutonomousRoutine::SpartanHopper:
                return "SpartanHopper";
            case AutonomousRoutine::KillerHopper:
                return "KillerHopper";
            case AutonomousRoutine::MidPegKpa:
                return "MidPegKpa";
            default:
                return "Error. RESTART!";
        }
    }

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
        switch (m_autoRoutine){
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
                HaltAuto();
                break;
            case AutonomousRoutine::CitrusKpaGearAuto:
                CitrusKpaAndGearAuto();
                break;
            case AutonomousRoutine::CitrusHopper:
                ModifiedCitrusHopperAuto();
                break;
            case AutonomousRoutine::SpartanHopper:
                SpartanHopperAuto();
                break;
            case AutonomousRoutine::KillerHopper:
                KillerHopperAuto();
                break;
            case AutonomousRoutine::MidPegKpa:
                MidPegKpaAuto();
                break;
        }
    }
}
