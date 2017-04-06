#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/GearIntake.h"
#include "lib/GreyCompressor.h"
#include "subsystems/BallIntake.h"
#include "controllers/PIDDrive.h"

namespace frc973 {
    static constexpr double DRIVER_STATION_BASE_LINE_DIST = 87.0;
    static constexpr double DRIVER_STATION_LAUNCHPAD_DIST = 185.3;
    static constexpr double KEY_DIST = 52.0;
    static constexpr double SHOOTER_RPM = 2960.0;
    //8.6 feet from side of field to side of airship
}
