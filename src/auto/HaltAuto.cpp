/**
 * Dont move
 */

#include "Robot.h"
#include "AutoCommon.h"

namespace frc973 {

void Robot::HaltAuto(void) {
    m_drive->ArcadeDrive(0.0, 0.0);
}

}
