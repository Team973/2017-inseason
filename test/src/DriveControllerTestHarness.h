#pragma once

#include "lib/DriveBase.h"

class DriveControllerTestHarness:
    public frc973::DriveController,
    public frc973::DriveSignalReceiver
{

}
