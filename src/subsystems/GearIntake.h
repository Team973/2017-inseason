#ifndef GEAR_INTAKE_SUBSYSTEM_H
#define GEAR_INTAKE_SUBSYSTEM_H
#include "WPILib.h"
#include "Solenoid.h"
#include "Robot.h"

using namespace frc;

class GearIntake : public CoopTask{
  public:
    enum GearIntakeState{
      released,
      grabbed
    }

    void GearIntake();
    virtual void ~GearIntake();

    void GrabGears();
    void ReleaseGears();

  private:
    GearIntakeState m_gearIntakeState;
}

#endif /*GEAR_INTAKE_SUBSYSTEM_H*/
