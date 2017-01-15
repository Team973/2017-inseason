using namespace frc;

namespace frc973 {

	void Robot::AutonomousStart(void) {
		printf("***auto start\n");

    m_shooter->SetFlywheelStop();
		m_ballIntake->BallIntakeStop();
		m_gearIntake->GrabGears();
		m_drive->Zero();

		m_autoState = 0;
	}

	void Robot::AutonomousStop(void) {
		printf("***auto stop\n");
	}

	void Robot::AutonomousContinuous(void) {
		switch (m_autoRoutine){
			case AutonomousRoutine::GearLefttPeg:
				GearLtPeg();
				break;
			case AutonomousRoutine::GearMiddlePeg:
				GearMidPeg();
				break;
			case AutonomousRoutine::GearRightPeg:
				GearRtPeg();
				break;
			case AutonomousRoutine::FuelBallToBoiler:
				FuelToBoiler();
				break;
			case AutonomousRoutine::HopperThenShootFuel:
				HopperThenShoot();
				break;
			case AutonomousRoutine::ShootFuelThenHopper:
				ShootThenHopper();
				break;
			case AutonomousRoutine::NoAuto:
				//Don't do any auto
				break;
		}
	}

	void Robot::GearLtPeg(){

	}

	void Robot::GearMidPeg(){

	}

	void Robot::GearRtPeg(){

	}

	void Robot::FuelToBoiler(){

	}

	void Robot::HopperThenShoot(){

	}

	void Robot::ShootThenHopper(){

	}
}
