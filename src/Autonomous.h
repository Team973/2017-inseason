using namespace frc;

namespace frc973 {

static constexpr double DRIVER_STATION_BASE_LINE_DIST = 93.3;
static constexpr double DRIVER_STATION_LAUNCHPAD_DIST = 185.3;
static constexpr double KEY_DIST = 52.0;

	void Robot::AutonomousStart(void) {
		printf("***auto start\n");

    m_shooter->SetFlywheelStop();
		m_ballIntake->BallIntakeStop();
		m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
		m_drive->Zero();

		m_autoState = 0;
	}

	void Robot::AutonomousStop(void) {
		printf("***auto stop\n");
	}

	void Robot::AutonomousContinuous(void) {
		switch (m_autoRoutine){
			case AutonomousRoutine::GearLeftPeg:
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
		switch (m_autoState){
			case 0:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 10.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_drive->PIDDrive(0.0, 130.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 2:
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
				break;
		}
	}

	void Robot::GearMidPeg(){
		switch (m_autoState) {
			case 0:
				m_drive->PIDDrive(DRIVER_STATION_BASE_LINE_DIST, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
				break;
		}
	}

	void Robot::GearRtPeg(){
		switch (m_autoState){
			case 0:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 10.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_drive->PIDDrive(0.0, -130.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 2:
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
				break;
		}
	}

	void Robot::FuelToBoiler(){
		switch (m_autoState){
			case 0:
				m_drive->PIDDrive(DRIVER_STATION_BASE_LINE_DIST / 2.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_drive->PIDDrive(0.0, -130.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 2:
				m_drive->PIDDrive(DRIVER_STATION_BASE_LINE_DIST / 2.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
			case 3:
				m_shooter->SetFlywheelPow(1.0);
				m_autoState++;
				break;
			case 4:
				m_ballIntake->BallIntakeStart();
		}
	}

	void Robot::HopperThenShoot(){
		switch (m_autoState){
			case 0:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_drive->PIDDrive(0.0, -90.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 2:
				m_drive->PIDDrive(-40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
			case 3:
				m_drive->PIDDrive(0.0, 180.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 4:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
			case 5:
				m_shooter->SetFlywheelPow(1.0);
				m_autoState++;
				break;
			case 6:
				m_ballIntake->BallIntakeStart();
				break;
		}
	}

	void Robot::ShootThenHopper(){
		switch (m_autoState){
			case 0:
				m_drive->PIDDrive(0.0, -90.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 1:
				m_drive->PIDDrive(40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 2:
				m_shooter->SetFlywheelPow(1.0);
				m_autoState++;
				break;
			case 3:
				m_ballIntake->BallIntakeStart();
				m_autoState++;
				break;
			case 4:
				m_drive->PIDDrive(0.0, 90.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 5:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 6:
				m_drive->PIDDrive(0.0, 180.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
				break;
			case 7:
				m_drive->PIDDrive(DRIVER_STATION_LAUNCHPAD_DIST + 40.0, 0.0, DriveBase::RelativeTo::SetPoint, 0.8);
				m_autoState++;
			case 8:
				m_shooter->SetFlywheelPow(1.0);
				m_autoState++;
				break;
			case 9:
				m_ballIntake->BallIntakeStart();
				break;
		}
	}
}
