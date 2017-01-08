/*
 * PoseManager.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: andrew
 */

#include <PoseManager.h>
#include <fstream>
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "lib/WrapDash.h"

#include <iostream>
namespace frc973 {

PoseManager::PoseManager(Shooter *shooter, Intake *intake)
		 : m_shooter(shooter)
		 , m_intake(intake)
		 , m_currPose(0)
		 , m_fileLoaded(false) {
	ReloadConfiguration();
}

PoseManager::~PoseManager() {
	// TODO Auto-generated destructor stub
}

void PoseManager::ChooseNthPose(int n) {
	m_currPose = n % m_configRoot["poses"].size();

	printf("Pose now selected: %s\n",
			m_configRoot["poses"][m_currPose]["name"].asCString());

	DBStringPrintf(DBStringPos::DB_LINE9,
			"%s", m_configRoot["poses"][m_currPose]["name"].asCString());
	AssumePose();
}

void PoseManager::ReloadConfiguration() {
	fprintf(stderr, "Starting parse config file\n");

	std::ifstream fileStream;// ("~/presets.json", std::ifstream::in);
	fileStream.open("/home/lvuser/presets.json");
	Json::Reader reader;

	if (fileStream) {
		reader.parse(fileStream, m_configRoot);
		printf("parsed config file correcly!\n");
		m_fileLoaded = true;
	}
	else {
		printf("No file found!\n");
		m_fileLoaded = false;
	}
	fprintf(stderr, "Finished parse config file\n");
}

void PoseManager::NextPose() {
	if (m_fileLoaded) {
		m_currPose = (m_currPose + 1) % m_configRoot["poses"].size();

		printf("Pose now selected: %s\n",
				m_configRoot["poses"][m_currPose]["name"].asCString());

		DBStringPrintf(DBStringPos::DB_LINE9,
				"%s", m_configRoot["poses"][m_currPose]["name"].asCString());
	}
	else {
		m_currPose = (m_currPose + 1) % 4;

		switch (m_currPose) {
		case STOW_POSE:
			DBStringPrintf(DBStringPos::DB_LINE9, "fallback stow");
			break;
		case BATTER_SHOT_POSE:
			DBStringPrintf(DBStringPos::DB_LINE9, "fallback batter shot");
			break;
		case CHIVAL_POSE:
			DBStringPrintf(DBStringPos::DB_LINE9, "fallback far defense");
			break;
		case NEAR_DEFENSE_SHOT_POSE:
			DBStringPrintf(DBStringPos::DB_LINE9, "fallback near defense");
			break;
		}
	}
}

void PoseManager::AssumePose() {
	if (m_fileLoaded) {
		Json::Value pose = m_configRoot["poses"][m_currPose];
		std::cout << "there are this many " << m_configRoot["poses"].size();
		std::cout << "my name is " << pose["name"];

		if (pose["intakeExtended"].asBool()) {
			m_intake->SetIntakePosition(Intake::IntakePosition::extended);
		}
		else {
			m_intake->SetIntakePosition(Intake::IntakePosition::retracted);
		}

		char shooterHeight =
				(pose["longCylinderExtended"].asBool() << 1) |
				pose["shortCylinderExtended"].asBool();
		switch(shooterHeight) {
		case 0x00:
			m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::wayLow);
			break;
		case 0x01:
			m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::midLow);
			break;
		case 0x02:
			m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::midHigh);
			break;
		case 0x03:
			m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::wayHigh);
			break;
		}

		std::cout << "observing %s\n" << pose["superiorFlywheelControl"].asString();
		if (pose["superiorFlywheelControl"].asString() == "closed") {
			m_shooter->SetFrontFlywheelSSShoot(pose["superiorFlywheelRPM"].asDouble());
			printf("oui oui!\n");
		}
		else {
			m_shooter->SetFrontFlywheelPower(pose["superiorFlywheelPower"].asDouble());
			printf("nont\n");
		}

		if (pose["inferiorFlywheelControl"].asString() == "closed") {
			m_shooter->SetBackFlywheelSSShoot(pose["inferiorFlywheelRPM"].asDouble());
		}
		else {
			m_shooter->SetBackFlywheelPower(pose["inferiorFlywheelPower"].asDouble());
		}
	}
	else {
		AssumePoseFallback(m_currPose);
	}
}

void PoseManager::Chill() {
	m_shooter->SetFlywheelPower(0.0);
}

void PoseManager::AssumePoseFallback(int p) {
	switch (m_currPose) {
	case STOW_POSE:
		m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::wayLow);
		m_shooter->SetFrontFlywheelPower(0.5);
		m_shooter->SetBackFlywheelPower(0.5);
		break;
	case BATTER_SHOT_POSE:
		m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::wayHigh);
		m_shooter->SetFrontFlywheelPower(1.0);
		m_shooter->SetBackFlywheelPower(1.0);
		break;
	case CHIVAL_POSE:
		m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::midHigh);
		m_shooter->SetFrontFlywheelSSShoot(5500.0);
		m_shooter->SetBackFlywheelPower(0.8);
		break;
	case NEAR_DEFENSE_SHOT_POSE:
		m_shooter->SetElevatorHeight(Shooter::ElevatorHeight::midHigh);
		m_shooter->SetFrontFlywheelSSShoot(4850.0);
		m_shooter->SetBackFlywheelPower(0.8);
		break;
}
}

} /* namespace frc973 */
