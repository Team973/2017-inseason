/*
 * CoopMTRobot.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: Andrew
 */

#include "unistd.h"

#include "WPILib.h"

#include "CoopMTRobot.h"
#include "lib/util/Ansi.h"
#include "util/Util.h"
#include "WrapDash.h"
#include <stdio.h>
#include <unistd.h>

namespace frc973 {

CoopMTRobot::CoopMTRobot(void
		): IterativeRobot()
		 , TaskMgr()
		 , m_prevMode(RobotMode::MODE_DISABLED)
		 , m_robotModeMutex(PTHREAD_MUTEX_INITIALIZER)
{
}

CoopMTRobot::~CoopMTRobot() {
}

void CoopMTRobot::RobotInit(void) {
	char hostName[MAXHOSTNAMELEN];
	gethostname(hostName, ARRAYSIZE(hostName));
	printf(ESC_PREFIX SGR_FG_BLACK
		   ESC_SEP SGR_BG_WHITE
		   ESC_SUFFIX
		   "\n****************************************\n"
		   "     Name: %s\n"
		   "  Program: %s\n"
		   " Compiled: %s, %s\n"
		   "\n****************************************\n"
		   ESC_NORMAL,
		   hostName, PROGRAM_NAME, __DATE__, __TIME__
		   );

	Initialize();
}

void CoopMTRobot::DisabledInit(void) {
	this->ModeStop(this->m_prevMode);
	pthread_mutex_lock(&m_robotModeMutex);
	this->m_prevMode = RobotMode::MODE_DISABLED;
	pthread_mutex_unlock(&m_robotModeMutex);
	this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::AutonomousInit(void) {
	this->ModeStop(this->m_prevMode);
	pthread_mutex_lock(&m_robotModeMutex);
	this->m_prevMode = RobotMode::MODE_AUTO;
	pthread_mutex_unlock(&m_robotModeMutex);
	this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::TeleopInit(void) {
	this->ModeStop(this->m_prevMode);
	pthread_mutex_lock(&m_robotModeMutex);
	this->m_prevMode = RobotMode::MODE_TELEOP;
	pthread_mutex_unlock(&m_robotModeMutex);
	this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::TestInit(void) {
	this->ModeStop(this->m_prevMode);
	pthread_mutex_lock(&m_robotModeMutex);
	this->m_prevMode = RobotMode::MODE_TEST;
	pthread_mutex_unlock(&m_robotModeMutex);
	this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::DisabledPeriodic(void) {
	this->TaskPrePeriodicAll(this->m_prevMode);
	this->DisabledContinuous();
	this->AllStateContinuous();
	this->TaskPeriodicAll(this->m_prevMode);
	this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::AutonomousPeriodic(void) {
	this->TaskPrePeriodicAll(this->m_prevMode);
	this->AutonomousContinuous();
	this->AllStateContinuous();
	this->TaskPeriodicAll(this->m_prevMode);
	this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::TeleopPeriodic(void) {
	this->TaskPrePeriodicAll(this->m_prevMode);
	this->TeleopContinuous();
	this->AllStateContinuous();
	this->TaskPeriodicAll(this->m_prevMode);
	this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::TestPeriodic(void) {
	this->TaskPrePeriodicAll(this->m_prevMode);
	this->TestContinuous();
	this->AllStateContinuous();
	this->TaskPeriodicAll(this->m_prevMode);
	this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::ModeStop(RobotMode toStop) {
	switch (toStop) {
	case RobotMode::MODE_DISABLED:
		TaskStopModeAll(toStop);
		DisabledStop();
		break;
	case RobotMode::MODE_AUTO:
		TaskStopModeAll(toStop);
		AutonomousStop();
		break;
	case RobotMode::MODE_TELEOP:
		TaskStopModeAll(toStop);
		TeleopStop();
		break;
	case RobotMode::MODE_TEST:
		TaskStopModeAll(toStop);
		TestStop();
		break;
	}
}

void CoopMTRobot::ModeStart(RobotMode toStart) {
	switch (toStart) {
	case RobotMode::MODE_DISABLED:
		TaskStartModeAll(toStart);
		DisabledStart();
		break;
	case RobotMode::MODE_AUTO:
		TaskStartModeAll(toStart);
		AutonomousStart();
		break;
	case RobotMode::MODE_TELEOP:
		TaskStartModeAll(toStart);
		TeleopStart();
		break;
	case RobotMode::MODE_TEST:
		TaskStopModeAll(toStart);
		TestStart();
		break;
	}
}

bool CoopMTRobot::IsDisabled() const {
	pthread_mutex_lock(&m_robotModeMutex);
	bool res = m_prevMode == MODE_DISABLED;
	pthread_mutex_unlock(&m_robotModeMutex);
	return res;
}

bool CoopMTRobot::IsEnabled() const {
	return !IsDisabled();
}

bool CoopMTRobot::IsOperatorControl() const {
	pthread_mutex_lock(&m_robotModeMutex);
	bool res = m_prevMode == MODE_TELEOP;
	pthread_mutex_unlock(&m_robotModeMutex);
	return res;
}

bool CoopMTRobot::IsAutonomous() const {
	pthread_mutex_lock(&m_robotModeMutex);
	bool res = m_prevMode == MODE_AUTO;
	pthread_mutex_unlock(&m_robotModeMutex);
	return res;
}

bool CoopMTRobot::IsTest() const {
	pthread_mutex_lock(&m_robotModeMutex);
	bool res = m_prevMode == MODE_TEST;
	pthread_mutex_unlock(&m_robotModeMutex);
	return res;
}

}
