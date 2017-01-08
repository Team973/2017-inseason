/*
 * Compressor.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: Andrew
 */

/*
 * Compressor.h
 *
 *  Created on: Nov 3, 2015
 *      Author: Andrew
 */

#include "GreyCompressor.h"

#include "WPILib.h"
#include "lib/filters/Debouncer.h"

namespace frc973 {

GreyCompressor::GreyCompressor(DigitalInput *pressureSwitch, Relay *compressor,
		TaskMgr *scheduler) :
				m_enabled(true),
				m_pressureSwitchFilter(new Debouncer(2.0)),
				m_airPressureSwitch(pressureSwitch),
				m_compressor(compressor),
				m_scheduler(scheduler)
{
	m_airPressureSwitch = pressureSwitch;
	m_compressor = compressor;
	m_scheduler = scheduler;

	this->m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);
}

GreyCompressor::~GreyCompressor() {
	this->m_scheduler->UnregisterTask(this);
}

void GreyCompressor::Enable() {
	m_enabled = true;
}

void GreyCompressor::Disable() {
	m_enabled = false;
}

void GreyCompressor::TaskPeriodic(RobotMode mode) {
	if (!m_airPressureSwitch->Get() && m_enabled) {
		m_compressor->Set(Relay::kOn);
	}
	else {
		m_compressor->Set(Relay::kOff);
	}
}

}
