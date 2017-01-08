/*
 * SingleThreadTaskMgr.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: Andrew
 */

#include "unistd.h"
#include "SingleThreadTaskMgr.h"
#include "lib/util/Util.h"

#include "WPILib.h"

#include <stdio.h>

namespace frc973 {

SingleThreadTaskMgr::SingleThreadTaskMgr(
		RobotStateInterface &stateProvider,
		double loopPeriod
	): m_thread()
	 , m_mutex(PTHREAD_MUTEX_INITIALIZER)
	 , m_loopPeriodSec(loopPeriod)
	 , m_actuallyRunning(false)
	 , m_shouldBeRunning(false)
	 , m_stateProvider(stateProvider)
{
}

SingleThreadTaskMgr::~SingleThreadTaskMgr() {
	Stop();
}

void SingleThreadTaskMgr::Start(void) {
	pthread_mutex_lock(&m_mutex);
	if (!m_shouldBeRunning) {
		m_shouldBeRunning = true;

		pthread_create(&m_thread, NULL, RunTasks, this);
	}
	pthread_mutex_unlock(&m_mutex);
}

void SingleThreadTaskMgr::Stop(void) {
	pthread_mutex_lock(&m_mutex);
	m_shouldBeRunning = false;
	pthread_mutex_unlock(&m_mutex);
}

double SingleThreadTaskMgr::GetLoopPeriodSec() {
	double period;

	pthread_mutex_lock(&m_mutex);
	period = m_loopPeriodSec;
	pthread_mutex_unlock(&m_mutex);

	return period;
}

void SingleThreadTaskMgr::SetLoopPeriod(double periodSec) {
	pthread_mutex_lock(&m_mutex);
	m_loopPeriodSec = periodSec;
	pthread_mutex_unlock(&m_mutex);
}

bool SingleThreadTaskMgr::IsRunning() {
	bool isRunning;

	pthread_mutex_lock(&m_mutex);
	isRunning = m_shouldBeRunning;
	pthread_mutex_unlock(&m_mutex);

	return isRunning;
}

void* SingleThreadTaskMgr::RunTasks(void *p) {
	SingleThreadTaskMgr *inst = (SingleThreadTaskMgr*) p;
	bool keepRunning = true;
	uint64_t timeSliceStartTimeUs = GetUsecTime();

	pthread_mutex_lock(&inst->m_mutex);
	inst->m_actuallyRunning = true;
	RobotMode state = GetRobotMode(inst->m_stateProvider);
	inst->TaskStartModeAll(state);
	pthread_mutex_unlock(&inst->m_mutex);

	while (keepRunning) {
		pthread_mutex_lock(&inst->m_mutex);

		RobotMode nextState = GetRobotMode(inst->m_stateProvider);

		if (state != nextState) {
			inst->TaskStopModeAll(state);
			inst->TaskStartModeAll(nextState);
			state = nextState;
		}

		inst->TaskPrePeriodicAll(state);
		inst->TaskPeriodicAll(state);
		inst->TaskPostPeriodicAll(state);

		pthread_mutex_unlock(&inst->m_mutex);

		uint64_t timeSliceUsedUs =
				GetUsecTime() - timeSliceStartTimeUs;

		uint64_t timeSliceAllotedUs =
				inst->GetLoopPeriodMs() * 1000.0;

		uint64_t timeSliceRemainingUs =
				timeSliceAllotedUs - timeSliceUsedUs;

		if (timeSliceUsedUs <= timeSliceAllotedUs) {
			usleep(timeSliceRemainingUs);
		}
		else {
			printf("TaskRunner (%fhz) taking too long.  "\
					"Time alloted for period: %llu us; time used %llu us",
					inst->GetLoopFrequency(), timeSliceAllotedUs,
					timeSliceUsedUs);
		}

		timeSliceStartTimeUs += timeSliceAllotedUs;

		pthread_mutex_lock(&inst->m_mutex);
		keepRunning = inst->m_shouldBeRunning;
		pthread_mutex_unlock(&inst->m_mutex);
	}

	pthread_mutex_lock(&inst->m_mutex);
	inst->m_actuallyRunning = false;
	pthread_mutex_unlock(&inst->m_mutex);

	return NULL;
}

}
