/*
 * LogSpreadsheet.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Andrew
 */

#include "lib/logging/LogSpreadsheet.h"
#include "lib/CoopTask.h"

#include <cstdio>
#include <cstdarg>
#include <ctime>

namespace frc973 {

LogCell::LogCell(const char *name,
	uint32_t size,
	uint32_t flags):
		m_buffer(new char[size]),
		m_name(name),
		m_buffSize(size),
		m_flags(flags),
		m_mutex(){
	pthread_mutex_init(&m_mutex, NULL);
	this->ClearCell();
}

LogCell::~LogCell() {
	pthread_mutex_destroy(&m_mutex);
	delete[] m_buffer;
}

void LogCell::LogText(const char *text) {
	this->LogPrintf("%s", text);
}

void LogCell::LogDouble(double val) {
	this->LogPrintf("%lf", val);
}

/**
 * vsnprintf works like printf, but writes into a fixed-sized-buffer
 * and takes a va_list.
 */
void LogCell::LogPrintf(const char *formatstr, ...) {
	va_list args;
	va_start (args, formatstr);

	AcquireLock();
	vsnprintf(m_buffer, m_buffSize, formatstr, args);
	ReleaseLock();

	va_end (args);
}

const char* LogCell::GetName() {
	return m_name;
}

const char *LogCell::GetContent() {
	return m_buffer;
}

void LogCell::ClearCell() {
	m_buffer[0] = '\0';
}

LogSpreadsheet::LogSpreadsheet(TaskMgr *scheduler):
		m_cells(),
		m_oFile(NULL),
		m_scheduler(scheduler),
		m_initialized(false),
		m_mode(RobotMode::MODE_DISABLED)
{
	fprintf(stderr, "Starting logger\n");
	this->m_scheduler->RegisterTask("Logger", this, TASK_POST_PERIODIC);
}

LogSpreadsheet::~LogSpreadsheet() {
	if (m_initialized)
		m_oFile->close();

	this->m_scheduler->UnregisterTask(this);
}

void LogSpreadsheet::TaskPostPeriodic(RobotMode mode) {
	m_mode = mode;

	if (!m_initialized)
		return;

	WriteRow();
}

void LogSpreadsheet::RegisterCell(LogCell *cell) {
	if (m_initialized) {
		printf("You can't add a column after the table has already been initialized: %s\n",
				cell->GetName());
	}
	else {
		m_cells.push_back(cell);
	}
}

void LogSpreadsheet::InitializeTable() {
	if (m_initialized) {
		printf("You can only initialize a table once\n");
		return;
	}

    time_t rawTime;
    struct tm * timeInfo;
    char buffer[80];

    time(&rawTime);
    timeInfo = localtime(&rawTime);
    strftime(buffer, 80, "/home/lvuser/log::%F::%X.txt", timeInfo);

	m_oFile = new std::ofstream(buffer);

	if (!m_oFile->is_open()) {
		printf("Could not open file `%s` for writing\n", buffer);
		return;
	}

	for (std::vector<LogCell*>::iterator it = m_cells.begin();
			it != m_cells.end(); ++it) {
		*m_oFile << "\"" << (*it)->GetName() << "\",";
	}
	*m_oFile << std::endl;

	m_initialized = true;
}

void LogSpreadsheet::WriteRow() {
	for (std::vector<LogCell*>::iterator it = m_cells.begin();
			it != m_cells.end(); ++it) {
		(*it)->AcquireLock();
		*m_oFile << "\"" << (*it)->GetContent() << "\",";

		if ((*it)->ClearOnRead()) {
			(*it)->ClearCell();
		}
		(*it)->ReleaseLock();
	}
	*m_oFile << std::endl;
}

}
