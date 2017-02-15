/*
 * LogSpreadsheetBase.h
 *
 * Base class for an object that writes a spreadsheet.
 *
 * Define a subclass to use this.  Creates a row every time TaskPostPeriodic
 * is called, so if you wanted you could create a TaskManager and log at
 * whatever frequency you want.
 *
 *  Created on: Nov 24, 2015
 *      Author: Andrew
 */

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include "../TaskMgr.h"
#include "../CoopTask.h"
#include <pthread.h>

namespace frc973 {

constexpr uint32_t DEFAULT_MAX_LOG_CELL_SIZE = 32;

constexpr uint32_t LOG_CELL_FLAG_CLEAR_ON_READ = 1;

/**
 * Represents a column in the spreadsheet.  For the column to be printed,
 * you must register the instance of a LogCell in the LogSpreadsheet
 * using LogSpreadsheet.RegisterCell
 */
class LogCell {
public:
	/**
	 * Instantiate a LogCell passing in the name of the column
	 * and the maximum length of the field (default 32 chars)
	 */
	LogCell(const char *name,
			uint32_t size = DEFAULT_MAX_LOG_CELL_SIZE,
			uint32_t flags = 0);

	/**
	 * Destroy the LogCell instance, freeing up the memory allocated
	 * for the cell contents
	 */
	virtual ~LogCell();

	/**
	 * Log a String (C-string)
	 *
	 * @param text to be put in the cell
	 */
	void LogText(const char *text);

	/**
	 * Log a double
	 *
	 * @param double to be logged in the cell
	 */
	void LogDouble(double val);

	/**
	 * Log a String using printf-style syntax
	 *
	 * @param formatstr containing text and printf-style %directives
	 * @param var_arg list of arguments to be printed
	 */
	void LogPrintf(const char *formatstr, ...);

	/**
	 * Return the name of this cell
	 */
	const char* GetName();

	/**
	 * Get the contents of the string
	 */
	virtual const char *GetContent();

	/**
	 * Clear the cell so its contents are empty.
	 */
	void ClearCell();

	/**
	 * Check whether the cell should be cleared after being read
	 *
	 * @returns whether the cell should be cleared after being logged
	 */
	bool ClearOnRead() {
		return m_flags & LOG_CELL_FLAG_CLEAR_ON_READ;
	}

	void AcquireLock() {
		pthread_mutex_lock(&m_mutex);
	}

	void ReleaseLock() {
		pthread_mutex_unlock(&m_mutex);
	}

protected:
	char *m_buffer;

private:
	const char *m_name;
	const int m_buffSize;
	const uint32_t m_flags;
	pthread_mutex_t m_mutex;
};

/**
 * A LogSpreadsheet writes to a file the contents of each of its registered
 * cells.
 */
class LogSpreadsheet : public CoopTask {
public:
	/**
	 * Create a new LogSpreadsheetBase... we need the scheduler that will
	 * be calling us.
	 *
	 * @param scheduler the Task Manager to register this with
	 */
	LogSpreadsheet(TaskMgr *scheduler);
	virtual ~LogSpreadsheet();

	/**
	 * Initialize the table... open the file, write the column headers, etc.
	 */
	void InitializeTable();

	/**
	 * Called regularly. If we are initialized, write a row of the table.
	 *
	 * @param RobotMode mode
	 */
	void TaskPostPeriodic(RobotMode mode);

	/**
	 * Register a new column to be logged.  You will figure this value out
	 * in GetValue.  You cannot register more columns after the header of
	 * the file has already been written.
	 *
	 * @param cell to register
	 * @param boolean says whether to clear the cell after reading it
	 */
	void RegisterCell(LogCell *cell);
private:
	/**
	 * Write a in the table...
	 *
	 * For each registered column, use GetValue to figure ouw what goes
	 * in that column then write some coma-seperated, quote-enclosed
	 * values with a line break at the end.
	 */
	void WriteRow();

	std::vector<LogCell*> m_cells;
	std::ofstream *m_oFile;
	TaskMgr *m_scheduler;
	bool m_initialized;
	RobotMode m_mode;
};

}
