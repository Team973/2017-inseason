/*
 * WrapDash.h
 *
 * Quick wrapper of wpilib's debug string interface
 *
 *  Created on: Sep 3, 2015
 *      Author: Andrew
 */

#ifndef FRCLIB_WRAPDASH_H_
#define FRCLIB_WRAPDASH_H_

namespace frc973 {

enum DBStringPos {
	DB_LINE0,
	DB_LINE1,
	DB_LINE2,
	DB_LINE3,
	DB_LINE4,
	DB_LINE5,
	DB_LINE6,
	DB_LINE7,
	DB_LINE8,
	DB_LINE9
};

/**
 * Use printf-like syntax to print to the smart dash debug string place
 */
void DBStringPrintf(DBStringPos position, const char *formatstring, ...);

}

#endif
