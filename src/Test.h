/*
 * Test.h
 *
 *  Created on: Oct 14, 2015
 *      Author: Andrew
 */

#ifndef SRC_TEST_H_
#define SRC_TEST_H_

namespace frc973 {

void Robot::TestStart(void) {
    printf("***test start\n");
    m_shooter->SetFlywheelStop();
}

void Robot::TestStop(void) {
    printf("***test stop\n");
}

void Robot::TestContinuous(void) {
}

}

#endif /* SRC_TEST_H_ */
