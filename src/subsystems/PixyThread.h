#pragma once

#include "pthread.h"
#include "lib/TaskMgr.h"
#include "lib/CoopTask.h"
#include "WPILib.h"

using namespace frc;

namespace frc973 {

class CoopTask;
class SingleThreadTaskMgr;

class Pixy;

class PixyThread : public CoopTask {
public:
    PixyThread(RobotStateInterface &stateProvider);
    virtual ~PixyThread();

    void TaskPeriodic(RobotMode mode) override;

    double GetOffset();

    bool GetDataFresh();
private:

    SingleThreadTaskMgr *m_thread;
    Pixy *m_pixy;
    int m_prevReading;
    double m_offset;
    uint32_t m_prevReadingTime;
	pthread_mutex_t	m_mutex;
};

}
