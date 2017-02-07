#pragma once

#include "lib/TaskMgr.h"
#include "lib/CoopTask.h"

using namespace frc;

namespace frc973 {

class CoopTask;
class TaskMgr;

class Pixy;

class PixyThread : public CoopTask {
public:
    PixyThread(TaskMgr *scheduler);

    virtual ~PixyThread();

    void TaskPeriodic(RobotMode mode) override;

    double GetOffset();

    bool GetDataFresh();
private:
    TaskMgr *m_scheduler;
    Pixy *m_pixy;
    int m_prevReading;
    double m_offset;
    uint32_t m_prevReadingTime;
};

}
