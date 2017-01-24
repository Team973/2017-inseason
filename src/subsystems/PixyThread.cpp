#include "lib/PixyI2C.h"
#include "lib/util/Util.h"
#include "subsystems/PixyThread.h"

using namespace frc;

namespace frc973 {
    PixyThread::PixyThread(TaskMgr *scheduler) :
        m_scheduler(scheduler),
        m_pixy(new TPixy(new LinkI2C())),
        m_prevReading(0),
        m_offset(0.0),
        m_prevReadingTime(0)
    {
        m_scheduler->RegisterTask("Pixy", this, TASK_PERIODIC);
        m_prevReadingTime = GetMsecTime();
    }

    PixyThread::~PixyThread() {
        m_scheduler->UnregisterTask(this);
    }

    void PixyThread::TaskPeriodic(RobotMode mode) {
        if (GetMsecTime() - m_prevReadingTime > 80) {
            printf("Recreating link\n");
            delete m_pixy;
            m_pixy = new TPixy(new LinkI2C());
        }

        int numBlocks = m_pixy->GetBlocks(4);
        if (numBlocks >= 2){
            if (m_pixy->blocks[0].x <= m_pixy->blocks[1].x ){
                m_prevReading = m_pixy->blocks[0].x;
            }
            else if (m_pixy->blocks[0].x > m_pixy->blocks[1].x) {
                m_prevReading = m_pixy->blocks[1].x;
            }
            m_prevReadingTime = GetMsecTime();
        }
        else if (numBlocks == 1){
            m_prevReading = m_pixy->blocks[0].x;
            m_prevReadingTime = GetMsecTime();
        }
    }

    double PixyThread::GetOffset() {
        return (2.0 * m_prevReading / PIXY_MAX_X) - 0.5;
    }

    bool PixyThread::GetDataFresh() {
        return GetMsecTime() - m_prevReadingTime < 50;
    }
}
