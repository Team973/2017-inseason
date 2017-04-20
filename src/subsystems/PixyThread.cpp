#include "WPILib.h"
#include "lib/SmartPixy.h"
#include "lib/util/Util.h"
#include "subsystems/PixyThread.h"
#include "lib/SingleThreadTaskMgr.h"

using namespace frc;

namespace frc973 {

PixyThread::PixyThread(RobotStateInterface &stateProvider) :
    m_thread(new SingleThreadTaskMgr(stateProvider, 1/50.0, false)),
    m_pixy(new Pixy()),
    m_prevReading(0),
    m_offset(0.0),
    m_prevReadingTime(0),
    m_mutex(PTHREAD_MUTEX_INITIALIZER)
{
    m_thread->Start();
    fprintf(stderr, "gonna register the pixy task\n");
    m_thread->RegisterTask("Pixy", this, TASK_PERIODIC);
    fprintf(stderr, "registered the pixy task\n");
    m_prevReadingTime = GetMsecTime();
}

PixyThread::~PixyThread() {
    m_thread->UnregisterTask(this);
    m_thread->Stop();
}

void PixyThread::TaskPeriodic(RobotMode mode) {
    /*
    if (GetMsecTime() - m_prevReadingTime > 80) {
        printf("Recreating link\n");
        delete m_pixy;
        m_pixy = new PixyDriver(new PixyLinkI2C(I2C::Port::kOnboard));
    }
    */

    int numBlocks = m_pixy->getBlocks(4);
    double currentRead = m_prevReading;

	pthread_mutex_lock(&m_mutex);
    if (numBlocks >= 2){
        currentRead = (
                (double) (m_pixy->blocks[0].x +
                          m_pixy->blocks[1].x)) / 2.0;
        m_prevReadingTime = GetMsecTime();
    }
    else if (numBlocks == 1){
        currentRead = m_pixy->blocks[0].x;
        m_prevReadingTime = GetMsecTime();
    }

    m_prevReading = (m_prevReading + currentRead) / 2.0;
/*
    printf("reading %lf\n", m_prevReading);
    printf("numBlocks %d\n", numBlocks);
    */
	pthread_mutex_unlock(&m_mutex);
}

double PixyThread::GetOffset() {
	pthread_mutex_lock(&m_mutex);
    double ret = -((m_prevReading / 319.0) - 0.5);
	pthread_mutex_unlock(&m_mutex);
    return ret;
}

bool PixyThread::GetDataFresh() {
	pthread_mutex_lock(&m_mutex);
    bool ret = GetMsecTime() - m_prevReadingTime < 50;
	pthread_mutex_unlock(&m_mutex);
    return ret;
}

}
