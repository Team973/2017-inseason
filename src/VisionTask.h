/*
 * VisionTask.h
 *
 *  Created on: Mar 20, 2016
 *      Author: Andrew
 */

#ifndef SRC_VISIONTASK_H_
#define SRC_VISIONTASK_H_


#include "pthread.h"
#include "WPILib.h"

namespace frc973 {

class VisionDataReceiver {
public:
	VisionDataReceiver() {}
	virtual ~VisionDataReceiver() {}

	virtual void VisionReceiveDistance(double distance) = 0;
	virtual void VisionReceiveXAngle(double angle) = 0;

	virtual void VisionReceiveFailure() = 0;
};

class VisionTask {
public:
	VisionTask();
	virtual ~VisionTask();

	/**
	 * Kill the vision task if it's running
	 */
	void KillTask();

	/**
	 * Wake up the vision system... ask it to process a single frame and
	 * report back to us when it's done.  Returns true on succes and false
	 * on failure.
	 */
	bool GrabOffset (VisionDataReceiver* receiver);

private:
	static void *InitVision(void *inst);
	static void *DoVision(void* inst);

	pthread_t m_thread;
	bool m_initDoneP;
	bool m_runningP;
	VisionDataReceiver *m_callback;
	static constexpr int BUFF_LEN = 100;
	char m_txtBuff[BUFF_LEN + 1];

	IMAQdxSession m_session;
	Image *m_image;
	Image *m_thresholdImage;
	static constexpr double TARGET_WIDTH_INCHES = 18;
	static constexpr double FOCAL_LENGTH = 713.75;
	static constexpr double X_FOV = 34.25 * 0.69;

	Range RING_HUE_RANGE = {80, 160};
	Range RING_SAT_RANGE = {140, 255};
	Range RING_VAL_RANGE = {140, 235};

	//A structure to hold measurements of a particle
	struct ParticleReport {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
	};

	static bool CompareParticleSizes(ParticleReport particle1,
			ParticleReport particle2) {
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	static int SetCameraSettings(IMAQdxSession &sess, double exp, double brit,
			double contrast);
};

} /* namespace frc973 */

#endif /* SRC_VISIONTASK_H_ */
