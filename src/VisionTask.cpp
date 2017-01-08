/*
 * VisionTask.cpp
 *
 *  Created on: Mar 20, 2016
 *      Author: Andrew
 */


#include <VisionTask.h>
#include "lib/WrapDash.h"

namespace frc973 {

VisionTask::VisionTask()
		 : m_thread()
		 , m_initDoneP(false)
		 , m_runningP(false)
		 , m_callback(nullptr)
		 , m_session()
		 , m_image(nullptr)
		 , m_thresholdImage(nullptr) {
	printf("starting vision task thread\n");
	pthread_create(&m_thread, NULL, InitVision, this);
	//InitVision(this);
	printf("finished starting vision task thread\n");
}

VisionTask::~VisionTask() {
	// TODO Auto-generated destructor stub
}

void VisionTask::KillTask() {
	if (m_runningP) {
		pthread_cancel(m_thread);
		m_runningP = false;
		m_callback = nullptr;
	}
}

bool VisionTask::GrabOffset(VisionDataReceiver* receiver) {
	DBStringPrintf(DBStringPos::DB_LINE9,
			"v init %d", m_initDoneP);
	if (m_initDoneP && !m_runningP) {
		m_runningP = true;
		m_callback = receiver;
		pthread_create(&m_thread, NULL, DoVision, this);
		return true;
	}
	else {
		if (receiver != nullptr) {
			receiver->VisionReceiveFailure();
		}
		return false;
	}
}

void *VisionTask::InitVision(void *p) {
	try {
		VisionTask *inst = (VisionTask *) p;
		printf("<InitVision>\n");
		inst->m_image = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		inst->m_thresholdImage = imaqCreateImage(IMAQ_IMAGE_U8, 0);
		int imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &inst->m_session);
		SetCameraSettings(inst->m_session, 5.0, 30.0, 10.0);
		SetCameraSettings(inst->m_session, 5.0, 30.0, 10.0);
		SetCameraSettings(inst->m_session, 5.0, 30.0, 10.0);

		if(imaqError != IMAQdxErrorSuccess) {
			printf("IMAQdxOpenCamera error: %d\n", imaqError);
		}

		if (imaqError == IMAQdxErrorSuccess) {
			imaqError = IMAQdxConfigureGrab(inst->m_session);
		}
		if(imaqError != IMAQdxErrorSuccess) {
			printf("IMAQdxOpenCamera error: %d\n", imaqError);
		}

		if (imaqError == IMAQdxErrorSuccess) {
			inst->m_initDoneP = true;
		}
	}
	catch (std::exception &e) {
		printf("Does C++ even support exceptions?  Like... how does it unroll the stack?  That's crazy\n");
	}

	printf("</Initvision>");
	return nullptr;
}

void *VisionTask::DoVision(void* p) {
	try {
		VisionTask *inst = (VisionTask *) p;
		int imaqError;
		char *buff = inst->m_txtBuff;

		double xDistance = NAN;
		double xAngle = NAN;
		bool success = false;

		imaqClearError();

		imaqError = IMAQdxStartAcquisition(inst->m_session);

		if (imaqError >= IMAQdxErrorSuccess) {
			imaqError = IMAQdxGrab(inst->m_session, inst->m_image, true, NULL);
			printf("grabbed frame\n");
		}

		if (imaqError >= IMAQdxErrorSuccess) {
			imaqError = imaqColorThreshold(inst->m_thresholdImage, inst->m_image, 255,IMAQ_HSV,
					&(inst->RING_HUE_RANGE), &(inst->RING_SAT_RANGE), &(inst->RING_VAL_RANGE));
			printf ("threshold image\n");
		}
		else {
			printf("failure grabbing image");
		}

		if (imaqError >= IMAQdxErrorSuccess) {
			int numParticles = 0;
			imaqCountParticles(inst->m_thresholdImage, 1, &numParticles);

			snprintf(buff, BUFF_LEN, "#parts %d", numParticles);
			//SmartDashboard::PutString("DB/String 3", buff);
			printf("Number of Particles %d\n ", numParticles);
		}
		else {
			printf("th er %d\n", imaqError);
			snprintf(buff, BUFF_LEN, "th er %d", imaqError);
			//SmartDashboard::PutString("DB/String 3", buff);
		}

		if (imaqError >= IMAQdxErrorSuccess) {
			int numParticles = 0;
			imaqCountParticles(inst->m_thresholdImage, 1, &numParticles);

			std::vector<ParticleReport> particles;
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
				ParticleReport par;
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &(par.PercentAreaToImageArea));
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_AREA, &(par.Area));
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(par.BoundingRectTop));
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(par.BoundingRectLeft));
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(par.BoundingRectBottom));
				imaqMeasureParticle(inst->m_thresholdImage, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(par.BoundingRectRight));
				if (par.Area < 100) {
					continue;
				}
				printf("Particle report: %lf %lf %lf %lf %lf %lf\n",
						par.PercentAreaToImageArea, par.Area, par.BoundingRectTop,
						par.BoundingRectLeft, par.BoundingRectBottom, par.BoundingRectRight);
				particles.push_back(par);
			}
			sort(particles.begin(), particles.end(), CompareParticleSizes);
			if (particles.size() != 0){
				double xPxOffset = (particles[0].BoundingRectLeft + particles[0].BoundingRectRight) / 2.0;
				double TARGET_WIDTH_PIXELS = particles[0].BoundingRectRight - particles[0].BoundingRectLeft;

				success = true;
				xAngle = (xPxOffset - 320.0) * X_FOV / 320.0;
				xDistance = TARGET_WIDTH_INCHES * FOCAL_LENGTH / TARGET_WIDTH_PIXELS;

				snprintf(buff, BUFF_LEN, "aa %.2lf x %lf", particles[0].PercentAreaToImageArea,
						xPxOffset);
				//SmartDashboard::PutString("DB/String 6", buff);

				snprintf(buff, BUFF_LEN, "u %lf z %lf", TARGET_WIDTH_PIXELS, xDistance);
				//SmartDashboard::PutString("DB/String 7", buff);

				snprintf(buff, BUFF_LEN, "xang %lf", xAngle);
				//SmartDashboard::PutString("DB/String 8", buff);
			}
			else {
				//SmartDashboard::PutString("DB/String 6", "noparticles");
			}
		}
		else {
			printf("bigO er %d\n", imaqError);
			snprintf(buff, BUFF_LEN, "bigO er %d", imaqError);
			//SmartDashboard::PutString("DB/String 3", buff);
		}

		printf("Done with vision processing\n");
		if (inst->m_callback != nullptr) {
			printf("Our callback is registered\n");
			if (success) {
				printf("Sending our vision data to callback: dist %lf angle %lf\n", xDistance, xAngle);
				inst->m_callback->VisionReceiveDistance(xDistance);
				inst->m_callback->VisionReceiveXAngle(xAngle);
			}
			else {
				printf("Sending our vision data to callback: Failure!!!!!");
				inst->m_callback->VisionReceiveFailure();
			}
		}

		CameraServer::GetInstance()->SetImage(inst->m_image);

		// stop image acquisition
		IMAQdxStopAcquisition(inst->m_session);

		inst->m_runningP = false;
	}
	catch (std::exception &e) {
		printf("No really... C++ shouldn't support exceptions it would have to create a new stack frame for each try block and note on that stack frame what handlers are registered, then try them all in reverse order and that's a lot of overhead I don't believe that happens\n");
	}
	return nullptr;
}

int VisionTask::SetCameraSettings(IMAQdxSession &sess, double exp, double brit, double contrast) {
	int ret = 0;

	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Exposure::Mode",
			IMAQdxValueTypeString, "Manual");
	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Exposure::Value",
			IMAQdxValueTypeF64, exp);
	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Brightness::Mode",
			IMAQdxValueTypeString, "Manual");
	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Brightness::Value",
			IMAQdxValueTypeF64, brit);

	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Contrast::Mode",
			IMAQdxValueTypeString, "Manual");
	ret |= IMAQdxSetAttribute(sess,
			"CameraAttributes::Contrast::Value",
			IMAQdxValueTypeF64, contrast);
	return ret;
}

} /* namespace frc973 */

