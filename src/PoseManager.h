/*
 * PoseManager.h
 *
 * PoseManager reads a configuration file... there's an example configuration file
 * in {repository-root}/config/presets.json.  Upload a file like this to the
 * home directory on the robot.
 *
 * Optional:  if you make changes to the presets.json file, you can run check.py
 * and it will check it for typos and type-errors.  I don't know what PoseManager
 * will do in the event of one of these errors.  In my experience, it fails
 * silently (looking into it).  So if it fails weirdly, try running check.py and
 * it'll tell you if there's a problem in the config file.
 *
 * At the moment, these are the controls:
 * 	driverJoystick.buttonX -> Chill (stop moving the motors)
 * 	driverJoystick.buttonY -> Go to the next pose in the list
 * 					if you hit the end of the list, wrap back around to the beginning
 * 					don't actually move the motors to the pose, just select it.  The motors
 * 							will move when you "Assume pose"
 * 	driverJoystick.rightTrigger -> Assume pose
 * 					Actually start moving the motors to get to the pose position.
 *
 *  Created on: Feb 26, 2016
 *      Author: andrew
 */

#ifndef SRC_POSEMANAGER_H_
#define SRC_POSEMANAGER_H_

#include "lib/json/json.h"

namespace frc973 {

class Shooter;
class Intake;

class PoseManager {
public:
	/**
	 * Initialize the PoseManager and select the first pose in the list
	 * of poses.
	 *
	 * @param shooter reference to the shooter subsystem
	 * @param intake reference to the intake subsystem
	 */
	PoseManager(Shooter *shooter, Intake *intake);
	virtual ~PoseManager();

	/**
	 * Choose the |n|th configuration in the file... ignore names just
	 * go in order of definition
	 *
	 * @param n the index of the pose
	 */
	void ChooseNthPose(int n);

	static const int STOW_POSE = 0;
	static const int BATTER_SHOT_POSE = 1;
	static const int CHIVAL_POSE = 2;
	static const int NEAR_DEFENSE_SHOT_POSE = 3;

	/**
	 * Reload the list of poses from the configuration file.
	 * Forget whatever pose was selected and go to the first one
	 */
	void ReloadConfiguration();

	/**
	 * Move down the list of poses and select the next one.  If we've reached
	 * the end of the list, loop back to the top.
	 */
	void NextPose();

	/**
	 * Assume the currently selected pose.  This actually sends commands to
	 * the subsystems (before, they were just doing whatever they were doing
	 * before)
	 */
	void AssumePose();

	/**
	 * Turn the subsystems off as appropriate.
	 * This probably means turn the shooter flywheel and the arm off, but
	 * leave the shooter at the height it was at before because it's scary
	 * when it drops.
	 */
	void Chill();
private:

	/**
	 * If we didn't load the file, panick and fall back to running this
	 * function
	 */
	void AssumePoseFallback(int poseNum);

	Shooter *m_shooter;
	Intake *m_intake;

	Json::Value m_configRoot;
	int m_currPose = 0;
	bool m_fileLoaded;
};

} /* namespace frc973 */

#endif /* SRC_POSEMANAGER_H_ */
