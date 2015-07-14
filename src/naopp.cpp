// Nao Projected Profile Crawl Gait library v0.1
//
// Projected Profile is a gaiting scheme that uses the
// sagittal projection of the robot in order to produce
// a sequence of joint motions that make the robot crawl.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard includes.
#include <vector>
#include <string>
#include <ostream>

// AL Libraries
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

// Robotlib includes.

// Special includes.


/*! \brief Nao Projected Profile namespace. */
namespace naopp{

	void testArmPose(int argc, char* argv[]){
		// Initialize Motion Proxy for walking.
		AL::ALMotionProxy motionPrx(argv[1], 9559);
		// Initialize Posture Proxy for going to predefined poses.
		AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);

	    // Stand Nao.
		pstrPrx.goToPosture("StandInit", 1.0f);

	    std::string effector   = "LArm";
	    int frame      = 0; // FRAME_TORSO
	    // int axisMask   = 7; // just control position
	    int axisMask   = 63; // just control position
	    bool useSensorValues = false;

	    // std::vector<std::vector<float> > path;
	    // std::vector<float> currentTf = motionPrx.getTransform(effector, frame, useSensorValues);
	    // std::vector<float> targetTf  = currentTf;
	    std::vector<float> targetTf(12,0.0f);
	    targetTf.at(3) = 0.1; // x
	    targetTf.at(7) = 0.2; // y
	    targetTf.at(11) = 0.3; // z

	    targetTf.at(8) = 1.0; // wx
	    targetTf.at(5) = 1.0; // wy
	    targetTf.at(2) = -1.0; // wz

	    // path.push_back(targetTf);
	    // path.push_back(currentTf);

	    // Go to the target and back again
	    // float times[]      = {2.0, 4.0}; // seconds

	    // motionPrx.transformInterpolations(effector, frame, path, axisMask, times);
	    motionPrx.setTransforms(effector, frame, targetTf, 0.2f, axisMask);
	    sleep(15.0);

	    targetTf.at(11) = 0.2; // z
	    motionPrx.setTransforms(effector, frame, targetTf, 0.2f, axisMask);
	    sleep(15.0);
	    // Go to rest position
	    motionPrx.rest();
	}
}
