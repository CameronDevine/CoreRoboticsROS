#include <vector>
#include "CoreRobotics.hpp"
#include "robot.h"

using namespace CoreRobotics;

int main(void) {
	CRManipulator* MyRobot = new CRManipulator();
	CRRigidBody* link;
	int linkIndex;
	for (int i = 0; i < frames.size(); i++) {
		link = new CRRigidBody(frames[i]);
		linkIndex = MyRobot->addLink(link);
	}
	CRFrameEuler* tool = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	toolIndex = MyRobot->addTool(linkIndex, tool);
	CRHardLimits solver = CRHardLimits(*MyRobot, toolIndex, convention, true);
}
