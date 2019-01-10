#include <vector>
#include "CoreRobotics.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define convention CR_EULER_MODE_XYZ

using namespace CoreRobotics;

class FK {
public:
	FK(void);
	void fk(const sensor_msgs::JointState::ConstPtr& msg);
private:
	CRManipulator* MyRobot;
};

FK::FK(void) {
	#include "robot.h"
	this->MyRobot = new CRManipulator();
	CRRigidBody* link;
	int linkIndex;
	for (std::vector<CRFrameEuler*>::iterator frame = frames.begin(); frame < frames.end(); frame++) {
		link = new CRRigidBody(*frame);
		linkIndex = this->MyRobot->addLink(link);
	}
	CRFrameEuler* tool = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	this->MyRobot->addTool(linkIndex, tool);
}

void FK::fk(const sensor_msgs::JointState::ConstPtr& msg) {
	this->MyRobot->setConfiguration(Eigen::VectorXd::Map(msg->position.data(), msg->position.size()));
	std::cout << this->MyRobot->getForwardKinematics() << std::endl << std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fk");
	ros::NodeHandle node;
	FK fk = FK();
	ros::Subscriber q_sub = node.subscribe("q", 10, &FK::fk, &fk);
	ros::spin();
}
