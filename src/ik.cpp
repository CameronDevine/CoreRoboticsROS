#include <vector>
#include "CoreRobotics.hpp"
#include "robot.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Empty.h"

using namespace CoreRobotics;

class Solver {
public:
	Solver(void);
	bool setPoseElements(
		sensor_msgs::JointState &req,
		std_msgs::Empty &res) {
			this->solver.setPoseElements(Eigen::Matrix<bool, 6, 1>(req.positions));
			return true;
	};
	bool getPoseElements(
		std_msgs::Empty &req,
		sensor_msgs::JointState &res) {
			res.positions = &this->solver.getPoseElements.cast<float>().data();
			return true;
	};
	bool setConfiguration(
		sensor_msgs::JointState &req,
		std_msgs::Empty &res) {
			this->solver.setConfiguration(Eigen::VectorXd(req.positions));
			return true;
	};
	bool getConfiguration(
		std_msgs::Empty &req,
		sensor_msgs::JointState &res) {
			res.positions = &this->solver.getConfiguration().data();
			return true;
	};
private:
	CRManipulator* MyRobot;
	int toolIndex;
	CRHardLimits solver;
}

Solver::Solver(void) {
	this->MyRobot = new CRManipulator();
	CRRigidBody* link;
	int linkIndex;
	for (int i = 0; i < frames.size(); i++) {
		link = new CRRigidBody(frames[i]);
		linkIndex = this->MyRobot->addLink(link);
	}
	CRFrameEuler* tool = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	this->toolIndex = MyRobot->addTool(linkIndex, tool);
	this->solver = CRHardLimits(*MyRobot, toolIndex, convention, true);
}

int main(void) {
	Solver solver = Solver();
}
