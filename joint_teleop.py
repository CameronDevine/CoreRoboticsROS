import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import numpy as np

dt = 0.1

inoutmap = {
	0: 0,
	1: 1,
	4: 2,
	3: 3,
	6: 4,
	7: 5}

joints = [
	'shoulder_pan_joint',
	'shoulder_lift_joint',
	'elbow_joint',
	'wrist_1_joint',
	'wrist_2_joint',
	'wrist_3_joint']

class Controller:
	axes = [0] * 8
	buttons = [0] * 13

	def callback(self, data):
		self.axes = data.axes
		self.buttons = data.buttons

class Teleop:
	q = np.zeros([6])
	vel = np.zeros([6])
	pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size = 5)

	def move(self, joint_vel):
		cmd = JointTrajectory()
		cmd.header.stamp = rospy.Time.now()
		cmd.joint_names = joints
		point = JointTrajectoryPoint()
		point.positions = self.q.tolist()
		point.velocities = self.vel.tolist()
		point.time_from_start = rospy.Duration.from_sec(0)
		cmd.points.append(point)
		self.vel = np.array(joint_vel)
		self.q += dt * self.vel
		point = JointTrajectoryPoint()
		point.positions = self.q.tolist()
		point.velocities = self.vel.tolist()
		point.time_from_start = rospy.Duration.from_sec(dt)
		cmd.points.append(point)
		self.pub.publish(cmd)

controller = Controller()
teleop = Teleop()

rospy.init_node('joint_teleop')
rospy.Subscriber("joy", Joy, controller.callback, queue_size = 1)

rate = rospy.Rate(1. / dt)

vel = [0, 0, 0, 0, 0, 0]
while True:
	for i, j in inoutmap.items():
		vel[j] = controller.axes[i]
	teleop.move(vel)
	rate.sleep()
