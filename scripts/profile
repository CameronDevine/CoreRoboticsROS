import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

joints = [
	'shoulder_pan_joint',
	'shoulder_lift_joint',
	'elbow_joint',
	'wrist_1_joint',
	'wrist_2_joint',
	'wrist_3_joint']

class Profile:
	dt = 0.1

	def __init__(self, joint_names = joints):
		self.joint_names = joint_names
		self.q = np.zeros([len(joint_names)])
		self.vel = np.zeros([len(joint_names)])
		rospy.init_node('profile')
		self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size = 10)
		rospy.Subscriber('/q', JointState, self.callback, queue_size = 10)

	def callback(self, data):
		msg = JointTrajectory()
		msg.joint_names = self.joint_names
		point = JointTrajectoryPoint()
		point.positions = self.q.tolist()
		point.velocities = self.vel.tolist()
		point.time_from_start = rospy.Duration.from_sec(0)
		msg.points.append(point)
		self.vel = (np.array(data.position) - self.q) / self.dt
		self.q = np.array(data.position)
		point = JointTrajectoryPoint()
		point.positions = self.q.tolist()
		point.velocities = self.vel.tolist()
		point.time_from_start = rospy.Duration.from_sec(self.dt)
		msg.points.append(point)
		self.pub.publish(msg)

if __name__ == '__main__':
	Profile()
	rospy.spin()
