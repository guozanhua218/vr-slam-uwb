#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest, GetJointProperties


class Stick(object):
	def __init__(self):
		self.pub_stick = rospy.Publisher('stick/joint_states', JointState, queue_size=1)
		self.get_stick_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		self.robot_name = rospy.get_param("~robot_name", "robot")
	def stick_state(self):
		rospy.wait_for_service("/gazebo/get_joint_properties")
		try:
			joint_name = self.robot_name+"::stick_move_joint"
			res = self.get_stick_state(joint_name)
			# print(res)

			pub_msg = JointState()
			pub_msg.position = res.position
			pub_msg.name = ['stick']
			self.pub_stick.publish(pub_msg)
		except (rospy.ServiceException) as e:
			print(e)
			return 0

	   



if __name__ == "__main__":
	rospy.init_node("wheel_tf")
	stiiiiiiick = Stick()
	while not rospy.is_shutdown():
		stiiiiiiick.stick_state()
		# rospy.sleep(0.1)
