#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from tf import TransformListener,TransformerROS
import math
import tf_conversions

def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)
def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi

if __name__=="__main__":
	# Tell ROS that we're making a new node.
	rospy.init_node("slam_tf_publisher",anonymous=False)
	listener = tf.TransformListener()
	br = tf.TransformBroadcaster()
	transformer = TransformerROS()
	count = 0
	rate = rospy.Rate(10.0)
	matrix = np.identity(4)
	(trans,rot) = listener.lookupTransform('/base_link', '/base_link', rospy.Time(0))
	(trans1,rot1) = (trans,rot)
	thres = 10

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
			(trans1,rot1) = listener.lookupTransform('/slam_map', '/odom', rospy.Time(0))
			# print (trans,rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass	
		transpose_matrix = transformer.fromTranslationRotation(trans, rot)
		transpose_matrix1 = transformer.fromTranslationRotation(trans1, rot1)

		matrix = np.dot(transpose_matrix1, transpose_matrix)
		_,_,phi =  euler_angles_from_rotation_matrix(matrix[:3,:3])
		tr = [matrix[0,3], matrix[1,3], matrix[2,3]]
		print "1....." ,(trans,rot) 
		print "2....." ,(trans1,rot1)
		print "3....." , tr
		
		br.sendTransform(tr,tf_conversions.transformations.quaternion_from_euler(0, 0, phi),rospy.Time.now(), "/slam_map", "/base_link")
		rate.sleep()
		