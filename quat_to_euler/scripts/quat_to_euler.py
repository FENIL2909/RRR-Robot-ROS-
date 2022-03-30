#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Quaternion


def callback(Quaternion):
	a = Quaternion.w
	b = Quaternion.x
	c = Quaternion.y
	d = Quaternion.z
	
	#A = (2*((a*b)+(c*d)))/((a*a)-(b*b)-(c*c)+(d*d))
	B = 2*((b*d)-(a*c))
	#C = (2*((a*d)+(b*c)))/((a*a)+(b*b)-(c*c)-(d*d))
	
	Phi = math.degrees(np.arctan2(2*((a*b)+(c*d)),((a*a)-(b*b)-(c*c)+(d*d))))
	Theta = math.degrees(-np.arcsin(B))
	Psi = math.degrees(np.arctan2(2*((a*d)+(b*c)),((a*a)+(b*b)-(c*c)-(d*d))))
	
	
	rospy.loginfo("PHI: %f", Phi)
	rospy.loginfo("THETA: %f", Theta)
	rospy.loginfo("PSI: %f", Psi)
	

def quat_to_euler():
	rospy.init_node('quat_to_euler', anonymous=True)
	rospy.Subscriber("QUATERNION_TO_EULER", Quaternion, callback)
	rospy.spin()

if __name__ == '__main__':
	quat_to_euler()
