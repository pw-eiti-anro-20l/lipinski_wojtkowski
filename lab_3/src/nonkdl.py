#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from tf import transformations
import dhjoint
from math import pi
from math import cos
from math import sin
from geometry_msgs.msg import PoseStamped

rospy.init_node('nonkdl', anonymous=True)
a1 = rospy.get_param('/a1')
a2 = rospy.get_param('/a2')
d3 = rospy.get_param('/d3')
alpha2 = rospy.get_param('/alpha2')
theta1 = rospy.get_param('/theta1')
theta2 = rospy.get_param('~/theta2')
joints=[]
joints.append(dhjoint.djoint(0,0,0,theta1,True))
joints.append(dhjoint.djoint(a1,0,0,theta2,True))
joints.append(dhjoint.djoint(a2,d3,alpha2,0,False))

def count(data):
	pos_change = data.position
	i=0
	prev=[]
	T = None
	for joint in joints:
		if joint.rot:
			joint.theta =joint.prev + pos_change[i]
		else :
			joint.d = joint.prev + pos_change[i]	
		i+=1
		A = np.array([[cos(joint.theta), -sin(joint.theta), 0., joint.a], 
		[sin(joint.theta)*cos(joint.alfa), cos(joint.theta)*cos(joint.alfa), -sin(joint.alfa), -joint.d*sin(joint.alfa)], 
		[sin(joint.theta)*sin(joint.alfa), cos(joint.theta)*sin(joint.alfa), cos(joint.alfa), joint.d*cos(joint.alfa)], 
		[0., 0., 0., 1.]])
		if T is None:
			T=np.array(A)
		else:
			T=np.array(np.dot(T,A))
		qua = transformations.quaternion_from_matrix(T) #konwersja macierzy T na kwaternion,
		delta=transformations.quaternion_matrix(qua)
		p = PoseStamped()
		p.header.frame_id = 'base'
		p.header.stamp = data.header.stamp
		p.pose.position.x = T[0,3]
		p.pose.position.y = T[1,3]
		p.pose.position.z = T[2,3]
		p.pose.orientation.x = qua[0]
		p.pose.orientation.y = qua[1]
		p.pose.orientation.z = qua[2]
		p.pose.orientation.w = qua[3]
		k=('PoseStamped%d' %i)
		rospy.Publisher(k, PoseStamped, queue_size = 10).publish(p)

def listener():
	rospy.Subscriber("joint_states", JointState, count)
	rospy.spin()
if __name__ == '__main__':
	listener()