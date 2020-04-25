#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import quaternion
import dhjoint
from math import pi
from math import cos
from math import sin
from geometry_msgs.msg import PoseStamped

rospy.init_node('nonkdl', anonymous=True)
# ogarnac obsluge danych w joint state - oba sa podobne tylko trzeba wyczaic co i jak 
#joint1 = rospy.get_param('joint1')
#joint3 = rospy.get_param('joint3')
#joint2 = rospy.get_param('joint2')
a1 = rospy.get_param('~a1')
a2 = rospy.get_param('~a2')
d3 = rospy.get_param('~d3')
alpha2 = rospy.get_param('~alpha2')
theta1 = rospy.get_param('~theta1')
theta2 = rospy.get_param('~theta2')
#pub = rospy.Publisher("PoseStamped", PoseStamped, queue_size = 10)
joints=[]
joints.append(dhjoint.djoint(0,0,0,theta1,True))
joints.append(dhjoint.djoint(a1,0,0,theta2,True))
joints.append(dhjoint.djoint(a2,d3,alpha2,0,False))

def count(data):
	pos_change = data.position
	i=0
	for joint in joints:
		if joint.rot:
			joint.theta = pos_change[i]
		else:
			joint.d = pos_change[i]		
		i+=1
		#A = np.array([[cos(joint.theta), -sin(joint.theta), 0., joint.a], 
		#[sin(joint.theta)*cos(joint.alfa), cos(joint.theta)*sin(joint.alfa), -sin(joint.alfa), -joint.d*sin(joint.alfa)], 
		#[sin(joint.theta)*sin(joint.alfa), cos(joint.theta)*sin(joint.alfa), cos(joint.alfa), joint.d*cos(joint.alfa)], 
		#[0., 0., 0., 1.]])
		A=np.array([[cos(joint.theta), -sin(joint.theta)*cos(joint.alfa), sin(joint.theta)*sin(joint.alfa), joint.a*cos(joint.theta)], 
		[sin(joint.theta), cos(joint.theta)*cos(joint.alfa), -cos(joint.theta)*sin(joint.alfa), -joint.a*sin(joint.theta)], 
		[0, sin(joint.alfa), cos(joint.alfa), joint.d], 
		[0., 0., 0., 1.]])
		try:
			T
		except NameError:
			T=np.array(A)
		else:
			T=np.dot(T,A)
		qua = quaternion.from_rotation_matrix(T[:3,:3]) #konwersja macierzy 3x3 z gory T na kwaternion
		p = PoseStamped()
		p.header.frame_id = 'base'
		p.header.stamp = data.header.stamp
		p.pose.position.x = T[0,3]
		p.pose.position.y = T[1,3]
		p.pose.position.z = T[2,3]
		p.pose.orientation.x = qua.x
		p.pose.orientation.y = qua.y
		p.pose.orientation.z = qua.z
		p.pose.orientation.w = qua.w
		k=('PoseStamped%d' %i)
		if i==2:
			print(T)
		rospy.Publisher(k, PoseStamped, queue_size = 10).publish(p)

def listener():
	rospy.Subscriber("joint_states", JointState, count) #dodac nazwe
	rospy.spin()
if __name__ == '__main__':
	listener()

