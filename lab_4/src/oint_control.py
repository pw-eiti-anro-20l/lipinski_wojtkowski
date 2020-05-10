#!/usr/bin/env python

import rospy
from lab_4.srv import oint_srv

rospy.init_node('oint_control')

def oint_client(x,y,z,r,p,ya,t):
	rospy.wait_for_service('oint_srv')
	try:
		oint_s = rospy.ServiceProxy('oint_srv', oint_srv)
		pos=[x,y,z,r,p,ya]
		resp = oint_s(pos,t)
		return resp.status
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
if __name__=="__main__":
	x=0
	y=0
	z=0
	r=-1.57
	p=0
	ya=-1.57
	t=10
	print "Requesting move to x=%s, y=%s, z=%s, r/p/y=%s/%s/%s, move duration = %s seconds"%(x,y,z,r,p,ya,t/10)
	if oint_client(x,y,z,r,p,ya,t):
		print "move finished"
	x=1
	y=2
	z=0
	r= 1.57
	p= 2
	ya = 0
	t=50
	print "Requesting move to x=%s, y=%s, z=%s, r/p/y=%s/%s/%s, move duration = %s seconds"%(x,y,z,r,p,ya,t/10)
	if oint_client(x,y,z,r,p,ya,t):
		print "move finished"
	z=3
	r=1
	p=0.5
	t=20
	print "Requesting move to x=%s, y=%s, z=%s, r/p/y=%s/%s/%s, move duration = %s seconds"%(x,y,z,r,p,ya,t/10)
	if oint_client(x,y,z,r,p,ya,t):
		print "move finished"
	x=0
	y=0
	z=0
	r=0
	p=0
	ya = 0
	t=20
	print "Requesting move to x=%s, y=%s, z=%s, r/p/y=%s/%s/%s, move duration = %s seconds"%(x,y,z,r,p,ya,t/10)
	if oint_client(x,y,z,r,p,ya,t):
		print "move finished"
	print "demonstration script finished"
