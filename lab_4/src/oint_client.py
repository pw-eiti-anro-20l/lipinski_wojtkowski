#!/usr/bin/env python

import sys
import rospy
sys.path.append('/home/marcin/catkin_ws/src/lab_4')
from lab_4.srv import oint_srv
def usage():
	return "%s [x y z r p y duration_in_10ths_seconds_(optional)]"%sys.argv[0]
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
	if len(sys.argv)==8:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		z = float(sys.argv[3])
		r = float(sys.argv[4])
		p = float(sys.argv[5])
		ya = float(sys.argv[6])
		t = float(sys.argv[7])
	elif len(sys.argv)==7:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		z = float(sys.argv[3])
		r = float(sys.argv[4])
		p = float(sys.argv[5])
		ya = float(sys.argv[6])
		t = 100
	else:
		print usage()
		sys.exit(1)
	print "Requesting move to x=%s, y=%s, z=%s, r/p/y=%s/%s/%s, move duration = %s seconds"%(x,y,z,r,p,ya,t/10)
	if oint_client(x,y,z,r,p,ya,t):
		print "move finished"
	
