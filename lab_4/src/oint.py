#!/usr/bin/env python

import numpy as np
import rospy
import sys
sys.path.append('/home/marcin/catkin_ws/src/lab_4/srv')
from tf.transformations import quaternion_from_euler
from lab_4.srv import oint_srv
from geometry_msgs.msg import PoseStamped

rospy.init_node('oint_server')
pub = rospy.Publisher('oint', PoseStamped, queue_size = 10)
position=[2, 0, -0.25, 0, 0 ,0]
def oint_server():
    s= rospy.Service('oint_srv',oint_srv, oint_handle)
    rospy.spin()
def oint_handle(srv):
    r=rospy.Rate(20)
    global position
    desired_position = srv.desired_position
    t=srv.mov_duration
    ti=0
    current_position = position[:]
    while ti<=t:
        for i in range(6):
            current_position[i] = position[i]+(desired_position[i]-position[i])/t*ti
        qua = quaternion_from_euler(current_position[4],current_position[3],current_position[5])
        p = PoseStamped()
        p.header.frame_id = 'base'
        p.header.stamp = rospy.get_rostime()
        #print current_position
        p.pose.position.x = current_position[0]
        p.pose.position.y = current_position[1]
        p.pose.position.z = current_position[2]
        p.pose.orientation.x = qua[0]
        p.pose.orientation.y = qua[1]
        p.pose.orientation.z = qua[2]
        p.pose.orientation.w = qua[3]
        pub.publish(p)
        ti+=0.5
        r.sleep()
    position = current_position[:]
    return True
if __name__ == "__main__":
    oint_server()
    
