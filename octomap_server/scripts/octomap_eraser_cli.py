#!/usr/bin/env python

"""Clear a region specified by a global axis-aligned bounding box in stored
OctoMap.

"""

import sys
from time import sleep

import roslib
roslib.load_manifest('octomap_server')
from geometry_msgs.msg import Point
import octomap_msgs.srv
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospy


#SRV_NAME = '/octomap_talker/clear_bbx'
SRV_NAME = '/octomap_talker/reset'

#SRV_INTERFACE = octomap_msgs.srv.BoundingBoxQuery
class OctomapErase:
    def __init__(self):
        rospy.Subscriber("/erasemap",String,self.erase_callback)
        self.erase=""
    def erase_callback(self,msg):
        self.erase = msg.data
    
    def main(self):
        if self.erase=='Activate' :
            service = rospy.ServiceProxy(SRV_NAME, Empty)
            service()

def main():
    rospy.init_node("octomap_control", anonymous=False)
    rate = rospy.Rate(20)
    octomapcontrol = OctomapErase()
    while not rospy.is_shutdown():
        octomapcontrol.main()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass