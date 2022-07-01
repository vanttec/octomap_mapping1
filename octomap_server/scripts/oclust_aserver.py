#! /usr/bin/env python3
import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import sys

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from geometry_msgs.msg import Pose
from octomap_server.msg import oclustAction
# from nav_v1 import uuv_instance
from scripts.uuv_octomap import UUVOctomap
import math

class oclustServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('oclust', oclustAction, self.execute, False)
    self.server.start()
    self.isrot = False
    self.ismov = False
    self.waypointstatus = None
    self.waypointtarget = None
    self.walk_control = 0 
    self.th = None
    self.ned_x = None
    self.ned_y = None
    self.ned_z = None
    self.yaw = None
    self.uuvocto = UUVOctomap()
    rospy.loginfo("oclust server loaded OK")
    rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)


  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    
    
    r = rospy.Rate(10)

    self.uuvocto.uuvoctomap()
    self.server.set_succeeded()
  

  def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z







if __name__ == '__main__':
  rospy.init_node('oclust_server')
  server = oclustServer()
  rospy.spin()