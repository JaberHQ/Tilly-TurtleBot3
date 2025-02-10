#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import behavior

class Follower:
    def __init__(self):
        rospy.init_node('follower')
        self.rate = rospy.Rate(10)  # 10hz
        self.behavior = behavior.Behavior()

    def run(self):

        forceQuit = False # Set flag
        while not rospy.is_shutdown() and forceQuit == False:
            forceQuit = self.behavior.fsm(forceQuit)
            #self.behavior.stopRobot()

        if forceQuit is True:
            self.behavior.stopRobot()
            #xit()

        rospy.spin() 

