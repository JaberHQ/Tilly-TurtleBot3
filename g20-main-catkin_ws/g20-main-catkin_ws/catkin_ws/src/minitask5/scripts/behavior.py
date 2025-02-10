#!/usr/bin/env python3
import rospy
from mover import Mover
from laser_scanner import Laser_scanner
from patrol import Move_Base_Sequence
import cv2, cv_bridge # type: ignore
import math
import random
from sensor_msgs.msg import Image
import numpy as np # type: ignore
from moth import Moth
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
# from visualization_msgs.msg import Marker
# from std_msgs.msg import Header, ColorRGBA

class Behavior:
    def __init__(self):

        # ROS defaults
        rospy.init_node('behavior', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Class references
        self.mover = Mover()
        self.laser_scanner = Laser_scanner()
        self.move_base_sequence = Move_Base_Sequence()
        self.moth = Moth()

        # Behaviour states
        self.states = ["moth", "obstacle_avoidance", "waypoints", "spin"] # Moth goes towards object
        self.current_state = self.states[0]

        # Booleans
        self.is_turning = False

        # Int/floats
        self.turn_degrees = 30
        self.check_for_moth_degree = 20
        self.is_irl = self.move_base_sequence.get_is_irl()

        
    def get_current_state(self):
        return self.current_state
    
    def set_current_state(self, current_state):
        self.current_state = current_state

    def stopRobot(self):
        self.mover.stop_robot()

    def waypoints_actions(self):
        while self.get_current_state() == 'waypoints':

            self.move_base_sequence.move_to_single_waypoint()
            self.moth.run(self.is_irl)
            self.set_current_state('spin')
        self.rate.sleep()
        #self.set_current_state('spin')

    # Behaviour in obstacle avoidance state
    def obstacle_avoidance_behavior(self):

        # Choose random direction 
        randomiser = random.choice([True,False])
        if(randomiser):
            self.mover.flip_direction()

        self.mover.publish_rotate_twist()

    def spin_actions(self):
        self.scan180()
        #if not self.moth.get_moth_is_complete():

    # Check if bot should go to moth state
    def check_for_moth(self):
        colour = self.moth.detect_object(self.is_irl)
        if colour != (None,None) and self.moth.image is not None:
            rospy.loginfo("Found a new object...")
            rospy.loginfo(colour)
            self.set_current_state('moth')
            return True
        return False
       
    # A 180 turn to scan the environment for any objects
    def scan180(self): 
        self.is_turning = True
        rospy.loginfo("Scanning environment...")
        
        delta_theta = 0 
        temp = self.check_for_moth_degree

        while self.is_turning is True:
            if delta_theta < self.turn_degrees: # Turn until 180 degrees  
                delta_theta += self.mover.get_rotate_twist_angular_z() # Rotate  
                self.mover.publish_rotate_twist()

                if delta_theta > temp:
                    if self.check_for_moth():
                        rospy.loginfo("Moth Found")
                        self.stopRobot()
                        self.set_current_state('moth')
                        self.moth.run(self.is_irl)
                        self.is_turning = False
                        return


            else:
                rospy.loginfo("Spin goal reached")
                self.stopRobot()
                #self.moth.run()
                self.set_current_state('waypoints')
                self.is_turning = False

            self.rate.sleep()


    def fsm(self):    
        
        rospy.loginfo(self.is_irl)
        # Moth State
        if self.get_current_state() == 'moth':
            rospy.loginfo("Moth state")
            self.moth.run(self.is_irl)
            if self.moth.get_moth_is_complete():
                self.moth.set_moth_is_complete(False)
                self.set_current_state('spin')
       
            

        # Spin State
        elif self.get_current_state() == 'spin':
            rospy.loginfo("Spin state")
            self.spin_actions()
            

        # Waypoints State
        elif self.get_current_state() == 'waypoints':
            rospy.loginfo("Waypoints state")
            self.waypoints_actions()

        else:
            self.set_current_state('spin')

    def run(self):
        self.set_current_state('spin')
        self.set_current_state('moth')
        while not rospy.is_shutdown():
            self.fsm()
        self.rate.sleep()

if __name__ == '__main__':
    try:
        Behavior().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
