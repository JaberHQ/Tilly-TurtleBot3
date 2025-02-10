#!/usr/bin/env python3
import rospy
import mover
import laser_scanner
import cv2, cv_bridge # type: ignore
import math
import random
from sensor_msgs.msg import Image
import numpy as np # type: ignore


class Behavior:
    def __init__(self):

        # Class references
        self.mover = mover.Mover()
        self.laser_scanner = laser_scanner.Laser_scanner()

        # Behaviour states
        self.states = ["moth", "obstacle_avoidance", "wander"] # Moth goes towards object
        self.current_state = self.states[0]

        # Booleans
        self.moveToObject = False
        self.numFound = False
        self.is_turning = False

        # Int/floats
        self.rand_angle = 0

        # CV Images
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original",1)
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)
        self.M = {}
        self.image = None
        self.circleDrawn = False

    def get_current_state(self):
        return self.current_state

    def set_current_state(self, current_state):
        self.current_state = current_state

    # Behaviour in moth state
    def moth_behavior(self, moveToObject, err):
        dead_zone = 30

        # Calculate moth twist
        if abs(err) > dead_zone:
            temp = -float(err) / 2000
            self.mover.set_moth_twist_z(temp)
        else:
            self.mover.set_moth_twist_z(0)

        # If object is in site, move towards it
        if moveToObject == True:
            self.mover.set_twist_linear_x(0.2, self.mover.get_moth_twist())
            self.mover.cmd_vel_publish_twist(self.mover.get_moth_twist())

        # Otherwise, stop 
        if moveToObject == False:
            self.mover.set_twist_linear_x(0.0, self.mover.get_moth_twist())
            self.mover.cmd_vel_publish_twist(self.mover.get_moth_twist())

    # Behaviour in obstacle avoidance state
    def obstacle_avoidance_behavior(self):

        # Choose random direction 
        randomiser = random.choice([True,False])
        if(randomiser):
            self.mover.flip_direction()

        self.mover.cmd_vel_publish_twist(self.mover.get_rotate_twist())

    # Behaviour in wander state
    def wander_behaviour(self):
        # Calculate a random degree for robot to rotate to
        if not self.numFound:
            self.rand_angle = random.randint(90,120)
            self.numFound = True

        # Get required variables
        delta_thetha = self.mover.get_delta_theta()
        delta_x = self.mover.get_delta_x()
        delta_y = self.mover.get_delta_y()
        self.is_Turning = self.mover.get_is_turning()
        rotateTwist = self.mover.get_rotate_twist()

        max_distance = 1
        # If robot has gone to it's max distance 
        if not self.is_turning and (delta_x >= max_distance or delta_y >= max_distance):
            # Reset it's delta x and y, then set it for rotation 
            self.mover.set_delta_x(0)
            self.mover.set_delta_y(0)
            self.is_turning = True
        
        # Go forward
        elif not self.is_turning:
            temp_twist = self.mover.get_forward_x()
            self.mover.cmd_vel_publish_twist(temp_twist)

        # Reset roation variables
        elif self.is_turning and delta_thetha >= math.radians(self.rand_angle):
            self.mover.set_delta_theta(0)
            self.numFound = False
            self.is_turning = False

        # Rotate robot
        elif self.is_turning:
            self.mover.cmd_vel_publish_twist(rotateTwist)

    # Set moth state 
    def set_moth(self, moveToObject, err):
        front_distance = self.laser_scanner.get_front_distance()
        self.moveToObject = False if front_distance <= 0.3 else moveToObject
        self.moth_behavior(self.moveToObject, err)

    # Moth state actions
    def moth_actions(self,err):
        if self.get_current_state() == 'moth':
            self.set_moth(True, err)
            self.mover.reset_states()

    # Wander state actions
    def wander_actions(self):
        self.wander_behaviour()
        self.mover.reset_states()

    # Obstacle avoidance actions
    def obstacle_avoidance_actions(self):
        self.obstacle_avoidance_behavior()
        self.mover.reset_states()

    # If robot needs to obstacle avoid or complete task
    def is_robot_close_to_object(self,flag):
        front_distance = self.laser_scanner.get_front_distance() # Get distance to object
        if front_distance < 0.6:

            # If the robot is in moth state
            if self.get_current_state() == 'moth':
                # Objective complete
                flag = self.reached_moth_objective_actions(flag)
                return flag 
            
            # Otherwise, change to obstacle avoidance 
            else:
                self.set_current_state('obstacle_avoidance')
                print ("obstacle Avoidance")
        return flag
    
    # Called when robot has reached its objective
    def reached_moth_objective_actions(self, flag):

        # Stop robot
        self.mover.cmd_vel_publish_twist(self.mover.get_empty_twist())
        flag = True
        return flag

    # Callback for CV
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # CV actions
    def image_actions(self):
        (h,w) = self.image.shape[:2] # Calculate height and width
        lightest = [90, 100, 50] # Lightest colour HSV
        darkest = [120, 255, 255] # Darkest colour HSV
        image_resized = cv2.resize(self.image, (w//4, h//4))
        cv2.namedWindow("original",1)
        #cv2.imshow("original", image)

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lightest = np.array(lightest, dtype = "uint8")
        darkest = np.array(darkest, dtype = "uint8")
        mask = cv2.inRange(hsv, lightest, darkest)
        output_image = cv2.bitwise_and(self.image,self.image, mask = mask)
        cv2.imshow("thresholded", output_image)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key = cv2.contourArea)
            self.M = cv2.moments(largest_contour)

        cv2.imshow("window", self.image)           
        cv2.waitKey(3)

        return self.image_processing(h,w)
    
    # If color has been detected
    def image_processing(self, h, w):
        if self.M and self.M.get('m00', 0) > 0:
            cx = int(self.M['m10']/self.M['m00'])
            cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.image, (cx,cy), 15, (0,0,255), -1)
            err = cx - w/2

            self.set_current_state('moth')
            print("moth")
            self.moth_actions(err)
            return True
        return False

    def stopRobot(self):
        self.mover.stop_robot()
        
    # Robots finite state machine
    def fsm(self, flag):
        # Wander state     
        if self.get_current_state() == 'wander':
            self.wander_actions()
        
        # Obstacle avoidance state
        if self.get_current_state() == 'obstacle_avoidance':
            self.obstacle_avoidance_actions()

        # Decision between obstacle avoidance state or objective complete 
        flag = self.is_robot_close_to_object(flag)

        # If objective complete, return flag
        if flag is True:
            return flag  

        # Check if robot should enter moth state         
        elif self.image is not None:
            self.image_actions() # Sets moth behaviour
        
        # Set wander state
        else:
            self.set_current_state('wander')
            print("wander")

        return flag

