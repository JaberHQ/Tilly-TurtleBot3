#!/usr/bin/env python3


import rospy
import math
from mover import Mover
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class Move_Base_Sequence:
    def __init__(self):
        #rospy.init_node('move_base_sequence')

        # Parameters from launch files
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        irl = rospy.get_param('move_base_seq/IRL')

        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        self.mover = Mover()
        self.rotate_num = 0
        self.objects = []
        self.isMoth = False
        self.is_irl = irl

        # Append coordinate parameters into array 
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle, axes='sxyz'))))

        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        self.total_waypoints = n
        #print (self.total_waypoints)

    def get_is_irl(self):
        return self.is_irl

    def object_memory(self):
        red = 0
        green = 0
        for col in self.objects:
            if col == 'red':
                red += 1
            elif col == 'green':
                green += 1

        print( f'Current object count: Green: {green}, Red: {red}')
 
    def movebase_client(self, goal):
        self.client.send_goal(goal)

        finished_within_time = self.client.wait_for_result(rospy.Duration(60))

        if not finished_within_time:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
               #self.detect_colour()
                

    def detect_colour(self):
        colour_detected = self.detector.image_actions()
        while colour_detected == False:
            colour_detected = self.detector.image_actions()
            self.objects.append(colour_detected)
            self.object_memory()

    def set_is_moth(self, bool):
        self.isMoth = bool

    def get_is_moth(self):
        return self.isMoth


    def move_to_waypoint(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        wait = self.client.wait_for_server(rospy.Duration(5.0))

        while self.goal_cnt < self.total_waypoints and not rospy.is_shutdown():
           
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            self.movebase_client(goal)
            self.goal_cnt += 1
        
        #exit()

    
    def move_to_single_waypoint(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if self.goal_cnt <= self.total_waypoints:
           
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            self.movebase_client(goal)
            self.goal_cnt += 1

        else:
            rospy.signal_shutdown("All waypoints complete, shutting down...")

    def pause_goal(self):
        self.client.cancel_goal()

    def stop_tracking_goal(self):
        self.client.stop_tracking_goal()
    
    def isEmpty(self):
        if self.goal_cnt == self.total_waypoints:
            return True
        return False
   

    def calculate_objects_world_coords(self):
        pass
    
    def run(self):
        #self.move_to_waypoint()
        #self.detect_colour()
        #self.scan360()
        pass
        
        
        
        
        


