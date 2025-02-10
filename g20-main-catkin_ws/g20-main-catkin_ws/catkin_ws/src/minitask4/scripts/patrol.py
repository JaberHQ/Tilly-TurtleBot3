#!/usr/bin/env python3

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class Move_Base_Sequence:
    def __init__(self):
        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')

        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle, axes='sxyz'))))

        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")


        wait = self.client.wait_for_server(rospy.Duration(5.0))

        while self.goal_cnt < 6 and not rospy.is_shutdown():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            self.movebase_client(goal)
            self.goal_cnt += 1
 
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

if __name__ == '__main__':
    try:
        Move_Base_Sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")