#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class Laser_scanner:
    def __init__(self):
        rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)
        self.laser_data = LaserScan()
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf') 

    def get_front_distance(self):
        return self.front_distance
    
    # Laser scan data    
    def laser_scan_callback(self, msg: LaserScan):
        self.laser_data = msg

        # Front data
        self.front_distances = []

        # 25 degree front view
        for i in range(25):
            front_val_1 = msg.ranges[i]
            front_val_2 = msg.ranges[359 - i]

            if(front_val_1 == 0.0):
                front_val_1 = 3.5

            if(front_val_2 == 0.0):
                front_val_2 = 3.5

            self.front_distances.append(front_val_1)
            self.front_distances.append(front_val_2)

        self.front_distance = min(self.front_distances)

        #RIGHT DATA
        self.right_distances = []

        for i in range(22):
            right_val_1 = msg.ranges[270 + i]
            right_val_2 = msg.ranges[269 - i]

            # Set any 0
            if(right_val_1 == 0.0):
                right_val_1 = 3.5
            if(right_val_2 == 0.0):
                right_val_2 = 3.5

            self.right_distances.append(right_val_1)
            self.right_distances.append(right_val_2)

        self.right_distance = min(self.right_distances)

        self.left_distance = msg.ranges[90]

        return self.front_distance, self.left_distance, self.right_distance


