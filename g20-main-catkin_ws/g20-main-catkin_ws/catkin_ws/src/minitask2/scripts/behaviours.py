#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math
import random

class Behavior:

    # Init Pose class
    class _Pose():
        def __init__(self):
            self.x = 0
            self.y = 0
            self.theta = 0

        def __str__(self) -> str:
            return '_Pose(x:%f, y:%f, theta:%f)' % (self.x, self.y, self.theta)

    # Laser scan data    
    def laser_scan_callback(self, msg: LaserScan):
        self.laser_data = msg

        #FRONT DATA
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
    
    def __init__(self):
        rospy.init_node('behavior', anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz
        rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)
        self.Pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.laser_data = LaserScan()
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.states = ["obstacle_avoidance", "rh_wall", "rand_walk"]
        self.current_state = self.states[2]
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rotate_twist = Twist()
        self.turn_direction = 25
        self.rotate_twist.angular.z = math.radians(self.turn_direction)
        self.forward_x = Twist()
        self.forward_x.linear.x = 0.1
        self.pose =  self._Pose()
        self.turning = False
        self.delta_theta = 0
        self.delta_x = 0
        self.delta_y = 0
        self.numFound = False
        self.wallFound = None

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
       
        self.delta_theta += math.fabs(math.fabs(yaw) - math.fabs(self.pose.theta))
        self.delta_x += math.fabs(msg.pose.pose.position.x - self.pose.x)
        self.delta_y += math.fabs(msg.pose.pose.position.y - self.pose.y)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def forward(self):
        print(self.front_distance, self.left_distance, self.right_distance)
        twist = Twist()
        twist.linear.x = 0.3
        self.Pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            
            #OBSTACLE AVOIDANCE
            if self.current_state == "obstacle_avoidance":

                #STATE DETECTORS
                if self.front_distance > 0.5:
                    if self.right_distance <= 0.3:
                        self.current_state = "rh_wall"
                        self.pub.publish(Twist())
                        pass
                    self.current_state = "rand_walk"
                    self.pub.publish(Twist())
                    pass

                #ACTIONS
                randomiser = random.choice([True, False])
                if randomiser:
                    self.turn_direction = self.turn_direction * -1
                self.pub.publish(self.rotate_twist)
                pass


            #RH WALL
            elif self.current_state == "rh_wall":

                #STATE DETECTORS
                if self.front_distance <= 0.4:
                    self.wallFound = None
                    self.current_state = "obstacle_avoidance"
                    self.pub.publish(Twist())
                    #print("Obstacle avoidance")
                    pass

                elif self.right_distance > 0.3:
                    self.wallFound = None
                    self.current_state = "rand_walk"
                    self.pub.publish(Twist())
                    #print("Random Walk")
                    pass

                #ACTIONS
                if self.wallFound == None:
                    self.wallFound = self.right_distance
                
                if self.wallFound < self.right_distance:
                    rotate_left = Twist()
                    rotate_left.angular.z = math.radians(5)
                    self.pub.publish(rotate_left)
                    
                elif self.wallFound > self.right_distance:
                    rotate_right = Twist()
                    rotate_right.angular.z = math.radians(-5)
                    self.pub.publish(rotate_right)

                forward = Twist()
                forward.linear.x = 0.1
                self.pub.publish(forward)
                pass
                

                
            
            elif self.current_state == "rand_walk":
                if not self.numFound:
                    rand_num = random.randint(90, 120)
                    self.numFound = True
                
                if self.front_distance <= 0.4:
                    self.current_state = "obstacle_avoidance"
                    self.pub.publish(Twist())
                    self.delta_y = 0
                    self.delta_x = 0
                    self.numFound = False
                    pass

                elif self.front_distance > 0.5:
                    if self.right_distance <= 0.3:
                        self.current_state = "rh_wall"
                        self.pub.publish(Twist())
                        pass
                    
                if(not self.turning and (self.delta_x >= 1 or self.delta_y >= 1)):
                    self.delta_x = 0
                    self.delta_y = 0
                    self.turning = True

                elif(not self.turning):
                    self.pub.publish(self.forward_x)

                elif(self.turning and self.delta_theta >= math.radians(rand_num)):
                    self.delta_theta = 0
                    self.numFound = False
                    self.turning = False

                elif(self.turning):
                    self.pub.publish(self.rotate_twist)
                pass

        self.rate.sleep()

if __name__ == '__main__':
    try:
        Behavior().run()
    except rospy.ROSInterruptException:
        pass
