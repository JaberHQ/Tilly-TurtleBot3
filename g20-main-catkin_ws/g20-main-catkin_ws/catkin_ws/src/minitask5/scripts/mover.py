#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

class _Pose():
        def __init__(self):
            self.x= 0
            self.y = 0
            self.theta = 0

        def __str__(self) -> str:
            return '_Pose(x:%f, y:%f, theta:%f)' % (self.x, self.y, self.theta)
class Mover:
    def __init__(self):
        #self.rate = rospy.Rate(10)  # 10hz
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rotateTwist = Twist()
        self.rotateTwist.angular.z = math.radians(25)
        self.forward_x = Twist()
        self.forward_x.linear.x = 0.7
        self.mothTwist = Twist()
        self.mothTwist.linear.x = 0.2
        self.mothTwist.angular.z = 0.2
        self.pose = _Pose()
        self.start_pose = _Pose()
        self.delta_theta = 0
        self.delta_x = 0
        self.delta_y = 0
        self.isTurning = False
        self.turn_direction = 25
        self.emptyTwist = Twist()
    
    def set_twist_linear_x(self, value, twist):
        twist.linear.x = value

    def set_twist_angular_z(self, value, twist):
        twist.angular.z = value

    def cmd_vel_publish_twist(self, Twist):
        self.cmd_vel_pub.publish(Twist)

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

    def set_delta_theta(self, delta_theta):
        self.delta_theta = delta_theta

    def set_delta_x(self, delta_x):
        self.delta_x = delta_x
    
    def set_delta_y(self, delta_y):
        self.delta_y = delta_y

    def get_delta_theta(self):
        self.delta_theta = self.pose.theta - self.start_pose.theta
        return self.delta_theta
    
    def get_delta_x(self):
        return self.delta_x
    
    def get_delta_y(self):
        return self.delta_y

    def get_is_turning(self):
        return self.isTurning
    
    def set_is_turning(self, isTurning):
        self.isTurning = isTurning

    def set_forward_x(self, forward_x):
        self.forward_x = forward_x

    def get_forward_x(self):
        return self.forward_x
    
    def set_moth_twist_x(self, moth_twist_x):
        self.mothTwist.linear.x = moth_twist_x

    def set_moth_twist_z(self, moth_twist_z):
        self.mothTwist.angular.z = moth_twist_z

    def get_moth_twist(self):
        return self.mothTwist
    
    def get_rotate_twist(self):
        return self.rotateTwist
    
    def get_turn_direction(self):
        return self.turn_direction
    
    def flip_direction(self):
        self.rotateTwist.angular.z *= 1

    def reset_states(self):
        self.forward_x.linear.x = 0.2
        self.rotateTwist.angular.z = math.radians(25)
        self.mothTwist.linear.x = 0.2
        self.mothTwist.angular.z = 0.2

    def get_empty_twist(self):
        return self.emptyTwist
    
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_rotate_twist(self):
        self.cmd_vel_publish_twist(self.rotateTwist)

    def reset_delta_theta(self):
        self.delta_theta = 0
        
    def get_rotate_twist_angular_z(self):
        return self.rotateTwist.angular.z
            
