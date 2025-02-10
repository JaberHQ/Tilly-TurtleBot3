#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import tf

class Mover:


   class _Pose():
       def __init__(self):
           self.x= 0
           self.y = 0
           self.z = 0
           self.theta = 0


       def __str__(self) -> str:
           return '_Pose(x:%f, y:%f, z:%f, theta:%f)' % (self.x, self.y, self.z, self.theta)

   def __init__(self):
       rospy.init_node('mover', anonymous=True)
       self.rate = rospy.Rate(10)
       rospy.Subscriber("odom", Odometry, self.odom_callback)
       self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       self.rotate_twist = Twist()
       self.rotate_twist.angular.z = 0.5
       self.forward_x = Twist()
       self.forward_x.linear.x = 0.3
       self.forward_z = Twist()
       self.forward_z.linear.z = 0.3
       self.start = 0
       self.pose =  self._Pose()
       self.start_pose = self._Pose()
       self.moving = True
       self.turning = False

   def odom_callback(self, msg):
       # Get (x, y, theta) specification from odometry topic
       quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
       (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
      
       self.pose.theta = yaw
       self.pose.x = msg.pose.pose.position.x
       self.pose.y = msg.pose.pose.position.y
       self.pose.z = msg.pose.pose.position.z

   def run(self):
       while not rospy.is_shutdown():
           delta_theta = self.pose.theta - self.start_pose.theta
           delta_x = self.pose.x - self.start_pose.x
           #delta_z = self.start_pose.z - self.pose.z
           #print(delta_z)
           #delta_x = self.pose.z - self.start_pose.x
           if self.moving:
               if delta_x < 1:
                   self.pub.publish(self.forward_x)
               else:
                   self.moving = False
                   delta_x = 0
                   self.turning = True
           elif self.turning:
               if delta_theta < 90 * (math.pi/180):
                   self.pub.publish(self.rotate_twist)
                   rospy.loginfo(delta_theta)
               else:
                   #self.pub.publish(Twist())
                   self.pub.publish(self.forward_z)
                   self.turning = False
                   delta_theta = 0
                   self.moving = True
           self.rate.sleep()
          




if __name__ == '__main__':
   try:
       Mover().run()
   except rospy.ROSInterruptException:
       pass
