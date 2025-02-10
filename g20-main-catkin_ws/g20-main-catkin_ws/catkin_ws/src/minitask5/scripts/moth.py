#!/usr/bin/env python3
import rospy
from mover import Mover
from laser_scanner import Laser_scanner
from nav_msgs.msg import Odometry
import cv2, cv_bridge # type: ignore
import numpy as np # type: ignore
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class Moth:
    def __init__(self):
        # Initialize ROS node
        #rospy.init_node('moth_behavior', anonymous=True)

        # Class references
        self.mover = Mover()
        self.laser_scanner = Laser_scanner()

        # State Variables
        self.detected_objects = []
        self.image = None
        self.robot_pose = None
        self.move_to_object = False

        # Thresholds and parameters
        self.min_area_red = 40000
        self.min_area_green = 11500
        self.dead_zone = 30

        # ROS Publishers and Subscribers
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        self.marker_id = 0
        self.markers = []
        self.rate = rospy.Rate(10)  # 10 Hz
        self.moth_is_complete = False

    def image_callback(self, msg):
        """Update the latest received image."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def odom_callback(self, msg):
        """Update the robot's current pose using odometry data."""
        self.robot_pose = msg.pose.pose  # Extract pose from odometry message

    def detect_object(self, is_irl):
        """Detect objects in the image and process their positions."""
        if self.image is None:
            rospy.logwarn("No image received yet.")
            return None, None

        # Convert to HSV and apply masks
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        if is_irl:
            target1_mask = cv2.inRange(hsv, np.array([140, 50, 50]), np.array([165, 255, 255])) # pink
            target2_mask = cv2.inRange(hsv, np.array([25, 70, 70]), np.array([35, 255, 255]))
        
        else:
            # Define color ranges

            # Green
            target1_mask = cv2.inRange(hsv, np.array([30, 50, 30]), np.array([90, 255, 255]))

            # Red
            target2_mask_1 = cv2.inRange(hsv, np.array([0, 100, 30]), np.array([10, 255, 255]))
            target2_mask_2 = cv2.inRange(hsv, np.array([170, 150, 30]), np.array([180, 255, 255]))
            target2_mask = cv2.bitwise_or(target2_mask_1, target2_mask_2)
        
        cv2.waitKey(1)  # Wait briefly to update the window


        # Detect contours
        colors = {"green": target1_mask, "red": target2_mask}
        for color, mask in colors.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if color == 'red' and area < self.min_area_red:
                    continue
                elif color == 'green' and area < self.min_area_green:
                    continue

                # Object found; compute center and bounding box
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2

                # Visualize object
                # cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.circle(self.image, (cx, cy), 5, (255, 255, 255), -1)

                #rospy.loginfo(f"{color.capitalize()} object detected at (x: {cx}, y: {cy}), area: {area}")
                cv2.imshow("Detected Object", self.image)
                cv2.waitKey(1)

                # Return horizontal error and detected color
                return cx - self.image.shape[1] // 2, color

        #rospy.loginfo("No objects detected.")
        return None, None

    def move_towards_object(self, error):
        """Control robot to move towards the detected object."""
        if abs(error) > self.dead_zone:
            angular_z = -float(error) / 2000
            self.mover.set_moth_twist_z(angular_z)
        else:
            self.mover.set_moth_twist_z(0)

        # Set linear velocity and publish twist
        self.mover.set_twist_linear_x(0.2, self.mover.get_moth_twist())
        self.mover.cmd_vel_publish_twist(self.mover.get_moth_twist())

    def stop_robot(self):
        """Stop the robot's movement."""
        self.mover.set_twist_linear_x(0.0, self.mover.get_empty_twist())
        self.mover.cmd_vel_publish_twist(self.mover.get_empty_twist())

    def show_marker(self, color):
        """Publish a square marker in front of the robot based on the detected color."""
        color_map = {"green": ColorRGBA(0.0, 1.0, 0.0, 0.8), "red": ColorRGBA(1.0, 0.0, 0.0, 0.8)}
        marker_color = color_map.get(color, ColorRGBA(1.0, 1.0, 1.0, 0.8))  # Default to white if unknown color

        marker = Marker(
            type=Marker.CUBE,
            id = self.marker_id,
            lifetime=rospy.Duration(0),  # Marker persists until explicitly deleted
            pose=Pose(
                Point(0.5, 0.0, 0.0),  # Position 0.5m ahead in robot's local frame
                Quaternion(0, 0, 0, 1)  # No rotation
            ),
            scale=Vector3(0.2, 0.2, 0.01),  # Marker dimensions
            header=Header(frame_id='base_link', stamp=rospy.Time.now()),  # Frame relative to the robot
            color=marker_color
        )

        self.marker_id += 1
        self.markers.append(marker)
        for marker in self.markers:
            self.marker_pub.publish(marker)

    def object_counter(self):
        green = 0
        red = 0
        for object in self.detected_objects:
            if object == "green":
                green += 1
            elif object == "red":
                red += 1
        print(f"Detected {green} green objects and {red} red objects")

    def run(self, is_irl):
        """Main loop for detecting objects and moving towards them."""
        while not rospy.is_shutdown():
            error, color = self.detect_object(is_irl)  # Detect objects and compute error
            if error is not None:
                front_distance = self.laser_scanner.get_front_distance()
                if front_distance <= 0.5:  # If close enough, show marker and stop
                    rospy.loginfo(f"Close to {color} object. Drawing marker and stopping.")
                    self.stop_robot()
                    self.show_marker(color)
                    self.detected_objects.append(color)
                    self.object_counter()
                    self.moth_is_complete = True
                    #rospy.loginfo("True")
                    return
                    #rospy.signal_shutdown("Got to object")
                else:  # Otherwise, move towards the object
                    #rospy.loginfo("Moving towards object")
                    self.move_towards_object(error)
            else:
                #rospy.loginfo("Moth.run()")
                error, color = self.detect_object(is_irl)  # Detect objects and compute error
                if error is None:
                    self.stop_robot()  # Stop if no objects detected
                
            self.rate.sleep()

    def get_moth_is_complete(self):
        return self.moth_is_complete
    
    def set_moth_is_complete(self, moth_is_complete):
        self.moth_is_complete = moth_is_complete
# if __name__ == '__main__':
#     try:
#         Moth().run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Moth behavior node terminated.")
