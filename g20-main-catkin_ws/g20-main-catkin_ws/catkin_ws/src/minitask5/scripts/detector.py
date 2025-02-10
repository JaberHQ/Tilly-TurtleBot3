#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2, cv_bridge
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class Detector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Debug Image", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.image = None
        self.robot_pose = None  # Current pose of the robot
        self.detected_objects = []  # Store detected objects with positions and attributes
        self.min_area = 1500  # Minimum area threshold for contours (adjust as needed)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rotate_twist = Twist()
        self.rotate_twist.angular.z = math.radians(15)
        self.err = None
        self.M = {}
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        self.image_to_moth = None
        rospy.loginfo("Detector node initialized. Waiting for images...")

    def odom_callback(self, msg):
        """Update the robot's current pose using odometry data."""
        self.robot_pose = msg.pose.pose  # Extract the pose from Odometry message

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses."""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return np.sqrt(dx**2 + dy**2)

    def object_already_detected(self, object_position, color):
        """Check if the object is already in memory."""
        proximity_threshold = 1.5  # Meters
        for detected_object in self.detected_objects:
            distance = self.calculate_distance(object_position, detected_object["position"])
            if distance < proximity_threshold and detected_object["color"] == color:
                return True, detected_object
        return False, None

    def show_green_square_in_rviz(self):
        marker = Marker(
            type=Marker.CUBE,
            id=1,
            lifetime=rospy.Duration(0),  # 0 means the marker will persist until explicitly deleted
            pose=Pose(Point(0.5, 0.0, 0.0), Quaternion(0, 0, 0, 1)),  # Position the square 0.5m in front of the robot
            scale=Vector3(0.2, 0.2, 0.01),  # Size of the square (20cm x 20cm x 1cm thick)
            header=Header(frame_id='base_link'),  # Use the robot's base frame
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Green color with 80% opacity
        )
        self.marker_publisher.publish(marker)



    def image_callback(self, msg):
        """Update the latest received image."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def image_actions(self):
        if self.image is None or self.robot_pose is None:
            rospy.logwarn("No image or robot pose received yet.")
            return False

        (h, w) = self.image.shape[:2]

        # Define HSV ranges
        lower_green = np.array([30, 50, 30], dtype="uint8")
        upper_green = np.array([90, 255, 255], dtype="uint8")
        lower_red_1 = np.array([0, 60, 30], dtype="uint8")
        upper_red_1 = np.array([10, 255, 255], dtype="uint8")
        lower_red_2 = np.array([170, 30, 30], dtype="uint8")
        upper_red_2 = np.array([180, 255, 255], dtype="uint8")

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)

        for color, contours in [("green", green_contours)]:
        #for color, contours in [("green", green_contours), ("red", red_contours)]:
            for cont in contours:
                area = cv2.contourArea(cont)
                if area < self.min_area:
                    continue

                x, y, w, h = cv2.boundingRect(cont)
                object_center = Pose()
                object_center.position.x = self.robot_pose.position.x + (x + w // 2) / w  # Example calculation
                object_center.position.y = self.robot_pose.position.y + (y + h // 2) / h  # Example calculation
                object_center.position.z = 0  # Assuming a flat world

                already_detected, detected_object = self.object_already_detected(object_center, color)
                if already_detected:
                    rospy.loginfo(f"Recognized {color} object from memory at "
                                f"({detected_object['position'].position.x}, {detected_object['position'].position.y}).")
                    continue

                # If not detected, add to memory
                self.detected_objects.append({
                    "position": object_center,
                    "color": color,
                    "bounding_box": (x, y, w, h),
                    "area": area
                })

                # Extract HSV values of the detected object's center
                object_hsv = hsv[y + h // 2, x + w // 2]
                rospy.loginfo(f"HSV values of detected {color} object: {object_hsv}")

                rospy.loginfo(f"New {color} object detected with area {area}: "
                            f"at ({object_center.position.x}, {object_center.position.y})")
                cv2.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(self.image, (x + w // 2, y + h // 2), 5, (255, 255, 255), -1)
                cv2.imshow("Detected Object", self.image)
                cv2.waitKey(1)  # Refresh display
                
                self.image_processing(h,w)
                return color

        rospy.loginfo("No new objects detected.")
        return False

    def get_err(self):
        return self.err

    # If color has been detected
    def image_processing(self, h, w):
        if self.M and self.M.get('m00', 0) > 0:
            cx = int(self.M['m10']/self.M['m00'])
            cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.image, (cx,cy), 15, (0,0,255), -1)
            err = cx - w/2
            print("err is "+ err)
            
            return True
        return False
    
    def run(self):
        """Main loop for processing images and odometry."""
        while not rospy.is_shutdown():
            self.show_green_square_in_rviz()
        #     print(len(self.detected_objects))
        #     self.cmd_vel_pub.publish(self.rotate_twist)
        #     if self.image is not None:
        #         result = self.image_actions()
        #         if result:
        #             rospy.loginfo(f"Detected object: {result}")
        #     else:
        #         rospy.logwarn("Waiting for an image...")
            self.rate.sleep()


