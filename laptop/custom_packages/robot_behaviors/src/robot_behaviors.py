#! /usr/bin/env python
import rospy
import actionlib
import move_base_msgs.msg
import actionlib_tutorials.msg
import cv2
import time
import math
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from playsound import playsound

ODOM_PUB_NAME = '/odom'
GOAL_POS_PUB_NAME = '/face_detector/goal_position'
RGB_IMG_PUB_NAME = '/camera/rgb/image_color'
FACE_DET_PUB_NAME = '/face_detector/is_there_face'

class Photographer:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('robot_behavior', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # Robot "moving to goal position" internal data
        self.client = actionlib.SimpleActionClient("move_base", \
            move_base_msgs.msg.MoveBaseAction)
        self.goal = move_base_msgs.msg.MoveBaseGoal()
        self.odom_x = 0
        self.odom_y = 0
        self.odom_th = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_th = 0

        # Robot "picture taking" internal data
        self.bridge = CvBridge()
        self.cv_image = None
        self.rgb_image = None

        # Robot behavior states
        self.face_detected = False
        self.person_reached = False
        self.picture_taken = False

        # Publishers
        # None

        # Subscribers
        rospy.Subscriber(ODOM_PUB_NAME, Odometry, self.odom_callback)
        rospy.Subscriber(GOAL_POS_PUB_NAME, Point, self.goal_pose_callback)
        rospy.Subscriber(RGB_IMG_PUB_NAME, Image, self.rgb_channel_callback)
        rospy.Subscriber(FACE_DET_PUB_NAME, Bool, self.face_detect_callback)

    def spin(self):
        """
            Function: Runs continous loop with specified rate in Hz

            Within the main function, the robot uses position of detected
            face as a goal position to traverse to using the navigation
            stack.
        """
        try:
            r = rospy.Rate(20)

            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()

                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def main(self):
        """
            Function: Behavior of the photographer robot
        """
        x = self.goal_x
        y = self.goal_y

        face_detected = self.face_detected
        person_reached = self.person_reached
        picture_taken = self.picture_taken

        if (face_detected == True):
            # go to face
            self.go_to_person(self.goal_x, self.goal_y, self.goal_th)

            if (person_reached == True):
                # take picture
                self.take_picture(self.cv_image)

                if (picture_taken == True):
                    # go to previous location and face opposite direction
                    self.go_to_person(-x, -y, math.pi*2)

        else:
            # explore
            print("no face so explore")

    def odom_callback(self, msg):
        """
            Function: gets current robot odometry
        """
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_th = msg.pose.pose.orientation.z

    def goal_pose_callback(self, msg):
        """
            Function: gets current robot goal position based on detected face
        """
        self.goal_x = msg.x
        self.goal_y = msg.y

    def rgb_channel_callback(self, msg):
        """
            Function: Color image from Kinect sensor will be used to
                        "take pictures" from using OpenCV functions.
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg)

        except CvBridgeError as e:
            print(e)

    def face_detect_callback(self, msg):
        """
            Function: gets status of detected face i.e True/False
        """
        self.face_detected = msg.data

    def take_picture(self, img):
        """
            Function: Robot behavior - take picture

            Using OpenCV, the image message subscribed from one of
            Kinect's image topic is saved and displayed.
        """
        self.take_picture_countdown()
        success = cv2.imwrite('people.png', img)
        self.rgb_image = cv2.imread('people.png')

        if (success):
            rospy.loginfo("The robot has taken a picture")
            cv2.imshow('img', self.rgb_image)
            cv2.waitKey(100)
            self.picture_taken = True
            return True

        else:
            rospy.loginfo("The robot has failed to take a picture")
            return False

    def take_picture_countdown(self):
        """
            Function: plays some sound effect before taking a picture
        """
        playsound('robot-take-picture.wav')
        playsound('countdown-3.wav')
        time.sleep(1)
        playsound('countdown-2.wav')
        time.sleep(1)
        playsound('countdown-1.wav')
        time.sleep(1)
        playsound('camera-sound.wav')

    def go_to_person(self, x, y, th):
        """
            Function: Robot behavior - go to detected face

            This functionality is basically the same as
            manually giving a goal position through rViz.
            The difference is that the (x, z) position of
            the detected face is being given to the navigation
            stack instead through actionlib.
        """
        while (not self.client.wait_for_server(rospy.Duration(10))):
            rospy.loginfo("Waiting for the move_base action server to start")

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.goal.target_pose.pose.position.x = self.odom_x + (x/2)
        self.goal.target_pose.pose.position.y = self.odom_y + y
        self.goal.target_pose.pose.orientation.w = 1
        self.goal.target_pose.pose.orientation.z = self.odom_th + th

        self.client.send_goal(goal)
        success = self.client.wait_for_result(rospy.Duration(60))

        if (success):
            rospy.loginfo("The robot reached goal destination")
            self.person_reached = True
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            self.person_reached = False
            return False

if __name__ == '__main__':
    try:
        p = Photographer()
        p.spin()

    except rospy.ROSInterruptException:
        pass
