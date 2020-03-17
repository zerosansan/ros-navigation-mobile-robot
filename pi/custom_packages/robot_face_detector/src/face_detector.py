#! /usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

FACE_CASCADE_FILE = 'haarcascade_frontalface_alt.xml'
EYE_CASCADE_FILE = 'haarcascade_eyes_alt.xml'
GOAL_PUB_NAME = '/face_detector/goal_position'
FACE_DET_PUB_NAME = '/face_detector/is_there_face'
RGB_IMG_SUB_NAME = '/camera/rgb/image_raw'
DEPTH_IMG_SUB_NAME = '/camera/depth/image_raw'

class FaceDetector:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('face_detector', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # Face detector internal data
        self.bridge = CvBridge()

        self.point = Point()
        self.rgb_array = None
        self.depth_array = None
        self.depth_array_roi = None
        self.rgb_image_roi = None
        self.depth_image_roi = None
        self.goal_pub = None
        self.face_detect_pub = None
        self.face_detected = False
        self.roi_x = 0
        self.roi_y = 0
        self.roi_w = 0
        self.roi_h = 0

        # Kinect sensor multiplier constant
        self.m_x = 1.12032
        self.m_y = 0.84024

        # Classifier files
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_FILE)
        self.eye_cascade = cv2.CascadeClassifier(EYE_CASCADE_FILE)

        # Publishers
        self.goal_pub = rospy.Publisher(GOAL_PUB_NAME, Point)
        self.face_detect_pub = rospy.Publisher(FACE_DET_PUB_NAME, Bool)

        # Subscribers
        rospy.Subscriber(RGB_IMG_SUB_NAME, Image, self.rgb_channel_callback)
        rospy.Subscriber(DEPTH_IMG_SUB_NAME, Image, self.depth_channel_callback)

    def spin(self):
        """
            Function: Runs continuous loop with specified rate in Hz

            Within the main function, the 3D position and status of detected face
            is published through the publisher.
        """
        try:
            r = rospy.Rate(1)

            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()

                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def main(self):
        """
            Function: Publishes the position and status of detected face
        """
        self.face_detect_pub.publish(self.face_detected)
        self.goal_pub.publish(self.point)

    def rgb_channel_callback(self, msg):
        """
            Function: Detects face based on the RGB Image of Kinect

            By subscribing to Kinect's RGB Image channel, we have obtained
            an array of RGB pixels which is needed to be reshaped to properly
            apply Haar's cascade.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)

        except CvBridgeError as e:
            print(e)

        self.haarcascade(cv_image)

    def depth_channel_callback(self, msg):
        """
            Function: Applies the ROI obtained from Haar's cascade
                        to depth image of Kinect

            The ROI of detected face from RGB Image channel is applied to
            the depth image channel to obtain the depth image of detected
            face. This is required to compute the 3D position of the face.
        """
        if (self.face_detected == True):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg)

            except CvBridgeError as e:
                print(e)

            self.depth_image_roi = cv_image[self.roi_y:self.roi_y + self.roi_h,\
                self.roi_x:self.roi_x + self.roi_w]

            # Convert depth image to pointcloud
            self.depthimage_to_pointcloud(self.depth_image_roi)

        else:
            pass

    def depthimage_to_pointcloud(self, img):
        """
            Function: Converts depth image of detected face
                        into pointcloud information (x, y, z)

            Please read my FYP Report for details with regards
            to which paper I referenced this from.

            The output is the position of the face (x, y, z)
            where the x and z components are used as a goal
            position for the robot to move towards to using
            the navigation stack.
        """
        i, j = np.mgrid[self.roi_x:self.roi_x + self.roi_w, \
            self.roi_y:self.roi_y + self.roi_h]

        x = ((j / 640.0) - 0.5) * self.m_x * self.depth_image_roi * 0.001
        y = ((i / 480.0) - 0.5) * self.m_y * self.depth_image_roi * 0.001
        z = self.depth_image_roi * 0.001

        x = np.sum(x) / (self.roi_w * self.roi_h)
        y = np.sum(y) / (self.roi_w * self.roi_h)
        z = np.sum(z) / (self.roi_w * self.roi_h)

        cv2.imshow('rgb', self.rgb_image_roi)
        cv2.imshow('depth', self.depth_image_roi)
        cv2.waitKey(100)

        self.set_goal_pose(x, z)

    def set_goal_pose(self, x, z):
        """
            Function: Publishes the position of detected face
        """
        self.point.x = z
        self.point.y = -x

    def haarcascade(self, img):
        """
            Function: Applies haar's cascade to RGB Image to obtain ROI (face)
        """
        faces = self.face_cascade.detectMultiScale(img, 1.05, 5)
        self.face_detected = False

        for (x, y, w, h) in faces:
            self.face_detected = True
            self.roi_x = x
            self.roi_y = y
            self.roi_w = w
            self.roi_h = h

            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            img_roi = img[y:y+h, x:x+w]
            self.rgb_image_roi = img_roi

            #cv2.imshow('img', img)
            #cv2.waitKey(100)

if __name__ == '__main__':
    try:
        fd = FaceDetector()
        fd.spin()

    except rospy.ROSInterruptException:
        pass
