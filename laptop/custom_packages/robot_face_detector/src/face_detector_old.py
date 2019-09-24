#! /usr/bin/env python
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class FaceDetector:
    def __init__(self):
        self.bridge = CvBridge()
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

        self.m_x = 1.12032 # multiplier constant based on FOV angle for X coordinate.
        self.m_y = 0.84024 # multiplier constant based on FOV angle for Y coordinate.

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')
        self.eye_cascade = cv2.CascadeClassifier('haarcascade_eyes_alt.xml')

    def main(self):
        try:
            rospy.init_node('face_detector', disable_signals = True)
            node_name = rospy.get_name()
            rospy.logwarn("%s node started" % node_name)
            rate = rospy.Rate(1)
            self.listener()

            while not rospy.is_shutdown():
                try:
                    #self.listener()
                    rate.sleep()
                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def listener(self):
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_channel_callback2)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_channel_callback2)
    
        self.goal_pub = rospy.Publisher('/face_detector/goal_position', Point)
        self.face_detect_pub = rospy.Publisher('/face_detector/is_there_face', Bool)

    def rgb_channel_callback(self, msg):
        self.rgb_array = np.frombuffer(msg.data, dtype = np.uint8)
        self.rgb_array = np.reshape(self.rgb_array, (480, 640))
        
        self.haarcascade(self.rgb_array)
        
        if (self.face_detected == True):
            try:
                self.rgb_image_pub.publish(self.bridge.cv2_to_imgmsg(self.rgb_image_roi))
            except CvBridgeError as e:
                print(e)

        cv2.imshow('img rgb', self.rgb_image_roi)
        cv2.waitKey(0)

    def depth_channel_callback(self, msg):
        if (self.face_detected == True):
            self.depth_array = np.frombuffer(msg.data, dtype = np.uint16)
            self.depth_array = np.reshape(self.depth_array, (480, 640))
            
            self.depth_array_roi = img[self.roi_y:self.roi_y+self.roi_h, self.roi_x:self.roi_x+self.roi_w]
            self.depth_image_roi = self.depth_array_roi
            
            cv2.imshow('img depth', self.depth_image)
            cv2.waitKey(0)

    def rgb_channel_callback2(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)

        self.haarcascade(cv_image)

    def depth_channel_callback2(self, msg):
        if (self.face_detected == True):
            self.face_detect_pub.publish(self.face_detected)
            rospy.loginfo("FACE")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg)
            except CvBridgeError as e:
                print(e)
            
            self.depth_image_roi = cv_image[self.roi_y:self.roi_y+self.roi_h, self.roi_x:self.roi_x+self.roi_w]
            self.depthimage_to_pointcloud(self.depth_image_roi)

        else:
            self.face_detect_pub.publish(self.face_detected)
            rospy.loginfo("NO FACE")

    def depthimage_to_pointcloud(self, img):
        i, j = np.mgrid[self.roi_x:self.roi_x + self.roi_w, self.roi_y:self.roi_y + self.roi_h]

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
        point = Point()
        point.x = z
        point.y = -x
        
        self.goal_pub.publish(point)

    def haarcascade(self, img):
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
        fd.main()

    except rospy.ROSInterruptException:
        pass
