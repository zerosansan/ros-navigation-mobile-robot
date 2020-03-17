#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class Transforms:
    def __init__(self):
        # ROS node initialization
        rospy.init_node('transforms_publisher', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # transforms publisher
        self.br = tf.TransformBroadcaster()

    def spin(self):
        """
            Function: Runs continous loop with specified rate in Hz

            Within the main function, the transforms between robot's
            base link and odometry is published

            For our robot specification, the Kinect is mounted 1.0m
            higher than the robot base.
        """
        try:
            r = rospy.Rate(20)

            while not rospy.is_shutdown():
                self.main()
                r.sleep()

        except rospy.ROSInterruptException: pass

    def main(self):
        self.br.sendTransform((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), \
            rospy.Time.now(), "camera_link", "base_link")

if __name__ == '__main__':
    try:
        tfm = Transforms()
        tfm.spin()

    except rospy.ROSInterruptException: pass
