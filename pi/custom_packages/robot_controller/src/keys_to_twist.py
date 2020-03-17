#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w': [0, 0.25],
               ' ': [0, 0],
               'a': [1.2, 0],
               's': [0, -0.25],
               'd': [-1.2, 0]
              }

TWIST_PUB_NAME = 'cmd_vel'
KEY_SUB_NAME = 'keys'

class KeysToTwist:
    def __init__(self):
        # ROS Node initialization
        rospy.init_node('keys_to_twist', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # Publishers
        self.twist_pub = rospy.Publisher(TWIST_PUB_NAME, Twist, queue_size = 1)

        # Subscribers
        rospy.Subscriber(KEY_SUB_NAME, String, self.keys_callback)

        self.last_twist = None

    def spin(self):
        """
            Function: Runs continous loop with specified rate in Hz

            Within the main function, the key presses are mapped velocity
            values and then published through the publisher.

            Please read "Programming Robots with ROS" by Quigley. The codes here
            are referenced from that book. The main purpose of this functionality
            is to control the robot manually for debugging purposes.
        """
        try:
            self.last_twist = Twist() # Initialized to zero
            r = rospy.Rate(20)

            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()

                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def main(self):
        self.twist_pub.publish(self.last_twist)

    def keys_callback(self, msg):
        if (len(msg.data) == 0 or not key_mapping.has_key(msg.data[0])):
            return -1

        vels = key_mapping[msg.data[0]]
        self.last_twist.linear.x = vels[1]
        self.last_twist.angular.z = vels[0]

if __name__ == '__main__':
    try:
        keystotwist = KeysToTwist()
        keystotwist.spin()

    except rospy.ROSInterruptException: pass
