#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class KeysToTwist:

    # ROS
    NODE_NAME = 'keys_to_twist'
    PUB_RATE = 20
    
    # Keyboard input to velocity (in m/s)
    KEY_MAPPING = {\
        'w': [0, 0.25],\
        ' ': [0, 0],\
        'a': [1.2, 0],\
        's': [0, -0.25],\
        'd':[-1.2, 0]
    }
    
    def __init__(self):
    
        # ROS Node initialization
        rospy.init_node(KeysToTwist.NODE_NAME, disable_signals = True)
        rospy.logwarn("%s node started" % KeysToTwist.NODE_NAME)
        
        # Publishers
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        # Subscribers
        rospy.Subscriber('keys', String, self._keys_callback)

        self.__last_twist = None

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
            r = rospy.Rate(KeysToTwist.PUB_RATE)

            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()
                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def main(self):
        self._twist_pub.publish(self.__last_twist)

    def keys_callback(self, msg):
        if (len(msg.data) == 0 or not KeysToTwist.KEY_MAPPING.has_key(msg.data[0])):
            return -1

        vels = KeysToTwist.KEY_MAPPING[msg.data[0]]
        self.last_twist.linear.x = vels[1]
        self.last_twist.angular.z = vels[0]

if __name__ == '__main__':
    try:
        keystotwist = KeysToTwist()
        keystotwist.spin()

    except rospy.ROSInterruptException: pass
