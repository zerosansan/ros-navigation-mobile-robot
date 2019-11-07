#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

class KeyPublisher:

    # ROS
    NODE_NAME = 'keyboard_driver'
    PUB_RATE = 100

    def __init__(self):
    
        # ROS Node initialization
        rospy.init_node(KeyPublisher.NODE_NAME, disable_signals = True)
        rospy.logwarn("%s node started" % KeyPublisher.NODE_NAME)

        # Publishers
        self.__key_pub = rospy.Publisher('keys', String, queue_size = 1)
    
    def spin(self):
        """
            Function: Runs continous loop with specified rate in Hz
            
            Within the main function, termios library captures raw keystrokes.
            
            Please read "Programming Robots with ROS" by Quigley. The codes here
            are referenced from that book. The main purpose of this functionality
            is to control the robot manually for debugging purposes.
        """
        try:
            old_attr = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            r = rospy.Rate(KeyPublisher.PUB_RATE)
            
            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()
                except KeyboardInterrupt:
                    break
        
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

        except rospy.ROSInterruptException: pass
        
    def main(self):
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            self.__key_pub.publish(sys.stdin.read(1))

if __name__ == '__main__':
    try:
        keypub = KeyPublisher()
        keypub.spin()

    except rospy.ROSInterruptException: pass
