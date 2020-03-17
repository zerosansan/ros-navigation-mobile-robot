#!/usr/bin/env python
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

KEY_PUB_NAME = 'keys'

class KeyPublisher:
    def __init__(self):
        # ROS Node initialization
        rospy.init_node('keyboard_driver', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # Publishers
        self.key_pub = rospy.Publisher(KEY_PUB_NAME, String, queue_size = 1)

        # Subscribers
        # None

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
            r = rospy.Rate(100)

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
            self.key_pub.publish(sys.stdin.read(1))

if __name__ == '__main__':
    try:
        keypub = KeyPublisher()
        keypub.spin()

    except rospy.ROSInterruptException: pass
