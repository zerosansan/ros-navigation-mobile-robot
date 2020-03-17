#!/usr/bin/env python
import rospy
import pigpio
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

pins = {'motor_left':18,
        'motor_right':17
        }

# PWM settings
STOP_WIDTH = 1500
FREQUENCY = 50

# pigpio initialization for PWM
LEFT_WHEEL = pigpio.pi()
RIGHT_WHEEL = pigpio.pi()

TWIST_SUB_NAME = '/cmd_vel'

class Controller:
    def __init__(self,
        motor_left_pin = pins['motor_left'],
        motor_right_pin = pins['motor_right'],
        width = STOP_WIDTH, freq = FREQUENCY):

        self.freq = freq
        self.width = width

        # GPIO pins setup
        self.motor_left_pin = motor_left_pin
        self.motor_right_pin = motor_right_pin

        # Motor PWM setup
        LEFT_WHEEL.set_PWM_frequency(self.motor_left_pin, self.freq)
        LEFT_WHEEL.set_servo_pulsewidth(self.motor_left_pin, self.width)
        RIGHT_WHEEL.set_PWM_frequency(self.motor_right_pin, self.freq)
        RIGHT_WHEEL.set_servo_pulsewidth(self.motor_right_pin, self.width)

        # ROS node initialization
        rospy.init_node('robot_controller', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)

        # Controller internal data
        self.vel_x = 0
        self.vel_y = 0
        self.vel_th = 0
        self.vel_left = 0
        self.vel_right = 0

        # Robot wheel specification (in meters)
        self.wheel_radius = 0.075
        self.axle_length = 0.392

        # Robot motor speed tuning
        self.vel_left_fwd_constant = (27.7 / 150.0)
        self.vel_right_fwd_constant = (33.2 / 150.0)
        self.vel_left_bwd_constant = (36.0 / 150.0)
        self.vel_right_bwd_constant = (32.0 / 150.0)

        # Publishers
        # None

        # Subscribers
        rospy.Subscriber(TWIST_SUB_NAME, Twist, self.twist_callback)

    def spin(self):
        """
            Function: Runs continuous loop with specified rate in Hz

            Within the main function, the twist message from Navigation stack
            is converted to pulse widths to control the velocity of the robot.

            Twist message is a velocity command expressed in velocity of x, y
            and theta (vx, vy, vth). In this node, we converted (vx, vy, vth)
            into left and right wheel velocity.
        """
        try:
            r = rospy.Rate(10)

            while not rospy.is_shutdown():
                try:
                    self.main()
                    r.sleep()

                except KeyboardInterrupt:
                    break

        except rospy.ROSInterruptException: pass

    def main(self):
        """
            Function: moves the robot around
        """
        self.move_left_wheel()
        self.move_right_wheel()

    def twist_callback(self, msg):
        """
            Function: Do something for every message received through subscriber
        """
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.vel_th = -msg.angular.z

        self.twist_to_velocity()

    def twist_to_velocity(self):
        """
            Function: convert Twist message to velocity commands

            For every Twist message received i.e velocity x, y and theta,
            convert to velocity of left and right wheel
        """
        if (self.vel_x == 0):
            # turning
            if (self.vel_th < 0):
                self.vel_left = self.vel_th * (self.axle_length / 2) + \
                    (self.vel_th * (self.axle_length / 2) * (33.3 / 100))
                self.vel_right = (-1) * ((self.vel_th * (self.axle_length / 2))\
                    + (self.vel_th * (self.axle_length / 2) * (15.5 / 100)))

            if (self.vel_th > 0):
                self.vel_left = self.vel_th * (self.axle_length / 2)
                self.vel_right = (-1) * ((self.vel_th * (self.axle_length / 2))\
                    + (self.vel_th * (self.axle_length / 2) * (20.1 / 100)))

            if (self.vel_th == 0):
                self.vel_left = self.vel_right = self.vel_th

        if (self.vel_th == 0):
            # forward or reverse
            if (self.vel_x > 0):
                self.vel_left = self.vel_x
                self.vel_right = self.vel_x + (self.vel_x * (5.5 / 100))

            if (self.vel_x < 0):
                self.vel_left = self.vel_x + (self.vel_x * (27.1 / 100))
                self.vel_right = self.vel_x + (self.vel_x * (21.3 / 100))

            if (self.vel_x == 0):
                self.vel_left = self.vel_right = self.vel_x
        else:
            # moving doing arcs
            self.vel_left = self.vel_x + self.vel_th * (self.axle_length / 2)
            self.vel_right = self.vel_x - self.vel_th * (self.axle_length / 2)

    def move_left_wheel(self):
        """
            Function: turns left wheel motor

            Inputs: left wheel velocity converted from Nav stack's Twist message
            Output: leftwheel turning
        """
        m = None
        if (self.vel_left > 0):
            m = self.vel_left / (self.vel_left_fwd_constant * \
                ((2 * math.pi)/ 60) * self.wheel_radius)
            LEFT_WHEEL.set_servo_pulsewidth(self.motor_left_pin, STOP_WIDTH - m)
        else:
            m = self.vel_left / (self.vel_left_bwd_constant * \
                ((2 * math.pi)/ 60) * self.wheel_radius)
            LEFT_WHEEL.set_servo_pulsewidth(self.motor_left_pin, STOP_WIDTH - m)

    def move_right_wheel(self):
        """
            Function: turns right wheel motor

            Inputs: right wheel velocity converted from Nav stack's Twist message
            Output: right wheel turning
        """
        m = None
        if (self.vel_right > 0):
            m = self.vel_right / (self.vel_right_fwd_constant * \
                ((2 * math.pi)/ 60) * self.wheel_radius)
            RIGHT_WHEEL.set_servo_pulsewidth(self.motor_right_pin, \
                STOP_WIDTH - m)
        else:
            m = self.vel_right / (self.vel_right_bwd_constant * \
                ((2 * math.pi)/ 60) * self.wheel_radius)
            RIGHT_WHEEL.set_servo_pulsewidth(self.motor_right_pin, \
                STOP_WIDTH - m)

    def move_robot(self):
        """
            Function: moves the robot around
        """
        self.move_left_wheel()
        self.move_right_wheel()

if __name__ == '__main__':
    try:
        ctrlr = Controller()
        ctrlr.spin()

    except rospy.ROSInterruptException: pass
