#!/usr/bin/env python

import rospy
import pigpio
import utils

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Controller:

    # ROS
    NODE_NAME = 'robot_controller'
    PUB_RATE = 10

    # PWM settings
    STOP_PULSE_WIDTH = 1500
    FREQUENCY = 50
    
    # GPIO pins
    PINS = {\
        'motor_left':18,\
        'motor_right':17
    }
    
    # Wheel specification (in meters)
    WHEEL_RADIUS = 0.075
    AXLE_LENGTH = 0.392
    
    def __init__(self):
    
        self.__wheel_radius = Controller.WHEEL_RADIUS
        self.__axle_length = Controller.AXLE_LENGTH
        
        # Motor PWM setup
        self.__left_wheel = pigpio.pi()
        self.__left_wheel.set_PWM_frequency(\
            Controller.PINS['motor_left'], Controller.FREQUENCY)
        self.__left_wheel.set_servo_pulsewidth(\
            Controller.PINS['motor_left'], Controller.STOP_PULSE_WIDTH)
            
        self.__right_wheel = pigpio.pi()
        self.__right_wheel.set_PWM_frequency(\
            Controller.PINS['motor_right'], Controller.FREQUENCY)
        self.__right_wheel.set_servo_pulsewidth(\
            Controller.PINS['motor_right'], Controller.STOP_PULSE_WIDTH)
            
        # Controller internal data
        self.__vel_x = 0
        self.__vel_y = 0
        self.__vel_th = 0
        self.__vel_left = 0
        self.__vel_right = 0

        # Robot motor speed tuning
        self.__VLF_MOD = (27.7 / 150.0)
        self.__VLB_MOD = (36.0 / 150.0)
        self.__VRF_MOD = (33.2 / 150.0)
        self.__VRB_MOD = (32.0 / 150.0)
        
        # ROS node initialization
        rospy.init_node(Controller.NODE_NAME, disable_signals = True)
        rospy.logwarn("%s node started" % Controller.NODE_NAME)

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self._twist_callback)

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
            r = rospy.Rate(Controller.PUB_RATE)
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
        self._move_left_wheel(self.__vel_left)
        self._move_right_wheel(self.__vel_right)

    def _twist_callback(self, msg):
        """
            Function: Do something for every message received through 
            subscriber.
        """
        self.__vel_x = msg.linear.x
        self.__vel_y = msg.linear.y
        self.__vel_th = -msg.angular.z

        twist_to_vel = utils.twist_to_velocity(self.__vel_x, self.__vel_y,\
            self.__vel_th, self.__axle_length)
        
        self.__vel_left = twist_to_vel['left']
        self.__vel_right = twist_to_vel['right']

    def _move_left_wheel(self, vel):
        """
            Function: turns left wheel motor

            Inputs: left wheel velocity converted from Navigation stack's 
            Twist message.
            Output: leftwheel turning
        """      
        _pulse = 0
        
        if (vel > 0):
            _pulse = utils.velocity_to_pulse(vel,\
                Controller.WHEEL_RADIUS, self.__VLF_MOD)
        else:
            _pulse = utils.velocity_to_pulse(vel,\
                Controller.WHEEL_RADIUS, self.__VLF_MOD)
        
        self.__right_wheel.set_servo_pulsewidth(Controller.PINS['motor_left'],\
            Controller.STOP_PULSE_WIDTH - _pulse)

    def _move_right_wheel(self, vel):
        """
            Function: turns right wheel motor

            Inputs: right wheel velocity converted from Navigation stack's
            Twist message.
            Output: right wheel turning
        """
        _pulse = 0
        
        if (vel > 0):
            _pulse = utils.velocity_to_pulse(vel,\
                Controller.WHEEL_RADIUS, self.__VRF_MOD)
        else:
            _pulse = utils.velocity_to_pulse(vel,\
                Controller.WHEEL_RADIUS, self.__VRF_MOD)
        
        self.__right_wheel.set_servo_pulsewidth(Controller.PINS['motor_right'],\
            Controller.STOP_PULSE_WIDTH - _pulse)

if __name__ == '__main__':
    try:
        ctrlr = Controller()
        ctrlr.spin()
        
    except rospy.ROSInterruptException: pass
