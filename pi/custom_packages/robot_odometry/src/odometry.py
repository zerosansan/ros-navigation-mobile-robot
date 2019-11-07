#!/usr/bin/env python

import rospy
import math
import RPi.GPIO as GPIO
import time
import tf
import utils
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

class Encoder:

    # ROS
    NODE_NAME = 'encoder_tick_publisher'
    PUB_RATE = 5.5
    
    # GPIO pins
    PINS = {\
        'enc_left_one':23,\
        'enc_left_two':24,\
        'enc_right_one':5,\
        'enc_right_two':6\
    }
    
    # Wheel specification (in meters)
    WHEEL_RADIUS = 0.075
    AXLE_LENGTH = 0.392

    def __init__(self):
    
        self.__enc_left1_pin = Encoder.PINS['enc_left_one']
        self.__enc_left2_pin = Encoder.PINS['enc_left_two']
        self.__enc_right1_pin = Encoder.PINS['enc_right_one']
        self.__enc_right2_pin = Encoder.PINS['enc_right_two']
        
        self.__odom_trans = utils.Transforms()
        self.__odom = utils.Odometry()
        
        # GPIO pins setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enc_left1_pin, GPIO.IN)
        GPIO.setup(self.enc_left2_pin, GPIO.IN)
        GPIO.setup(self.enc_right1_pin, GPIO.IN)
        GPIO.setup(self.enc_right2_pin, GPIO.IN)

        # Encoder internal data
        self.__wheel_status = None
        
        # Odometry internal data
        self.__vel_left = 0
        self.__vel_right = 0

        self.__curr_time = rospy.get_time()
        self.__last_time = rospy.get_time()
        
        # ROS node initialization
        rospy.init_node(Encoder.NODE_NAME, disable_signals = True)
        rospy.logwarn("%s node started" % Encoder.NODE_NAME)

        # Publishers
        self.__odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 10)
        self.__twist_pub = rospy.Publisher('/twist', Twist, queue_size = 10)

    def spin(self):
        """
            Function: Runs continuous loop with specified rate in Hz

            Within the main function, the odometry is computed and 
            two message type is published via created publishers. 
            
            The two messages are:
            1. Transforms (Maintains a 3D point relationship between robot base 
                and odometry source)
            2. Odometry (Odometry is based on
        """
        try:
            r = rospy.Rate(Encoder.PUB_RATE)
            while not rospy.is_shutdown():
                self.main()
                r.sleep()

        except rospy.ROSInterruptException: pass

    def main(self):
        """
            Function: computes location of the robot in x, y and theta

            In addition to computing odometry, the function publishes
            two message types which are:

            1.Transforms
                Maintains a positional relationship between the robot
                base link and the odometry source.

            2.Odometry
                The computed position of the robot based on its wheel
                velocities and direction of travel (forward/reverse)
        """
        self._compute_location()
        
    def _compute_location(self):
        """
            Function: same as above main() function

            Inputs:
            1. left wheel velociy (in rad/s)
            2. right wheel velocity (in rad/s)
            3. "d_t" (delta time) is the time step

            Outputs:
            1. Odometry (in meters and radians [x, y, theta])
            2. Transforms (in meters and radians [x, y, theta])
           
        """
        self.__curr_time = rospy.get_time()
        d_t = self.curr_time - self.last_time
        
        _vel_th = utils.compute_vel_th(encoderL1.__vel_left,\
            encoderL1.__vel_right, d_t, Encoder.WHEEL_RADIUS,\
            Encoder.AXLE_LENGTH)
        _vel_x = utils.compute_vel_x(encoderL1.__vel_left,\
            encoderL1.__vel_right, d_t, Encoder.WHEEL_RADIUS,\
            Encoder.AXLE_LENGTH)
        _vel_y = utils.compute_vel_y(encoderL1.__vel_left,\
            encoderL1.__vel_right, d_t, Encoder.WHEEL_RADIUS,\
            Encoder.AXLE_LENGTH)

        _x += _vel_x
        _y += _vel_y
        _th += _vel_th

        self.__odom_trans.update_translation(_x, _y, _th)
        self.__odom.set_publisher()
        self.__odom.set_position(_x, _y, _th)
        self.__odom.set_velocity(_vel_x, _vel_y, _vel_th)
        
        self.__odom_trans.publish(_x, _y, _th)
        self.__odom_pub.publish(self.__odom)

        self.__last_time = self.__curr_time
        
        self._reset_velocity()
    
    def _reset_velocity(self):
    
        encoderL1.__vel_left = 0
        encoderR1.__vel_right = 0
        
    def _wheel_encoder_tick_callback(self, pin):
        """
            Function: measures the encoder ticks

            An encoder tick is the timelapse when the signal is High.

            Input: PWM signals of first encoder based on GPIO callbacks

            Output: encoder ticks (in sec)
        """
        _start_time = 0
        _counter = 0
        
        if (pin == Encoder.PINS['enc_left_one']):
            _start_time = time.time()
            _counter = 1
            if (_counter == 1):
                _enc_left_tick = (time.time() - _start_time)
                self.__vel_left = utils.compute_velocity(_enc_left_tick,\
                    self.__wheel_status)
                _counter = 0

        if (pin == Encoder.PINS['enc_right_one']):
            _start_time = time.time()
            _counter = 1
            if (_counter == 1):
                _enc_right_tick = (time.time() - _start_time)
                self.__vel_right = utils.compute_velocity(_enc_right_tick,\
                    self.__wheel_status)
                _counter = 0

    def _wheel_encoder_status_callback(self, pin):
        """
            Function: determines the direction of wheel turn

            Using both the first and second encoders in both wheels. We can
            determine the direction of wheel turn by knowing which encoders
            gives a high signal first.

            Example:
            if first encoder is high while second encoder is low, then the
            wheel is in clockwise or "forward" motion.

        """
        if (pin == Encoder.PINS['enc_left_two'] and\
            GPIO.input(Encoder.PINS['enc_left_one']) == 1):
            self.__wheel_status = "FORWARD"

        if (pin == Encoder.PINS['enc_left_two'] and\
            GPIO.input(Encoder.PINS['enc_left_one']) == 0):
            self.__wheel_status = "REVERSE"

        if (pin == Encoder.PINS['enc_right_two'] and\
            GPIO.input(Encoder.PINS['enc_right_one']) == 0):
            self.__wheel_status = "FORWARD"

        if (pin == Encoder.PINS['enc_right_two'] and\
            GPIO.input(Encoder.PINS['enc_right_one']) == 1):
            self.__wheel_status = "REVERSE"

if __name__ == '__main__':
    try:
        encoder = Encoder()
        encoderL1 = Encoder()
        encoderL2 = Encoder()
        encoderR1 = Encoder()
        encoderR2 = Encoder()

        GPIO.add_event_detect(Encoder.PINS['enc_left_one'], GPIO.RISING,\
            callback=encoderL1._wheel_encoder_tick_callback)
        GPIO.add_event_detect(Encoder.PINS['enc_left_two'], GPIO.RISING,\
            callback=encoderL2._wheel_encoder_status_callback)
        GPIO.add_event_detect(Encoder.PINS['enc_right_one'], GPIO.RISING,\
            callback=encoderR1._wheel_encoder_tick_callback)
        GPIO.add_event_detect(Encoder.PINS['enc_right_two'], GPIO.RISING,\
            callback=encoderR2._wheel_encoder_status_callback)

        encoder.spin()

    except rospy.ROSInterruptException: pass