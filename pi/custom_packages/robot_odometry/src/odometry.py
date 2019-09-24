#!/usr/bin/env python
import rospy
import math
import RPi.GPIO as GPIO
import time
import tf

from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

pins = {'enc_left_one':23,
        'enc_left_two':24,
        'enc_right_one':5,
        'enc_right_two':6
        }

class Encoder:
    def __init__(self, enc_left1_pin = pins['enc_left_one'], enc_left2_pin = pins['enc_left_two'], enc_right1_pin = pins['enc_right_one'], enc_right2_pin = pins['enc_right_two']):
        # GPIO pins setup
        self.enc_left1_pin = enc_left1_pin
        self.enc_left2_pin = enc_left2_pin
        self.enc_right1_pin = enc_right1_pin
        self.enc_right2_pin = enc_right2_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enc_left1_pin, GPIO.IN)
        GPIO.setup(self.enc_left2_pin, GPIO.IN)
        GPIO.setup(self.enc_right1_pin, GPIO.IN)
        GPIO.setup(self.enc_right2_pin, GPIO.IN)

        # ROS node initialization
        rospy.init_node('encoder_tick_publisher', disable_signals = True)
        self.node_name = rospy.get_name()
        rospy.logwarn("%s node started" % self.node_name)

        # Encoder internal data
        self.counter = 0
        self.start_time = 0
        self.enc_left_tick = -1
        self.enc_left_counter = 0
        self.enc_right_tick = -1
        self.enc_right_counter = 0
        self.lwheel_status = None
        self.rwheel_status = None

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 10)
        self.twist_pub = rospy.Publisher('/twist/', Twist, queue_size = 10)

        # Subscribers
        # None

        # Robot wheel specificiations (in meters)
        self.wheel_radius = 0.075
        self.axle_length = 0.392

        # Odometry internal data
        self.vel_left = 0
        self.vel_right = 0
        
        self.x = 0
        self.y = 0
        self.th = 0
        
        self.vel_x = 0
        self.vel_y = 0
        self.vel_th = 0
        
        self.br = tf.TransformBroadcaster()
        self.odom_trans = geometry_msgs.msg.TransformStamped()
        self.odom = Odometry()

        self.curr_time = rospy.get_time()
        self.last_time = rospy.get_time()

    def spin(self):
        """
            Function: Runs continuous loop with specified rate in Hz

            Within the main function, the odometry is computed and 
            two message type is published via created publishers. 
            
            The two messages are:
            1. Transforms (Maintains a 3D point relationship between robot base and odometry source)
            2. Odometry (Odometry is based on
        """
        try:
            r = rospy.Rate(5.5)

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
        self.compute_location()
        
    def compute_location(self):
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
        self.curr_time = rospy.get_time()
        d_t = self.curr_time - self.last_time
        self.vel_th = ((self.wheel_radius / self.axle_length) * (encoderR1.vel_right - encoderL1.vel_left)) * d_t
        self.vel_x = ((self.wheel_radius / 2) * (encoderL1.vel_left + encoderR1.vel_right) * math.cos(self.th)) * d_t
        self.vel_y = ((self.wheel_radius / 2) * (encoderL1.vel_left + encoderR1.vel_right) * math.sin(self.th)) * d_t

        self.x += self.vel_x
        self.y += self.vel_y
        self.th += self.vel_th

        # Update Transform
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom_quat = Quaternion(*odom_quat)
        self.odom_trans.header.stamp = rospy.Time.now()
        self.odom_trans.transform.translation.x = self.x
        self.odom_trans.transform.translation.y = self.y
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = odom_quat

        # Publish odometry message over ROS
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        # Set position
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = odom_quat

        # Set velocity
        self.odom.twist.twist.linear.x = self.vel_x
        self.odom.twist.twist.linear.y = self.vel_y
        self.odom.twist.twist.angular.z = self.vel_th

        # Publish odometry and transforms
        self.br.sendTransform((self.x, self.y, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, self.th), rospy.Time.now(), "base_link", "odom")
        self.odom_pub.publish(self.odom)

        self.last_time = self.curr_time
        
        encoderL1.vel_left = 0
        encoderR1.vel_right = 0

    def compute_velL(self):
        """
            Function: compute velocity of left wheel

            The smallest resolution of a single tick is 1/36th the
            wheel circumference in radians. The resolution is
            determined by the number of encoder used.

            We are only using the first encoder for each wheel 
            to determine the ticks.

            The second encoder is used to determine the direction
            of wheel turn (clockwise or anti clockwise)

            Inputs:
            1. left encoder ticks (pulse width)
            2. left encoder wheel status (Forward or Reverse)

            Outputs:
            1. left wheel velocity (in rad/s)
        """
        dist_pos = (2 * math.pi) / 36
        dist_neg = -((2 * math.pi) / 36)

        if (encoderL2.lwheel_status is "FORWARD"):
            self.vel_left = dist_pos / encoderL1.enc_left_tick

        if (encoderL2.lwheel_status is "REVERSE"):
            self.vel_left = dist_neg / encoderL1.enc_left_tick

    def compute_velR(self):
        """
            Function: compute velocity of right wheel
            
            The smallest resolution of a single tick is 1/36th the
            wheel circumference in radians. The resolution is
            determined by the number of encoder used.

            We are only using the first encoder for each wheel 
            to determine the ticks.

            The second encoder is used to determine the direction
            of wheel turn (clockwise or anti clockwise)

            Inputs:
            1. right encoder ticks (pulse width)
            2. right encoder wheel status (Forward or Reverse)

            Outputs:
            1. right wheel velocity (in rad/s)
        """
        dist_pos = (2 * math.pi) / 36
        dist_neg = -((2 * math.pi) / 36)

        if (encoderR2.rwheel_status is "FORWARD"):
            self.vel_right = dist_pos / encoderR1.enc_right_tick
        
        if (encoderR2.rwheel_status is "REVERSE"):
            self.vel_right = dist_neg / encoderR1.enc_right_tick

    def wheel_encoder_callback(self, pin):
        """
            Function: measures the encoder ticks

            An encoder tick is the timelapse when the signal is High.

            Input: PWM signals of first encoder based on GPIO callbacks

            Output: encoder ticks (in sec)
        """
        start_time = time.time()

        if (pin == pins['enc_left_one']):
            self.enc_left_one_start_time = time.time() - start_time
            if (self.counter == 0):
                self.start_time = time.time()
                self.counter = 1
            else:
                self.enc_left_tick = (time.time() - self.start_time)
                self.compute_velL()
                self.counter = 0
                self.enc_left_counter += 1

        if (pin == pins['enc_right_one']):
            if (self.counter == 0):
                self.start_time = time.time()
                self.counter = 1
            else:
                self.enc_right_tick = (time.time() - self.start_time)
                self.compute_velR()
                self.counter = 0
                self.enc_right_counter += 1

    def wheel_encoder_callback_two(self, pin):
        """
            Function: determines the direction of wheel turn

            Using both the first and second encoders in both wheels. We can
            determine the direction of wheel turn by knowing which encoders
            gives a high signal first.

            Example:
            if first encoder is high while second encoder is low, then the
            wheel is in clockwise or "forward" motion.

        """
        if (pin == pins['enc_left_two'] and GPIO.input(pins['enc_left_one']) == 1):
            self.lwheel_status = "FORWARD"

        if (pin == pins['enc_left_two'] and GPIO.input(pins['enc_left_one']) == 0):
            self.lwheel_status = "REVERSE"

        if (pin == pins['enc_right_two'] and GPIO.input(pins['enc_right_one']) == 0):
            self.rwheel_status = "FORWARD"

        if (pin == pins['enc_right_two'] and GPIO.input(pins['enc_right_one']) == 1):
            self.rwheel_status = "REVERSE"

if __name__ == '__main__':
    try:
        encoder = Encoder()
        encoderL1 = Encoder()
        encoderL2 = Encoder()
        encoderR1 = Encoder()
        encoderR2 = Encoder()

        GPIO.add_event_detect(pins['enc_left_one'], GPIO.RISING, callback=encoderL1.wheel_encoder_callback)
        GPIO.add_event_detect(pins['enc_left_two'], GPIO.RISING, callback=encoderL2.wheel_encoder_callback_two)

        GPIO.add_event_detect(pins['enc_right_one'], GPIO.RISING, callback=encoderR1.wheel_encoder_callback)
        GPIO.add_event_detect(pins['enc_right_two'], GPIO.RISING, callback=encoderR2.wheel_encoder_callback_two)

        encoder.spin()

    except rospy.ROSInterruptException: pass
