import rospy
import math
import tf
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

DIST_POS = (2 * math.pi) / 36
DIST_NEG = -(2 * math.pi) / 36

def compute_velocity(tick, status):
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
    _vel = 0
    
    if (status is "FORWARD"):
        _vel = DIST_POS / tick
    if (status is "REVERSE"):
        _vel = DIST_NEG / tick
        
    return vel

class Odometry:

    HEADER_FRAME_ID = "odom"
    CHILD_FRAME_ID = "base_link"

    def __init__(self):
    
        self.__odom = Odometry()
        
    def compute_vel_x(self, vleft, vright, dt, wheel_rad, axle_len):
        
        vel_x = ((wheel_rad / 2) * (vleft + vright) * math.cos(self.__th)) * dt
        
        return vel_x
        
    def compute_vel_y(self, vleft, vright, dt, wheel_rad, axle_len):
        
        vel_y = ((wheel_rad / 2) * (vleft + vright) * math.sin(self.__th)) * dt
        
        return vel_y
        
    def compute_vel_th(self, vleft, vright, dt, wheel_rad, axle_len):
    
        vel_th = ((wheel_rad / axle_len) * (vright - vleft)) * dt

        return vel_th
        
    def set_position(self, x, y, th):
    
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_quat = Quaternion(*odom_quat)
        
        self.__odom.pose.pose.position.x = x
        self.__odom.pose.pose.position.y = y
        self.__odom.pose.pose.position.z = 0.0
        self.__odom.pose.pose.orientation = odom_quat
        
    def set_velocity(self, vel_x, vel_y, vel_th):
    
        self.__odom.twist.twist.linear.x = vel_x
        self.__odom.twist.twist.linear.y = vel_y
        self.__odom.twist.twist.angular.z = vel_th
        
    def set_publisher(self):

        self.__odom.header.stamp = rospy.Time.now()
        self.__odom.header.frame_id = Odometry.HEADER_FRAME_ID
        self.__odom.child_frame_id = Odometry.CHILD_FRAME_ID
        
    def publish_msg(self

class Transforms(Odometry):

    def __init__(self):
        
        self.__br = tf.TransformBroadcaster()
        self.__odom_trans = geometry_msgs.msg.TransformStamped()
        self.__odom_quat
        
    def update_translation(self, x, y, th):
    
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_quat = Quaternion(*odom_quat)
        
        self.__odom_trans.header.stamp = rospy.Time.now()
        self.__odom_trans.transform.translation.x = x
        self.__odom_trans.transform.translation.y = y
        self.__odom_trans.transform.translation.z = 0.0
        self.__odom_trans.transform.rotation = odom_quat
        
    def publish(self, x, y, th):
    
        self.__br.sendTransform((x, y, 0.0),\
        tf.transformations.quaternion_from_euler(0.0, 0.0, th),\
        rospy.Time.now(), Odometry.CHILD_FRAME_ID, Odometry.HEADER_FRAME_ID)
        