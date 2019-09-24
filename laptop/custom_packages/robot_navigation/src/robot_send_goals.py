#! /usr/bin/env python
import rospy
import actionlib
import move_base_msgs.msg
import actionlib_tutorials.msg

def move(x, y, th):
    client = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))
    
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.orientation.z = th

    client.send_goal(goal)
    success = client.wait_for_result(rospy.Duration(60))
    rospy.loginfo(success)

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_to_goal', disable_signals = True)
        node_name = rospy.get_name()
        rospy.logwarn("%s node started" % node_name)
        move(0.0, 0.0, 1.0)

    except rospy.ROSInterruptException: pass
    
