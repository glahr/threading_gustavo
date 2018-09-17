#! /usr/bin/env python

from __future__ import print_function
import rospy

import actionlib
from geometry_msgs.msg import TwistStamped, PoseStamped
from threading_gustavo.msg import ApproachControllerAction, ApproachControllerGoal, AdmittanceControllerGoal, AdmittanceControllerAction

def threading_gustavo():
    isApproach = False
    
    
    if isApproach == True:
        client = actionlib.SimpleActionClient('/approach_controller/approach', ApproachControllerAction)
        client.wait_for_server()
        approach_goal = ApproachControllerGoal()
        approach_goal.desired_twist.header.frame_id = "yumi_link_7_l"
        approach_goal.desired_twist.twist.linear.x = 0.1
        approach_goal.max_contact_force = 1.5
        client.send_goal(approach_goal)
        client.wait_for_result()
    else:
        client = actionlib.SimpleActionClient('/admittance_controller/admittance', AdmittanceControllerAction)
        client.wait_for_server()
        initial_pose = AdmittanceControllerGoal()
        initial_pose.use_right = True
        initial_pose.desired_right_pose.header.frame_id = "yumi_link_7_l"
        initial_pose.desired_right_pose.pose.position.x = 0.1
        initial_pose.desired_right_pose.pose.position.z = 0.3
        client.send_goal(initial_pose)
        client.wait_for_result()



    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('threading_gustavo')
        result = threading_gustavo()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

