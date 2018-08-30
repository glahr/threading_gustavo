#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <threading_gustavo/ApproachControllerAction.h>
#include <threading_gustavo/ApproachControllerGoal.h>
#include "../../../devel/include/threading_gustavo/ApproachControllerAction.h"
#include "../../generic_control_toolbox/include/generic_control_toolbox/kdl_manager.hpp"
#include <iostream>
#include <geometry_msgs/Twist.h>

#include <threading_gustavo/AdmittanceControllerAction.h>
#include <threading_gustavo/AdmittanceControllerGoal.h>
#include "../../../devel/include/threading_gustavo/AdmittanceControllerAction.h"

//using namespace generic_control_toolbox;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "threading_gustavo");

  //MOVING TO STARTING POSITION
  
  
  //ADMITTANCE
  actionlib::SimpleActionClient<threading_gustavo::AdmittanceControllerAction> ac("/admittance_controller/admittance", true);
  ac.waitForServer(); //will wait for infinite time
  threading_gustavo::AdmittanceControllerGoal initial_pose;
  initial_pose.use_left = true;
  initial_pose.desired_left_pose.header.frame_id = "yumi_link_7_l";
  initial_pose.desired_left_pose.pose.position.x = 0.4;
  initial_pose.desired_left_pose.pose.position.y = 0.0;
  initial_pose.desired_left_pose.pose.position.z = 0.25;
  initial_pose.desired_left_pose.pose.orientation.x = 1;
  
  std::cout << initial_pose << std::endl;
  
  ac.sendGoal(initial_pose);
  
  
//   ROS_INFO("waiting");
  ac.waitForResult(ros::Duration(30.0));
//   ROS_INFO("pass");
  
  ac.cancelGoal();
  
  //APPROACH
  actionlib::SimpleActionClient<threading_gustavo::ApproachControllerAction> ac_approach("/approach_controller/approach", true);
  
//   ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_approach.waitForServer(); //will wait for infinite time
//   ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  threading_gustavo::ApproachControllerGoal insertion;
  
  insertion.desired_twist.twist.linear.z = 0.01;
  insertion.desired_twist.header.frame_id = "yumi_link_7_l";
  insertion.max_contact_force = 30;
  
  
  ac_approach.sendGoal(insertion);	


  std::cout << "[ INFO] [" << ros::Time::now() << "]: Antes" << std::endl;
  //ROS_INFO("Antes");
  //ROS_INFO(begin);
  //wait for the action to return
  bool finished_before_timeout = ac_approach.waitForResult(ros::Duration(0.0));
  //ROS_INFO("Depois");
  std::cout << "[ INFO] [" << ros::Time::now() << "]: Depois" << std::endl;
  //ROS_INFO();
  
//  while(ros::ok())
	//{
	  //ros::spin();
//	}

//   if (finished_before_timeout)
//   {
//     actionlib::SimpleClientGoalState state = ac.getState();
//     ROS_INFO("Action finished: %s",state.toString().c_str());
//   }
//   else
//     ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
