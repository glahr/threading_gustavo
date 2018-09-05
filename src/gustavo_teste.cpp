#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../../../devel/include/threading_gustavo/ApproachControllerAction.h"
#include "../../../devel/include/threading_gustavo/AdmittanceControllerAction.h"
#include "../include/admittance_controller.hpp"
#include <iostream>
//#include <robot_model.h>
#include <tf/transform_listener.h>

using namespace generic_control_toolbox;
using namespace yumi_experiments;

geometry_msgs::PoseStamped *meuValor = new geometry_msgs::PoseStamped;
sensor_msgs::JointState states;

void done_cb(const actionlib::SimpleClientGoalState& state,
            const threading_gustavo::AdmittanceControllerResultConstPtr& result)
{
    ROS_INFO("soh pra nao passar em branco");
}

void active_cb()
{
  ROS_INFO("Goal just went active");
}


// void feedback_cb(const threading_gustavo::AdmittanceControllerFeedbackConstPtr& feedback, int a)
void feedback_cb(const threading_gustavo::AdmittanceControllerFeedbackConstPtr& feedback)
{
    
        int tamanho = feedback->linear_error_norm.size();
        //std::cout << tamanho << std::endl;
        if (tamanho == 1)
        {
//             *meuValor = feedback->actual_pose;
//             std::cout << *meuValor << std::endl;
//             std::cout << feedback->linear_error_norm[0] << std::endl;            
        }
            
}

void statesCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    states =  *msg;
//     std::cout << states << std::endl;
//     std::cout << "ta aqui" << std::endl;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "threading_gustavo");
  
//  ros::NodeHandle tn_; //threading node
//  ros::Subscriber states_sub = tn_.subscribe("/yumi/joint_states",1,statesCb);
  
  //MOVING TO STARTING POSITION
  
  actionlib::SimpleActionClient<threading_gustavo::AdmittanceControllerAction> ac("/admittance_controller/admittance", true);
  ac.waitForServer(); //will wait for infinite time
  
  threading_gustavo::AdmittanceControllerGoal initial_pose;
  threading_gustavo::AdmittanceControllerGoal aux_pose;
  //threading_gustavo::AdmittanceControllerActionFeedback feedback;
  
  //feedback.header.frame_id = "yumi_link_7_l";
  
  initial_pose.use_left = true;
  initial_pose.desired_left_pose.header.frame_id = "yumi_link_7_l";
  initial_pose.pure_ft_control = false;

  initial_pose.desired_left_pose.pose.position.x = 0.4;
  initial_pose.desired_left_pose.pose.position.y = 0.08;
  initial_pose.desired_left_pose.pose.position.z = 0.25;
  initial_pose.desired_left_pose.pose.orientation.x = 1;
  
  std::cout << "APPROACH \n" << std::endl;
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
  ac.waitForResult(ros::Duration(20.0));  
  ac.cancelGoal();
  ac.stopTrackingGoal();
  
// //   std::cout << "sending new goal" << std::endl;
// 
//   initial_pose.desired_left_pose.pose.position.x = 0.3;
//   initial_pose.desired_left_pose.pose.position.y = 0.1;
//   initial_pose.desired_left_pose.pose.position.z = 0.3;
//   initial_pose.desired_left_pose.pose.orientation.x = 0;
//   
//   ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);  
//   std::cout << "TROQUEI 2 \n" << std::endl;
//   ac.waitForResult(ros::Duration(10.0));
//   ac.cancelGoal();
//   ac.stopTrackingGoal();
  
//   aux_pose.desired_left_pose.pose.position.x = 0.4;
//   aux_pose.desired_left_pose.pose.position.y = 0.08;
//   aux_pose.desired_left_pose.pose.position.z = 0.25;
//   aux_pose.pure_ft_control = true;
  
//   aux_pose.desired_left_pose.pose.orientation.x = -0.146;
//   aux_pose.desired_left_pose.pose.orientation.y = 0.354;
//   aux_pose.desired_left_pose.pose.orientation.z = 0.354;
//   aux_pose.desired_left_pose.pose.orientation.w = 0.854;
//   aux_pose.desired_left_pose.pose.orientation.x = 1;
  
//   ac.sendGoal(aux_pose, &done_cb, &active_cb, &feedback_cb);  
//   std::cout << "PONTO 2 \n" << std::endl;
//   ac.waitForResult(ros::Duration(0.0));
  
  initial_pose.pure_ft_control = true;
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);  
  std::cout << "ALIGNMENT \n" << std::endl;
  ac.waitForResult(ros::Duration(0.0));
  ac.cancelGoal();
  ac.stopTrackingGoal();
  
  
  std::cout << "THREADING \n" << std::endl;
  initial_pose.pure_ft_control = false;
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
  
//   std::cout << "LOOP INFINITO \n" << std::endl;
//   while(true)
//   {
//       ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
//       ac.waitForResult(ros::Duration(0.0));
//       ac.sendGoal(aux_pose, &done_cb, &active_cb, &feedback_cb);  
//       ac.waitForResult(ros::Duration(0.0));
//   }
  
  
//   std::cout << "ENTRANDO NO SENO \n" << std::endl;
//   ros::Duration(1.0).sleep();
//   
//   ros::Rate r(100);
//   double begin = ros::Time::now().toSec();
//   
//   while(ros::ok())
//   { 
//     double t = ros::Time::now().toSec() - begin;
//     initial_pose.desired_left_pose.pose.position.x = 0.3 + 0.1*sin(0.2*t);
//     initial_pose.desired_left_pose.pose.position.y = 0.1;
//     initial_pose.desired_left_pose.pose.position.z = 0.3;
//     initial_pose.desired_left_pose.pose.orientation.x = 1;
//     
//     ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
//     r.sleep();
//   }
  
  
// // // // // // // // // // // // // // // // // // // // // // // //   //APPROACH
  actionlib::SimpleActionClient<threading_gustavo::ApproachControllerAction> ac_approach("/approach_controller/approach", true);
  
//   ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_approach.waitForServer(); //will wait for infinite time
//   ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  threading_gustavo::ApproachControllerGoal insertion;
  
  insertion.desired_twist.twist.linear.z = 0.01;
  insertion.desired_twist.twist.angular.z = 0.1;
  insertion.desired_twist.header.frame_id = "yumi_link_7_l";
  insertion.max_contact_force = 1.0;
  
  
  ac_approach.sendGoal(insertion);	


  std::cout << "[ INFO] [" << ros::Time::now() << "]: Antes" << std::endl;
  //ROS_INFO("Antes");
  //ROS_INFO(begin);
  //wait for the action to return
  ac_approach.waitForResult(ros::Duration(0.0));
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
