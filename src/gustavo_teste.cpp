#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../../../devel/include/threading_gustavo/ApproachControllerAction.h"
#include "../../../devel/include/threading_gustavo/AdmittanceControllerAction.h"
//#include "../../generic_control_toolbox/include/generic_control_toolbox/kdl_manager.hpp"
#include <iostream>

//#include <geometry_msgs/Twist.h>
//#include <threading_gustavo/AdmittanceControllerAction.h>
//#include <threading_gustavo/ApproachControllerAction.h>
//#include <threading_gustavo/ApproachControllerGoal.h>

//using namespace generic_control_toolbox;

// void done_cb(const threading_gustavo::AdmittanceControllerResult& state,
//             const threading_gustavo::AdmittanceControllerResultConstPtr& result)
// {
//     ROS_INFO("soh pra nao passar em branco");
// }

double *meuValor = new double;

void done_cb(const actionlib::SimpleClientGoalState& state,
            const threading_gustavo::AdmittanceControllerResultConstPtr& result)
{
    ROS_INFO("soh pra nao passar em branco");
}


void active_cb()
{
  ROS_INFO("Goal just went active");
}


void feedback_cb(const threading_gustavo::AdmittanceControllerFeedbackConstPtr& feedback)
{
    
    try {
        int tamanho = feedback->linear_error_norm.size();
        //std::cout << tamanho << std::endl;
        if (tamanho == 1)
        {
            *meuValor = feedback->linear_error_norm[0];
            std::cout << (*feedback).linear_error_norm[0] << std::endl;
            //return (*feedback).linear_error_norm[0];
        }
        
        if (tamanho == 2)
            std::cout << "TAMANHO 2" << std::endl;
            
    //ROS_INFO("Got Feedback of length %lu", feedback->linear_error_norm.size());
        //ROS_INFO("Got Feedback of length %f", meuvalor);
    }
    catch(std::runtime_error err) {
        std::cout << "What was that?";
        //break;
    }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "threading_gustavo");

  //MOVING TO STARTING POSITION
  
  
  //ADMITTANCE
  actionlib::SimpleActionClient<threading_gustavo::AdmittanceControllerAction> ac("/admittance_controller/admittance", true);
  ac.waitForServer(); //will wait for infinite time
  
  threading_gustavo::AdmittanceControllerGoal initial_pose;
  //threading_gustavo::AdmittanceControllerActionFeedback feedback;
  
  //feedback.header.frame_id = "yumi_link_7_l";
  
  initial_pose.use_left = true;
  initial_pose.desired_left_pose.header.frame_id = "yumi_link_7_l";
  initial_pose.desired_left_pose.pose.position.x = 0.4;
  initial_pose.desired_left_pose.pose.position.y = 0.0;
  initial_pose.desired_left_pose.pose.position.z = 0.25;
  initial_pose.desired_left_pose.pose.orientation.x = 1;
  
  
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
  
  
  double delta = 0;
  
  while(true)
  {
      if(delta - *meuValor != 0)
      {
        std::cout << *meuValor << std::endl;
        delta = *meuValor;
      }
  }    
   
  
  std::cout << "esperando \n" << std::endl;
  
//   ROS_INFO("waiting");
  ac.waitForResult(ros::Duration(10.0));
//   ROS_INFO("pass");
  
  ac.cancelGoal();
  std::cout << "acabou a espera \n" << std::endl;
  ac.stopTrackingGoal();
  
  std::cout << "sending new goal" << std::endl;
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
  ac.waitForResult(ros::Duration(10.0));
  ac.cancelGoal();
  ac.stopTrackingGoal();
  
  
  
// // // // // // // // // // // // // // // // // // // // // // // //   //APPROACH
  actionlib::SimpleActionClient<threading_gustavo::ApproachControllerAction> ac_approach("/approach_controller/approach", true);
  
//   ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_approach.waitForServer(); //will wait for infinite time
//   ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  threading_gustavo::ApproachControllerGoal insertion;
  
  insertion.desired_twist.twist.linear.z = 0.01;
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
