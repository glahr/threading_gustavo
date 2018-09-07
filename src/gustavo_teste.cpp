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
//     ROS_INFO("soh pra nao passar em branco");
}

void active_cb()
{
//   ROS_INFO("Goal just went active");
}

void feedback_cb(const threading_gustavo::AdmittanceControllerFeedbackConstPtr& feedback)
{
    
        int tamanho = feedback->linear_error_norm.size();
        //std::cout << tamanho << std::endl;
        if (tamanho == 1)
        {
//             *meuValor = feedback->actual_pose;
//             std::cout << *meuValor << std::endl;
//             std::cout << "linear_error_norm: " << feedback->linear_error_norm[0] << std::endl;
//             std::cout << "roll_error: " << feedback->roll_error[0] << std::endl;
//             std::cout << "pitch_error: " << feedback->pitch_error[0] << std::endl;
//             std::cout << "yaw_error: " << feedback->yaw_error[0] << "\n" << std::endl;
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
  
  actionlib::SimpleActionClient<threading_gustavo::AdmittanceControllerAction> ac_admitance("/admittance_controller/admittance",
 true);
  actionlib::SimpleActionClient<threading_gustavo::ApproachControllerAction> ac_approach("/approach_controller/approach", true);
  

// // // // // // // // // // // // // // // // // // INITIAL POSITION
  std::cout << "\nINITIAL POSITION \n" << std::endl;
  
  ac_admitance.waitForServer(); //will wait for infinite time  
  threading_gustavo::AdmittanceControllerGoal alignment_pose_goal;
  
  alignment_pose_goal.use_left = true;
  alignment_pose_goal.desired_left_pose.header.frame_id = "yumi_link_7_l";
  alignment_pose_goal.pure_ft_control = false;

  alignment_pose_goal.desired_left_pose.pose.position.x = 0.3;
  alignment_pose_goal.desired_left_pose.pose.position.y = 0.15;
  alignment_pose_goal.desired_left_pose.pose.position.z = 0.3;
  alignment_pose_goal.desired_left_pose.pose.orientation.x = 1;
//   alignment_pose_goal.desired_left_pose.pose.orientation.y = 0;
  
  ac_admitance.sendGoal(alignment_pose_goal, &done_cb, &active_cb, &feedback_cb);
  ac_admitance.waitForResult(ros::Duration(0.0));  
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();
  
// // // // // // // // // // // // // // // // // // // ALIGNMENT
  std::cout << "ALIGNMENT \n" << std::endl;
  
  alignment_pose_goal.pure_ft_control = true;
  ac_admitance.sendGoal(alignment_pose_goal, &done_cb, &active_cb, &feedback_cb);  
  ac_admitance.waitForResult(ros::Duration(0.0));
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();
  
  
// // // // // // // // // // // // // // // // // // // THREADING  
  std::cout << "THREADING \n" << std::endl;
  
  ac_approach.waitForServer(); //will wait for infinite time
  threading_gustavo::ApproachControllerGoal threading;
  
//   threading.desired_twist.twist.linear.z = 0.01;
  threading.desired_twist.twist.angular.z = 0.1;
  threading.desired_twist.header.frame_id = "yumi_link_7_l";
  threading.max_contact_force = 1.0;
  threading.threading = true;
  
  
  ac_approach.sendGoal(threading);
  ac_approach.waitForResult(ros::Duration(0.0));
  ac_approach.cancelGoal();
  ac_approach.stopTrackingGoal();
  
  
// // // // // // // // // // // // // // // // // // RECOVERING
  std::cout << "RECOVERING \n" << std::endl;
  
  
  threading.desired_twist.twist.angular.z = -0.1;
  threading.threading = false;
  ac_admitance.sendGoal(alignment_pose_goal, &done_cb, &active_cb, &feedback_cb);  
  ac_admitance.waitForResult(ros::Duration(0.0));
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();

  
  return 0;
}
