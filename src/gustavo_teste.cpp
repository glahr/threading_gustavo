#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../../../devel/include/threading_gustavo/ApproachControllerAction.h"
#include "../../../devel/include/threading_gustavo/AdmittanceControllerAction.h"
#include "../../generic_control_toolbox/include/generic_control_toolbox/kdl_manager.hpp"
#include "../../yumi_experiments/include/yumi_experiments/admittance_controller.hpp"
#include <iostream>
//#include <robot_model.h>
#include <tf/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


using namespace generic_control_toolbox;
using namespace yumi_experiments;

double *meuValor = new double;
sensor_msgs::JointState states;
// tf::TransformListener myKinematics;

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
    
//         ros::spinOnce(); //esse cara ta aqui porque assim eu consigo ler as juntas no callback
        
//         std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
//         
//         sensor_msgs::JointState ret = states;
//         sensor_msgs::JointState current_state = states;
//         std::vector<KDL::Frame> pose(2);
//         std::vector<KDL::Twist> vel(2), desired_vel(2);
//         std::vector<Eigen::Affine3d> pose_eig(2);
//         std::vector<Vector6d > vel_eig(2);
//         std::vector<Vector6d > wrench(2);
//         std::vector<Vector6d > acc(2);
//         std::vector<Vector6d > error(2);
//         char eef_name[][20] = {"yumi_link_7_l", "yumi_link_7_r"};
//         generic_control_toolbox::WrenchManager wrench_manager_;
// 
//         kdl_manager_->getGrippingPoint(eef_name[LEFT_ARM], current_state, pose[LEFT_ARM]);
// //         kdl_manager_->getGrippingPoint(eef_name_[RIGHT_ARM], current_state, pose[RIGHT_ARM]);
//         tf::transformKDLToEigen(pose[LEFT_ARM], pose_eig[LEFT_ARM]);
// //         tf::transformKDLToEigen(pose[RIGHT_ARM], pose_eig[RIGHT_ARM]);
//         kdl_manager_->getGrippingTwist(eef_name[LEFT_ARM], current_state, vel[LEFT_ARM]);
// //         kdl_manager_->getGrippingTwist(eef_name_[RIGHT_ARM], current_state, vel[RIGHT_ARM]);
//         tf::twistKDLToEigen(vel[LEFT_ARM], vel_eig[LEFT_ARM]);
// //         tf::twistKDLToEigen(vel[RIGHT_ARM], vel_eig[RIGHT_ARM]);
//         wrench_manager_.wrenchAtGrippingPoint(eef_name[RIGHT_ARM], wrench[RIGHT_ARM]);
// //         wrench_manager_.wrenchAtGrippingPoint(eef_name_[LEFT_ARM], wrench[LEFT_ARM]);
//         
//         std::cout << pose[LEFT_ARM].p[0] << std::endl;
        
        
        int tamanho = feedback->linear_error_norm.size();
        //std::cout << tamanho << std::endl;
        if (tamanho == 1)
        {
//             *meuValor = feedback->linear_error_norm[0];
//             std::cout << states.position[0] << std::endl;
//             std::cout << (*feedback).linear_error_norm[0] << std::endl;
            //return (*feedback).linear_error_norm[0];
            
            
        }
        
        if (tamanho == 2)
            std::cout << "TAMANHO 2" << std::endl;
            
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
  
   
  
//   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//   ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
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
  
  
//   const threading_gustavo::AdmittanceControllerFeedbackConstPtr& feedback = nullptr;
//   int a;
  
//   ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb(feedback, a));
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
//   std::cout << a << std::endl;
  
  
// // // // // //   MY TRY ON FK
  
//   std::string arm_name = "yumi_link_7_l";
//   
//   generic_control_toolbox::ArmInfo info;
//   std::string base_frame;
//   tn_.getParam("kinematic_chain_base_link", base_frame);
//   std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager;
//   kdl_manager = std::make_shared<generic_control_toolbox::KDLManager>(base_frame);
//   generic_control_toolbox::setKDLManager(info, kdl_manager);
//   KDL::Frame out;
//   kdl_manager getEefPose("yumi_link_7_l", states, out);
//   
//   std::cout << info.
//   
  
// // // // // // // // // //   
  
  
  // rotina para pegar o valor do feedback. Não consegui pensar um jeito melhor ainda
// //   double delta = 0;
// //   while(true)
// //   {
// //       if(delta - *meuValor != 0)
// //       {
// //         std::cout << *meuValor << std::endl;
// //         delta = *meuValor;
// //       }
// //   }
   
  
//  std::cout << "esperando \n" << std::endl;
  
  std::cout << "TROQUEI 1 \n" << std::endl;
  ac.waitForResult(ros::Duration(10.0));
  
  
//   ac.cancelGoal();
// //   std::cout << "acabou a espera \n" << std::endl;
//   ac.stopTrackingGoal();
  
//   std::cout << "sending new goal" << std::endl;

  initial_pose.desired_left_pose.pose.position.x = 0.4;
  initial_pose.desired_left_pose.pose.position.y = 0.0;
  initial_pose.desired_left_pose.pose.position.z = 0.3;
  initial_pose.desired_left_pose.pose.orientation.x = 1;
  
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);  
  std::cout << "TROQUEI 2 \n" << std::endl;
  ac.waitForResult(ros::Duration(10.0));
//   ac.cancelGoal();
//   ac.stopTrackingGoal();
  
  initial_pose.desired_left_pose.pose.position.x = 0.3;
  initial_pose.desired_left_pose.pose.position.y = 0.1;
  initial_pose.desired_left_pose.pose.position.z = 0.3;
  initial_pose.desired_left_pose.pose.orientation.x = 1;
  
  ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);  
  std::cout << "TROQUEI 3 \n" << std::endl;
  ac.waitForResult(ros::Duration(10.0));
  
  
  std::cout << "ENTRANDO NO SENO \n" << std::endl;
  ros::Duration(1.0).sleep();
  
  ros::Rate r(100);
  double begin = ros::Time::now().toSec();
  
  while(ros::ok())
  {
      
    
    double t = ros::Time::now().toSec() - begin;
    initial_pose.desired_left_pose.pose.position.x = 0.3 + 0.1*sin(0.2*t);
    initial_pose.desired_left_pose.pose.position.y = 0.1;
    initial_pose.desired_left_pose.pose.position.z = 0.3;
    initial_pose.desired_left_pose.pose.orientation.x = 1;
    
    ac.sendGoal(initial_pose, &done_cb, &active_cb, &feedback_cb);
    r.sleep();
  }
  
  
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
