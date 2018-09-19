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
float *posZ = new float;
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
//             std::cout << "linear_error_norm: \t" << feedback->linear_error_norm[0] << std::endl;
//             std::cout << "roll_error: \t" << feedback->roll_error[0] << std::endl;
//             std::cout << "pitch_error: \t" << feedback->pitch_error[0] << std::endl;
//             std::cout << "yaw_error: \t" << feedback->yaw_error[0] << std::endl;
//             std::cout << "fz: \t" << feedback->fz[0] << std::endl;
//             std::cout << "posZ: \t" << feedback->posZ[0] << "\n" << std::endl;
            *posZ = feedback->posZ[0];
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


  threading_gustavo::AdmittanceControllerGoal initial_position;
  threading_gustavo::ApproachControllerGoal going_up;
  threading_gustavo::ApproachControllerGoal back_spinning;
  threading_gustavo::ApproachControllerGoal threading_task;
//   threading_gustavo::ApproachControllerGoal recovering_task;

//   recovering_task.desired_twist.twist.angular.z = -0.2;
//   recovering_task.desired_twist.header.frame_id = "yumi_link_7_l";
//   recovering_task.threading = false;
//   recovering_task.back_spin = false;
//   recovering_task.recovering = true;
//   recovering_task.going_up = false;
//   recovering_task.max_contact_force = 1.0;

  threading_task.desired_twist.twist.angular.z = 0.2;
  threading_task.desired_twist.header.frame_id = "yumi_link_7_l";
  threading_task.max_contact_force = 1.0;
  threading_task.threading = true;
  threading_task.back_spin = false;
  threading_task.recovering = false;
  threading_task.going_up = false;

  back_spinning.desired_twist.twist.angular.z = -0.2;
  back_spinning.desired_twist.header.frame_id = "yumi_link_7_l";
  back_spinning.threading = false;
  back_spinning.recovering = false;
  back_spinning.going_up = false;
  back_spinning.back_spin = true;
  back_spinning.max_contact_force = 1.0;

  going_up.desired_twist.twist.linear.z = -0.02;
  going_up.desired_twist.header.frame_id = "yumi_link_7_l";
  going_up.threading = false;
  going_up.recovering = false;
  going_up.going_up = true;
  going_up.back_spin = false;
  going_up.max_contact_force = 1.0;


  // // // // // // // // // // // // // // // // // // INITIAL POSITION
  int i = 1;
  std::cout << "\nINITIAL POSITION - loop #" << i << "\n" << std::endl;

  ac_admitance.waitForServer(); //will wait for infinite time

  initial_position.use_left = true;
  initial_position.desired_left_pose.header.frame_id = "yumi_link_7_l";
  initial_position.pure_ft_control = false;

  initial_position.desired_left_pose.pose.position.x = 0.4;
  initial_position.desired_left_pose.pose.position.y = 0.18;
  initial_position.desired_left_pose.pose.position.z = 0.15;
  initial_position.desired_left_pose.pose.orientation.x = 1;
//   initial_position.desired_left_pose.pose.orientation.y = 0.707;

  ac_admitance.sendGoal(initial_position, &done_cb, &active_cb, &feedback_cb);
  ac_admitance.waitForResult(ros::Duration(45.0));
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();

//   going_up = initial_position;

// while(ros::ok())
// {
//
// // // // // // // // // // // // // // // // // // // // ALIGNMENT
  std::cout << "ALIGNMENT \n" << std::endl;

  ac_admitance.waitForServer();

  initial_position.pure_ft_control = true;
  ac_admitance.sendGoal(initial_position, &done_cb, &active_cb, &feedback_cb);
  ac_admitance.waitForResult(ros::Duration(0.0));
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();


// // // // // // // // // // // // // // // // // // // THREADING  
  std::cout << "BACK-SPINNING \n" << std::endl;

  ac_approach.waitForServer(); //will wait for infinite time

  ac_approach.sendGoal(back_spinning);
  ac_approach.waitForResult(ros::Duration(0.0));
  ac_approach.cancelGoal();
  ac_approach.stopTrackingGoal();


// // // // // // // // // // // // // // // // // // // THREADING
  std::cout << "THREADING \n" << std::endl;

  ac_approach.waitForServer();

  ac_approach.sendGoal(threading_task);
  ac_approach.waitForResult(ros::Duration(0.0));
  ac_approach.cancelGoal();
  ac_approach.stopTrackingGoal();


// // // // // // // // // // // // // // // // // // RECOVERING
  std::cout << "RECOVERING \n" << std::endl;

  ac_approach.waitForServer();

//   ac_approach.sendGoal(recovering_task);
  ac_approach.sendGoal(back_spinning);
  ac_approach.waitForResult(ros::Duration(0.0));
  ac_approach.cancelGoal();
  ac_approach.stopTrackingGoal();


// // // // // // // // // // // // // // // // // // GOING UP
  std::cout << "GOING UP \n" << std::endl;

  ac_approach.waitForServer();

  ac_approach.sendGoal(going_up);
  ac_approach.waitForResult(ros::Duration(0.0));
  ac_approach.cancelGoal();
  ac_approach.stopTrackingGoal();


// // // // // // // // // // // // // // // // // // INITIAL POSITION
  i++;
  std::cout << "INITIAL POSITION - loop #" << i << "\n" << std::endl;

  ac_admitance.waitForServer();

  initial_position.pure_ft_control = false;
  ac_admitance.sendGoal(initial_position, &done_cb, &active_cb, &feedback_cb);
  ac_admitance.waitForResult(ros::Duration(0.0));
  ac_admitance.cancelGoal();
  ac_admitance.stopTrackingGoal();

}


  return 0;
}
