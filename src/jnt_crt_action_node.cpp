#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void chatterCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  //ROS_INFO("I heard: [%s]", msg);
    std::cout << *msg << std::endl;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "joint_cartesian_states");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/yumi/joint_states", 1, chatterCallback);

  ros::spin();

  return 0;
}
