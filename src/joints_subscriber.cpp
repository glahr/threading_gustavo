#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>

//namespace joints_subscriber
//{

	void chatterCallback(const sensor_msgs::JointState& msg)
	{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
		std::cout << msg << std::endl;
	
	}



	int main(int argc, char **argv)
	{
	ros::init(argc, argv, "joints_subscriber");

	ros::NodeHandle n;

	ros::Subscriber joints_sub = n.subscribe("/yumi/joint_states", 1000, chatterCallback);

	ros::spin();

	return 0;
	}
	
//}