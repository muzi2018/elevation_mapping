#include "ros/ros.h"
#include "std_msgs/String.h"

// Callback function to process the received message
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "check_map");
  ros::NodeHandle nh;

  // Subscribe to the topic "chatter"
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // Enter a loop, pumping callbacks
  ros::spin();

  return 0;
}
