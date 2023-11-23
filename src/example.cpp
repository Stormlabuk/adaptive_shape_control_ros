#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

void boolCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // Instantiate a service client
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("trigger_service");

  // Call the service
  std_srvs::Trigger srv;
  if (client.call(srv))
  {
    ROS_INFO("Service call successful");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_node");
  ros::NodeHandle nh;

  // Subscribe to the bool topic
  ros::Subscriber sub = nh.subscribe("bool_topic", 10, boolCallback);

  ros::spin();

  return 0;
}
