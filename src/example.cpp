#include <ros/ros.h>
#include <std_srvs/Trigger.h>

bool triggerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // Perform some action here
    ROS_INFO("Trigger service called");
    
    // Set the response message
    res.success = true;
    res.message = "Trigger service executed successfully";
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "trigger_node");
    ros::NodeHandle nh;
    
    // Create the service server
    ros::ServiceServer triggerService = nh.advertiseService("trigger_service", triggerCallback);
    
    // Create the service client
    ros::ServiceClient triggerClient = nh.serviceClient<std_srvs::Trigger>("trigger_service");
    
    // Create the service request
    std_srvs::Trigger srv;
    
    // Call the service
    if (triggerClient.call(srv))
    {
        ROS_INFO("Trigger service called successfully");
        ROS_INFO("Response: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call trigger service");
        return 1;
    }
    
    // Spin the node and process callbacks
    ros::spin();
    
    return 0;
}
