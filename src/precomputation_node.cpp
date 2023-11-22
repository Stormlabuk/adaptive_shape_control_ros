#include <adaptive_ctrl/rl_angles.h>
#include <ros/ros.h>
#include <ros_coils/magField.h>

void rlAnglesCallback(const adaptive_ctrl::rl_angles::ConstPtr& msg) {
    // Process the received rl_angles message and compute the magField

    // Create a magField message
    ros_coils::magField magFieldMsg;

    // Populate the magField message with the computed values

    // Publish the magField message
    ros::NodeHandle nh;
    ros::Publisher magFieldPub =
        nh.advertise<ros_coils::magField>("/base_field", 10);
    magFieldPub.publish(magFieldMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "precomputation_node");
    ros::NodeHandle nh;

    // Subscribe to the "des_angles" topic
    ros::Subscriber rlAnglesSub =
        nh.subscribe("des_angles", 10, rlAnglesCallback);

    ros::spin();

    return 0;
}
