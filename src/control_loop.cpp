#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <adaptive_ctrl/rl_angles.h>
#include <adaptive_ctrl/error.h>
#include <ros_coils/magField.h>


void desAnglesCallback(const adaptive_ctrl::rl_angles::ConstPtr msg)
{
    // Process desired angles
    std::vector<float> desAngles = msg->angles;
    // TODO: Add your desired angles processing code here
}

void obvAnglesCallback(const adaptive_ctrl::rl_angles::ConstPtr& msg)
{
    // Process observed angles
    std::vector<float> obvAngles = msg->angles;
    // TODO: Add your observed angles processing code here
}

void baseFieldCallback(const ros_coils::magField::ConstPtr& msg)
{
    // Process base field
    Eigen::Vector3f baseField = Eigen::Vector3f(msg->bx, msg->by, msg->bz);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_loop_node");
    ros::NodeHandle nh;

    // Subscribe to desired angles topic
    ros::Subscriber desAnglesSub = nh.subscribe("/des_angles", 10, desAnglesCallback);

    // Subscribe to observed angles topic
    ros::Subscriber obvAnglesSub = nh.subscribe("/obv_angles", 10, obvAnglesCallback);

    // Subscribe to base field topic
    ros::Subscriber baseFieldSub = nh.subscribe("/base_field", 10, baseFieldCallback);

    // Create publishers
    ros::Publisher fieldPub, errorPub;

    // Create publisher for field topic
    fieldPub = nh.advertise<ros_coils::magField>("/field", 10);
    errorPub = nh.advertise<adaptive_ctrl::error>("/error", 10);
    // TODO: Add your control loop code here

    ros::spin();

    return 0;
}
