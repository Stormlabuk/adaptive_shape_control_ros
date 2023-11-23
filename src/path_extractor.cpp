#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <adaptive_ctrl/rl_angles.h>

class CameraSubscriber
{
public:
    CameraSubscriber()
    {
        ros::NodeHandle nh;
        camera_sub_ = nh.subscribe("/camera_input", 10, &CameraSubscriber::cameraCallback, this);
        des_angles_pub_ = nh.advertise<adaptive_ctrl::rl_angles>("/des_angles", 10);
    }

    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        // Process the camera input here
    }

private:
    ros::Subscriber camera_sub_;
    ros::Publisher des_angles_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_subscriber");

    CameraSubscriber camera_subscriber;

    ros::spin();

    return 0;
}
