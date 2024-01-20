#ifndef SHAPE_SENSING_CPP
#define SHAPE_SENSING_CPP

#include <ros/ros.h>
#include <shapeforming_msgs/tent_curvature.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

using namespace cv;

Mat extractImg(const sensor_msgs::Image::ConstPtr& msg){
    Mat img = Mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
    return img;
}

// Define a callback function to handle incoming Image messages
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    extractImg(msg);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "shape_sensing_node");
    ros::NodeHandle nh;

    // Create a publisher for the "tent_curvature" message
    ros::Publisher shape_pub = nh.advertise<shapeforming_msgs::tent_curvature>("tent_curvature_topic", 10);

    // Create a subscriber for the "camera_input" topic
    ros::Subscriber image_sub = nh.subscribe("camera_input", 10, imageCallback);

    // Set the publishing rate to 1Hz
    ros::Rate rate(1);

    while (ros::ok())
    {
        // Create a new "tent_curvature" message
        shapeforming_msgs::tent_curvature msg;
        msg.num_points = 0;
        msg.px = {};
        msg.py = {};

        // TODO: Fill in the message fields with your desired values

        // Publish the message
        shape_pub.publish(msg);

        // Sleep to maintain the desired publishing rate
        rate.sleep();
    }

    return 0;
}

#endif // SHAPE_SENSING_CPP
