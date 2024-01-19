#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

int main(int argc, char **argv)
{
    int trajectory[3] = {512,612,712};
    cv::Mat image(1024,1024,CV_8UC3);

    for(int i = 0;i<3;i++)
    {
        cv::circle(image, cv::Point(trajectory[i],trajectory[i]), 100, CV_RGB(255,0,0));
    }

#if 0
    cv::namedWindow("image", CV_WINDOW_NORMAL);
    cv::resizeWindow("image", 1024,1024);
    cv::imshow("image", image);   
    cv::waitKey(0);   
    cv::destroyWindow("image"); 
#endif

        #if 1
        ros::init(argc, argv, "draw_circle");
        ros::NodeHandle n;
        image_transport::ImageTransport it_(n);
        image_transport::Publisher image_pub_ = it_.advertise("/traj_output", 1);

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    ros::Rate loop_rate(10);
    while(ros::ok()) {


        #if 1
        ros::Time time = ros::Time::now();
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = time;
        #endif
        cv_ptr->header.frame_id = "traj_output";

        cv_ptr->image = image;
        image_pub_.publish(cv_ptr->toImageMsg());

        ROS_INFO("ImageMsg Send.");
        #endif
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;

}