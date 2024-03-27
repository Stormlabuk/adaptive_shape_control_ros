#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

static const std::string OPENCV_WINDOW = "Image window";

class TentacleExtractor
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber base_sub_;
    cv::Mat tent_img;
    std::vector<cv::Point> tentacle_points;

    /* data */
public:
    TentacleExtractor(/* args */);
    ~TentacleExtractor();
    void base_callback(const sensor_msgs::ImageConstPtr& msg);
};




int main(int argc, char* argv[]);