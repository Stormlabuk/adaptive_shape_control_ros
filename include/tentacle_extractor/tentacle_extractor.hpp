#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <shapeforming_msgs/DiscretiseCurve.h>
#include <std_srvs/SetBool.h>


static const std::string OPENCV_WINDOW = "Image window";

class TentacleExtractor
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber skeleton_sub_;
    image_transport::Publisher tent_only_pub_;
    cv::Mat skeleton_img_;
    std::vector<cv::Point> tentacle_points;
    ros::ServiceClient discretise_client;
    int mm_pixel_; //!< 1mm = 5 pixel. Converts mm to pixel
    double pixel_mm_; //!< Converts pixel to mm
    double cached_distance = 0;
    int link_mm = 10; //!< Length of each link in mm
    /* data */
public:
    TentacleExtractor(/* args */);
    void skeleton_callback(const sensor_msgs::ImageConstPtr& msg);
    void extract_tentacle(cv::Mat &tent_only);
};




int main(int argc, char* argv[]);