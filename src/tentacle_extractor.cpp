#include "tentacle_extractor/tentacle_extractor.hpp"

TentacleExtractor::TentacleExtractor() : it_(nh_) {
    base_sub_ =
        it_.subscribe("base_img", 1, &TentacleExtractor::base_callback, this);

    tent_img = cv::imread(
        "/home/vittorio/ros_ws/src/adaptive_ctrl/fake_tentacle/neutral.png",
        cv::IMREAD_COLOR);
    cv::namedWindow(OPENCV_WINDOW);
}

TentacleExtractor::~TentacleExtractor() { cv::destroyWindow(OPENCV_WINDOW); }

void TentacleExtractor::base_callback(const sensor_msgs::ImageConstPtr& msg) {
    if (tent_img.empty()) {
        ROS_WARN("Tentacle image is not received");
        return;
    }
    cv::Mat tent_img = this->tent_img.clone();
    if (tent_img.channels() != 1) {
        cv::cvtColor(tent_img, tent_img, cv::COLOR_BGR2GRAY);
    }

    if (msg->data.empty()) {
        ROS_WARN("Base image is not received");
        return;
    }
    sensor_msgs::Image tent_msg = *msg;
    tent_msg.encoding = sensor_msgs::image_encodings::MONO8;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(tent_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat tent_only = cv::Mat::zeros(cv_ptr->image.size(), CV_8UC1);
    cv::Mat base_img = cv_ptr->image;

    if (base_img.channels() != 1) {
        cv::cvtColor(base_img, base_img, cv::COLOR_BGR2GRAY);
    }

    try {
        cv::bitwise_not(base_img, base_img);
        cv::bitwise_xor(base_img, tent_img, tent_only);

        cv::compare(tent_only, 255, tent_only, cv::CMP_EQ);


        cv::Mat element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(tent_only, tent_only, element);
        cv::ximgproc::thinning(tent_only, tent_only);
    } catch (cv::Exception& e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
        return;
    }
    // cv::imshow(OPENCV_WINDOW, tent_img);
    // cv::waitKey(3);
    return;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tentacle_extractor");
    TentacleExtractor tentacle_extractor;
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}