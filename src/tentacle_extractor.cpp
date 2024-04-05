#include "tentacle_extractor/tentacle_extractor.hpp"

TentacleExtractor::TentacleExtractor() : it_(nh_) {
    base_sub_ =
        it_.subscribe("base_img", 1, &TentacleExtractor::base_callback, this);

    tent_img = cv::imread(
        "/home/vittorio/ros_ws/src/adaptive_ctrl/fake_tentacle/neutral.png",
        cv::IMREAD_COLOR);

    discretise_client = nh_.serviceClient<shapeforming_msgs::DiscretiseCurve>(
        "obv_discretise_curve");

    mm_pixel_ = ros::param::param<int>("~mm_pixel", 5);
    pixel_mm_ = 1.0 / mm_pixel_;
}

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
        extract_tentacle(tent_only);
    } catch (cv::Exception& e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
        return;
    }
    return;
}

void TentacleExtractor::extract_tentacle(cv::Mat& tent_only) {
    shapeforming_msgs::DiscretiseCurveRequest req;
    std::vector<cv::Point2i> points;
    cv::findNonZero(tent_only, points);
    // Sort points based on distance from the origin
    std::sort(points.begin(), points.end(),
              [](const cv::Point2i& a, const cv::Point2i& b) {
                  double distanceA = std::sqrt(a.x * a.x + a.y * a.y);
                  double distanceB = std::sqrt(b.x * b.x + b.y * b.y);
                  return distanceA < distanceB;
              });

    // Sort points with the same distance from the origin based on x values
    std::stable_sort(
        points.begin(), points.end(),
        [](const cv::Point2i& a, const cv::Point2i& b) { return a.x < b.x; });

    // Calculate the distance covered by the points
    double distance = 0.0;
    for (int i = 1; i < points.size(); i++) {
        cv::Point2i prevPoint = points[i - 1];
        cv::Point2i currPoint = points[i];
        double dx = currPoint.x - prevPoint.x;
        double dy = currPoint.y - prevPoint.y;
        distance += std::sqrt(dx * dx + dy * dy);
    }

    // Find the next highest multiple of 10mm (converted to pixels) that covers
    // the points
    int pixelDistance = static_cast<int>(std::ceil(distance * pixel_mm_));
    int SlicedPoints = pixelDistance / 10;
    req.tentacle.num_points = SlicedPoints;
    int numPoints = points.size();

    std::vector<cv::Point> selectedPoints;
    for (int i = 0; i < SlicedPoints; i++) {
        int index = i * numPoints / SlicedPoints;
        req.tentacle.px.push_back(points[index].x);
        req.tentacle.py.push_back(points[index].y);
    }

    shapeforming_msgs::DiscretiseCurveResponse res;
    try {
        discretise_client.call(req, res);
        if (res.success) {
            ROS_INFO("Discretisation successful");
        } else {
            ROS_ERROR("Discretisation failed");
        }
    } catch (ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
        return;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tentacle_extractor");
    TentacleExtractor tentacle_extractor;
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}