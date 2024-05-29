#include "tentacle_extractor/tentacle_extractor.hpp"

TentacleExtractor::TentacleExtractor() : it_(nh_) {
    skeleton_sub_ = it_.subscribe("tentacle_img", 1,
                                  &TentacleExtractor::skeleton_callback, this);



    discretise_client = nh_.serviceClient<shapeforming_msgs::DiscretiseCurve>(
        "obv_discretise_curve");

    mm_pixel_ = ros::param::param<int>("~mm_pixel", 15);
    pixel_mm_ = 1.0 / mm_pixel_;
}

void TentacleExtractor::skeleton_callback(
    const sensor_msgs::ImageConstPtr& msg) {
    if (msg->data.empty()) {
        ROS_WARN("Skeleton image is not received");
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    skeleton_img_ = cv_ptr->image;
    // cv::imshow(OPENCV_WINDOW, skeleton_img_);
    // cv::waitKey(3);
    if (skeleton_img_.rows == 0 || skeleton_img_.cols == 0) {
        ROS_WARN("Base image is not received");
        return;
    }
    try {
        extract_tentacle(skeleton_img_);
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
    int numPoints = points.size();
    int link_px = link_mm * mm_pixel_;
    int SlicedPoints = numPoints / link_px;
    req.tentacle.num_points = SlicedPoints;
    ROS_INFO("Number of points: %d", numPoints);
    ROS_INFO("Number of sliced points: %d", SlicedPoints);
        
    std::vector<cv::Point> selectedPoints;
    for (int i = 0; i < SlicedPoints; i++) {
        int index = i * numPoints / SlicedPoints;
        req.tentacle.px.push_back(points[index].x);
        req.tentacle.py.push_back(points[index].y);
    }

    shapeforming_msgs::DiscretiseCurveResponse res;
    try {
        discretise_client.call(req, res);
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