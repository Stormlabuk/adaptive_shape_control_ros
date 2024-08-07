#include "tentacle_extractor/tentacle_extractor.hpp"

TentacleExtractor::TentacleExtractor() : it_(nh_) {
    skeleton_sub_ = it_.subscribe("tentacle_img", 1,
                                  &TentacleExtractor::skeleton_callback, this);

    discretise_client = nh_.serviceClient<shapeforming_msgs::DiscretiseCurve>(
        "obv_discretise_curve");

    mm_pixel_ = ros::param::param<int>("mm_to_pixel", 15);
    double link_float = ros::param::param<double>("/precomputation/len", 10.0); 
    link_mm = static_cast<int>(link_float * 1000) - 2;
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
    // ROS_INFO("Number of points: %lu", points.size());
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

    // create an array containing the distance between each point and the first
    // point in the rray
    std::vector<double> distances;
    distances.push_back(0.0);
    for (int i = 1; i < points.size(); i++) {
        cv::Point2i firstPoint = points[i - 1];
        cv::Point2i currPoint = points[i];
        double dx = currPoint.x - firstPoint.x;
        double dy = currPoint.y - firstPoint.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        distances.push_back(distance);

    }
    std::vector<double> distances_cumulative;
    distances_cumulative.push_back(0.0);
    for (int i = 1; i < distances.size(); i++) {
        distances_cumulative.push_back(distances_cumulative[i - 1] +
                                       distances[i]);
    }

    double totalDistance = distances_cumulative.back();

    // Find the next highest multiple of 10mm (converted to pixels) that covers
    // the points
    int link_px = link_mm * mm_pixel_;    
    int numLinks = std::floor(totalDistance / link_px);
    
    if (totalDistance < link_px) {
        // ROS_INFO("Tentacle is too short");
        req.tentacle.num_points = 0;
        req.tentacle.px.clear();
        req.tentacle.py.clear();
        shapeforming_msgs::DiscretiseCurveResponse res;
        try {
            discretise_client.call(req, res);
        } catch (ros::Exception& e) {
            ROS_ERROR("ROS exception: %s", e.what());
            return;
        }
        return;
    }

    if (numLinks > 1) {
        int linksToFind = numLinks - 1;
        req.tentacle.num_points = numLinks + 1;
        req.tentacle.px.resize(numLinks + 1);
        req.tentacle.py.resize(numLinks + 1);
        req.tentacle.px[0] = points[0].x;
        req.tentacle.py[0] = points[0].y;
        req.tentacle.px[numLinks] = points.back().x;
        req.tentacle.py[numLinks] = points.back().y;

        for (int i = 0; i < linksToFind; i++) {
            double targetDistance = (i + 1) * link_px;
            int j = 0;
            while (distances_cumulative[j] < targetDistance) {
                j++;
            }
            double ratio =
                (targetDistance - distances_cumulative[j - 1]) /
                (distances_cumulative[j] - distances_cumulative[j - 1]);
            req.tentacle.px[i + 1] =
                points[j - 1].x + ratio * (points[j].x - points[j - 1].x);
            req.tentacle.py[i + 1] =
                points[j - 1].y + ratio * (points[j].y - points[j - 1].y);
        }
    } else if (numLinks == 1) {
        req.tentacle.num_points = 2;
        req.tentacle.px.resize(2);
        req.tentacle.py.resize(2);
        req.tentacle.px[0] = points[0].x;
        req.tentacle.py[0] = points[0].y;
        req.tentacle.px[1] = points.back().x;
        req.tentacle.py[1] = points.back().y;
    } else {
        ROS_INFO("Tentacle is too short");
        req.tentacle.num_points = 0;
        req.tentacle.px.clear();
        req.tentacle.py.clear();
        return;
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
        ros::Rate(15).sleep();
    }

    return 0;
}