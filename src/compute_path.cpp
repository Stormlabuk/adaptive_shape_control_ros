#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <shapeforming_msgs/GetInsertion.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <opencv2/opencv.hpp>

class ComputePath {
   private:
    std::string img_path_;
    cv::Mat img_, img_proc_;
    ros::Subscriber img_path_sub_;
    ros::Publisher inserter_pos_, costmap_pub_, grid_pub_;
    std::vector<cv::Point> poly_approx_;
    cv::Point2i insertion_point;

   public:
    ComputePath() {
        ros::NodeHandle nh;
        img_path_sub_ =
            nh.subscribe("/image_path", 1, &ComputePath::imgPathCallback, this);
        inserter_pos_ = nh.advertise<geometry_msgs::Point>("/inserter_pos", 1);
        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
        grid_pub_ = nh.advertise<nav_msgs::Path>("/grid", 1);
    }

    void imgPathCallback(const std_msgs::String& msg) {
        img_path_ = msg.data;
        ROS_INFO("Image path received: %s", img_path_.c_str());
        this->img_ = cv::imread(img_path_, cv::IMREAD_GRAYSCALE);
        // Check for invalid input
        if (img_.empty()) {
            ROS_ERROR("Could not open or find the image");
            return;
        }
        cv::threshold(img_, img_proc_, 127, 255, cv::THRESH_BINARY_INV);
        this->poly_approx_ = getPoly(img_proc_);
        // get size of poly_approx_
        int n = poly_approx_.size();
        ROS_INFO("Number of points in polygon: %d", n);
        imgToCostmap(img_proc_);
    }

    cv::Point getInsertionPoint() {
        // // get the centroid of the polygon
        // cv::Moments M = cv::moments(poly_approx_);
        // cv::Point centroid = cv::Point(M.m10 / M.m00, M.m01 / M.m00);
        // // get the point closest to the centroid
        // int min_dist = INT_MAX;
        // for (int i = 0; i < poly_approx_.size(); i++) {
        //     int dist = cv::norm(poly_approx_[i] - centroid);
        //     if (dist < min_dist) {
        //         min_dist = dist;
        //         insertion_point = poly_approx_[i];
        //     }
        // }
        // return insertion_point;
        cv::RotatedRect bounding_rect = cv::minAreaRect(poly_approx_);
        cv::Point centroid = bounding_rect.center;
        double orientation = bounding_rect.angle;
        cv::Point2f vertices[4];
        bounding_rect.points(vertices);
        double a, b;
        a = cv::norm(vertices[0] - vertices[1]);
        b = cv::norm(vertices[1] - vertices[3]);
        bool is_horizontal = a > b;
        orientation = is_horizontal ? -orientation : 90 - orientation;
    }

    cv::Point constructPoint(std::vector<cv::Point> poly) {
        int x_coor, y_coor;
        // x_coor should be the average x-coordinate of the polygon
        for (int i = 0; i < poly.size(); i++){
            x_coor += poly[i].x;
        }
        x_coor = x_coor / poly.size();
        // y_coor should be the max y-coordinate of the polygon
        for (int i = 0; i < poly.size(); i++){
            y_coor = std::max(y_coor, poly[i].y);
        }
        return cv::Point(x_coor, y_coor);
        
    }

    std::vector<cv::Point> getPoly(cv::Mat img) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img, contours, hierarchy, cv::RETR_TREE,
                         cv::CHAIN_APPROX_SIMPLE);
        // find the contour with the minimum surface area
        auto min_contour = *std::min_element(
            contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a,
               const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        std::vector<cv::Point> poly;
        cv::approxPolyDP(min_contour, poly, 1, true);
        return poly;
    }

    void imgToCostmap(cv::Mat img) {
        nav_msgs::OccupancyGrid costmap;
        costmap.header.stamp = ros::Time::now();
        costmap.header.frame_id = "map";
        costmap.info.resolution = 1;
        costmap.info.width = img.cols;
        costmap.info.height = img.rows;
        costmap.info.origin.position.x = 0;
        costmap.info.origin.position.y = 0;
        costmap.info.origin.position.z = 0;
        costmap.info.origin.orientation.x = 0;
        costmap.info.origin.orientation.y = 0;
        costmap.info.origin.orientation.z = 0;
        costmap.info.origin.orientation.w = 1;
        // convert image to costmap
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                if (img.at<uchar>(i, j) == 255) {
                    costmap.data.push_back(0);
                } else {
                    costmap.data.push_back(100);
                }
            }
        }
        costmap_pub_.publish(costmap);
        ROS_INFO("Costmap published");
    }
};
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "compute_path");
    ComputePath cp;
    ROS_INFO("compute_path node started");
    ros::spin();
    return 1;
}