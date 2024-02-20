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
    cv::Mat img_, img_proc_, img_phantom_only;
    ros::Subscriber img_path_sub_;
    ros::Publisher inserter_pos_, costmap_pub_, grid_pub_;
    ros::Publisher insertion_marker;
    std::vector<cv::Point> poly_approx_;
    cv::Point2i insertion_point;

   public:
    ComputePath() {
        ros::NodeHandle nh;
        img_path_sub_ =
            nh.subscribe("/image_path", 1, &ComputePath::imgPathCallback, this);
        inserter_pos_ = nh.advertise<geometry_msgs::Point>("/inserter_pos", 1);
        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
        grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
        insertion_marker = nh.advertise<visualization_msgs::Marker>(
            "/ins_pos", 1);
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
        // ROS_INFO("Number of points in polygon: %d", n);

        insertion_point = getInsertionPoint();
        geometry_msgs::Point ins_point;
        ins_point.x = insertion_point.x;
        ins_point.y = insertion_point.y;
        ins_point.z = 0;
        imgToCostmap(this->img_phantom_only);
        imgToGrid(this->img_proc_);
        inserter_pos_.publish(ins_point);
        visualization_msgs::Marker marker = populateMarker(std::vector<cv::Point>{insertion_point});
        insertion_marker.publish(marker);
    }

    cv::Point getInsertionPoint() {
        cv::RotatedRect bounding_rect = cv::minAreaRect(this->poly_approx_);
        cv::Point centroid = bounding_rect.center;
        double orientation = bounding_rect.angle;
        cv::Point2f vertices[4];
        bounding_rect.points(vertices);
        double a, b;
        a = cv::norm(vertices[0] - vertices[1]);
        b = cv::norm(vertices[0] - vertices[3]);
        bool is_horizontal = a > b;
        orientation = is_horizontal ? -orientation :-orientation - 90;
        std::vector<cv::Point> rotated_poly =
            rotatePolygon(poly_approx_, orientation, centroid);
        cv::Point insertion_point = constructPoint(rotated_poly);
        insertion_point = rotatePolygon(std::vector<cv::Point>{insertion_point},
                                        -orientation, centroid)[0];
        // check that this->img_proc_ is a 3-channel image
        this->img_phantom_only = this->img_proc_.clone();
        fillPoly(this->img_phantom_only, std::vector<std::vector<cv::Point>>{this->poly_approx_}, cv::Scalar(255));
        
        return insertion_point;
    }

    cv::Point constructPoint(std::vector<cv::Point> poly) {
        int x_coor = 0, y_coor = 0;
        // x_coor should be the average x-coordinate of the polygon
        for (int i = 0; i < poly.size(); i++) {
            x_coor += poly[i].x;
        }
        x_coor = x_coor / poly.size();
        // y_coor should be the max y-coordinate of the polygon
        for (int i = 0; i < poly.size(); i++) {
            y_coor = std::max(y_coor, poly[i].y);
        }
        return cv::Point(x_coor, y_coor);
    }

    std::vector<cv::Point> rotatePolygon(std::vector<cv::Point> poly,
                                           float angle,
                                           const cv::Point& centroid) {
        // Convert angle from degrees to radians
        double angle_rad = angle * CV_PI / 180.0;

        // Create rotation matrix
        cv::Mat rotmatMan = (cv::Mat_<double>(2, 2) << cos(angle_rad),
                             -sin(angle_rad), sin(angle_rad), cos(angle_rad));

        std::vector<cv::Point> rotatedPoly;
        for (const auto& point : poly) {
            cv::Mat pointMat = (cv::Mat_<double>(2, 1) << point.x - centroid.x,
                                point.y - centroid.y);
            cv::Mat rotatedPointMat = rotmatMan * pointMat;
            rotatedPoly.push_back(
                cv::Point(rotatedPointMat.at<double>(0, 0) + centroid.x,
                            rotatedPointMat.at<double>(1, 0) + centroid.y));
        }

        return rotatedPoly;
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

    void imgToCostmap(const cv::Mat &img) {
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

        cv::Mat kernel = cv::Mat::ones(3, 3, CV_32F);
        cv::Mat diffused_img;
        cv::Mat structuringElement = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(11, 11), cv::Point(-1, -1));
        cv::erode(img, img, structuringElement);
        cv::filter2D(img, diffused_img, -1, kernel, cv::Point(-1, -1), 0,
                     cv::BORDER_DEFAULT);

        this->addOcclusion(diffused_img, 15, 20, 10);

        for (int i = 0; i < diffused_img.rows; i++) {
            for (int j = 0; j < diffused_img.cols; j++) {
                if (diffused_img.at<uchar>(i, j) == 255) {
                    costmap.data.push_back(0);
                } else {
                    costmap.data.push_back(100);
                }
            }
        }
        // cv::normalize(diffused_img, diffused_img, 0, 100, cv::NORM_MINMAX);
        // for (int i = 0; i < diffused_img.rows; i++) {
        //     for (int j = 0; j < diffused_img.cols; j++) {
        //         costmap.data.push_back(100 - diffused_img.at<uchar>(i, j));
        //     }
        // }

        costmap_pub_.publish(costmap);
        ROS_INFO("Costmap published");
    }

    void addOcclusion(cv::Mat img, int radius, int xoffset, int yoffset){
        cv::Point centreLeft = this->insertion_point + cv::Point(-xoffset, yoffset);
        cv::Point centreRight = this->insertion_point + cv::Point(xoffset, yoffset);
        cv::circle(img, centreLeft, radius, cv::Scalar(0), -1);
        cv::circle(img, centreRight, radius, cv::Scalar(0), -1);
    }

    void imgToGrid(const cv::Mat &img) {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "map";
        grid.info.resolution = 1;
        grid.info.width = img.cols;
        grid.info.height = img.rows;
        grid.info.origin.position.x = 0;
        grid.info.origin.position.y = 0;
        grid.info.origin.position.z = 0;
        grid.info.origin.orientation.x = 0;
        grid.info.origin.orientation.y = 0;
        grid.info.origin.orientation.z = 0;
        grid.info.origin.orientation.w = 1;
        // convert image to costmap
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                if (img.at<uchar>(i, j) == 255) {
                    grid.data.push_back(0);
                } else {
                    grid.data.push_back(100);
                }
            }
        }
        grid_pub_.publish(grid);
        ROS_INFO("Grid published");
    }
    visualization_msgs::Marker populateMarker(std::vector<cv::Point> points){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "ins_pos";
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 10;
        marker.scale.y = 10;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.color.a = 1.0;
        for (int i = 0; i < points.size(); i++) {
            geometry_msgs::Point p;
            p.x = points[i].x;
            p.y = points[i].y;
            p.z = 0;
            marker.points.push_back(p);
        }
        return marker;
    }
};
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "compute_path");
    ComputePath cp;
    ROS_INFO("compute_path node started");
    ros::spin();
    return 1;
}