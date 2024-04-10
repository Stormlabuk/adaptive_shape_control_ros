#include "high_controller/high_controller.hpp"

HighController::HighController() {
    des_angles_sub_ = nh_.subscribe("des_angles", 1,
                                    &HighController::desAnglesCallback, this);
    obv_angles_sub_ = nh_.subscribe("obv_angles", 1,
                                    &HighController::obvAnglesCallback, this);

    insertion_ori_sub_ = nh_.subscribe(
        "insertion_ori", 1, &HighController::insertionOriCallback, this);

    insertion_point_sub_ = nh_.subscribe(
        "insertion_point", 1, &HighController::insertionPointCallback, this);
    goal_sub_ = nh_.subscribe("goal", 1, &HighController::goalCallback, this);

    initial_imgproc_ = nh_.serviceClient<std_srvs::SetBool>("initial_imgproc");
    initial_imgproc_.waitForExistence();
    std_srvs::SetBoolRequest boolReq;
    boolReq.data = true;
    try {
        initial_imgproc_.call(boolReq);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call initial_imgproc service, error %s", e.what());
    }

    path_client_ = nh_.serviceClient<heuristic_planners::GetPath>("get_path");
    path_client_.waitForExistence();
    heuristic_planners::GetPathRequest pathReq;
    pathReq.start = insertion_point_;
    pathReq.goal = goal_;
    try {
        path_client_.call(pathReq);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call get_path service, error %s", e.what());
    }

    if (des_angles_.angles.size() != obv_angles_.angles.size()) {
        ROS_WARN("Desired and observed angles are not the same size");
        return;
    }
}

void HighController::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    ROS_INFO("Received desired angles");
    des_angles_ = *msg;
}

void HighController::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    ROS_INFO("Received observed angles");
    obv_angles_ = *msg;
}

void HighController::insertionOriCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("Received insertion orientation");
}

void HighController::insertionPointCallback(
    const geometry_msgs::Point::ConstPtr& msg) {
    ROS_INFO("Received insertion point");
    insertion_point_ = *msg;
}

void HighController::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    ROS_INFO("Received goal");
    goal_ = *msg;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "high_controller_node");
    HighController highController;
    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}