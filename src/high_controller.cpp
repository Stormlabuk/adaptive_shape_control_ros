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
    path_client_ = nh_.serviceClient<heuristic_planners::GetPath>(
        "/planner_ros_node/request_path");
    
    precomputation_client_ =
        nh_.serviceClient<shapeforming_msgs::CalcInitialField>(
            "/precomputation/calc_initial_field");

    spin_controller_client_ = nh_.serviceClient<std_srvs::SetBool>(
        "/control_node/spin_controller");

    reinitMap();
    recalcPath();
    recalcField();

    if (des_angles_.angles.size() != obv_angles_.angles.size()) {
        ROS_WARN("Desired and observed angles are not the same size");
        return;
    }
}

/**
 * @brief sets the initial_imgproc service to true and calls service.
 * Causes costmap to be recalculated, as well as inserter to be relocated.
 * This should NOT be called with the tentacle inserted, since it will be
 * absorbed by the map.
 */
void HighController::reinitMap() {
    initial_imgproc_.waitForExistence();
    std_srvs::SetBoolRequest boolReq;
    std_srvs::SetBoolResponse boolRes;
    boolReq.data = true;
    try {
        initial_imgproc_.call(boolReq, boolRes);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call initial_imgproc service, error %s", e.what());
    }
}

/**
 * @brief recalculates the path by triggering the /planner_ros_node/request_path.
 * Goal and insertion point are fetched via subscribers.
 * Calling this service will eventually cause the path to be republished.
 * Which will in turn cause the des_angles to be recalculated by the appropriate node.
 */
void HighController::recalcPath() {
    path_client_.waitForExistence();
    heuristic_planners::GetPathRequest pathReq;
    heuristic_planners::GetPathResponse pathRes;
    pathReq.start = insertion_point_;
    pathReq.goal = goal_;
    try {
        path_client_.call(pathReq, pathRes);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call get_path service, error %s", e.what());
    }
}

/**
 * @brief recalculates the field by triggering the /precomputation/calc_initial_field.
 * Desired angles are fetched via subscribers.
 * Calling this service will eventually cause the field to be republished.
 */
void HighController::recalcField() {
    // 1. Initialise a calcinitialfield service
    shapeforming_msgs::CalcInitialFieldRequest precompReq;
    shapeforming_msgs::CalcInitialFieldResponse precompRes;

    // 2. Truncate des_angles_ to whatever subslice is suitable
    int joints_found = obv_angles_.angles.size();
    shapeforming_msgs::rl_angles des_slice;
    des_slice.count = joints_found + 1;
    des_slice.angles = std::vector<float>(
        des_angles_.angles.begin(), des_angles_.angles.begin() + joints_found);

    precompReq.tentacle.header.stamp = ros::Time::now();
    precompReq.tentacle = des_slice;
    precompReq.orientation = insertion_ori_;
    precomputation_client_.waitForExistence();
    precomputation_client_.call(precompReq, precompRes);
    if (!precompRes.success) {
        ROS_ERROR("Failed to call precomputation service");
        return;
    }
}

/**
 * @brief spins the controller by calling the spin_controller service.
 * @param spin - true to start the controller, false to stop it.
 
*/
void HighController::spinController(bool spin) {
    std_srvs::SetBoolRequest spinReq;
    std_srvs::SetBoolResponse spinRes;
    spinReq.data = spin;
    try {
        spin_controller_client_.call(spinReq, spinRes);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call spin_controller service, error %s", e.what());
    }
}

void HighController::highLoop(){
	return;
}

void HighController::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    ROS_INFO("Received desired angles");
    des_angles_ = *msg;
	if(des_angles_.angles.size() != 0) {
		recalcField();
	}
}

void HighController::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    ROS_INFO("Received observed angles");
    obv_angles_ = *msg;
}

void HighController::insertionOriCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("Received insertion orientation");
    insertion_ori_ = *msg;
	
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
        highController.highLoop();
        ros::spinOnce();
    }
    return 0;
}