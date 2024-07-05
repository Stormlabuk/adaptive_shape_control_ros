#include "high_controller/high_controller.hpp"

HighController::HighController() {
    // subscriber initialisation
    des_angles_sub_ = nh_.subscribe("des_angles", 1,
                                    &HighController::desAnglesCallback, this);
    obv_angles_sub_ = nh_.subscribe("obv_angles", 1,
                                    &HighController::obvAnglesCallback, this);

    insertion_ori_sub_ = nh_.subscribe(
        "insertion_ori", 1, &HighController::insertionOriCallback, this);

    insertion_point_sub_ = nh_.subscribe(
        "insertion_point", 1, &HighController::insertionPointCallback, this);

    goal_sub_ =
        nh_.subscribe("clicked_point", 1, &HighController::goalCallback, this);

    error_sub_ =
        nh_.subscribe("error", 1, &HighController::errorCallback, this);

    // publishers initialisation
    inserter_pub_ = nh_.advertise<std_msgs::Int32>("stepper", 1);
    des_trunc_ = nh_.advertise<shapeforming_msgs::rl_angles>("des_trunc", 1);
    obv_trunc_ = nh_.advertise<shapeforming_msgs::rl_angles>("obv_trunc", 1);
    field_pub_ = nh_.advertise<ros_coils::magField>("base_field", 1);

    // ros parameters
    nh_.param<double>("error_lb", error_lb, 0);
    nh_.param<double>("error_dot__lb", error_dot_lb, 0);

    // ros service clients
    initial_imgproc_ = nh_.serviceClient<std_srvs::SetBool>("initial_imgproc");
    skeleton_init_ = nh_.serviceClient<std_srvs::SetBool>("publish_skeleton");
    path_client_ = nh_.serviceClient<heuristic_planners::GetPath>(
        "/planner_ros_node/request_path");

    precomputation_client_ =
        nh_.serviceClient<shapeforming_msgs::CalcInitialField>(
            "/precomputation/calc_initial_field");

    spin_controller_client_ =
        nh_.serviceClient<std_srvs::SetBool>("/control_node/spin_controller");

    controller_spinning_sub_ =
        nh_.subscribe("controller/spinning", 1,
                      &HighController::controllerSpinningCallback, this);

    reinitMap();
    // recalcPath();
    // recalcField();
    // std_msgs::Int32 stepper_msg;
    // stepper_msg.data = 1;

    //     inserter_pub_.publish(stepper_msg);
    // while (obv_angles_.angles.size() < 1) {
    //     inserter_pub_.publish(stepper_msg);
    // }
    //     controller_spinning_ = true;
    //     spinController(controller_spinning_);
}

void HighController::highLoop() {
    if (!controllerSpinning) {

        if(inserting){
            std_msgs::Int32 stepper_msg;
            stepper_msg.data = 1;
            inserter_pub_.publish(stepper_msg);
            if (obvJointNo_ == targetJointNo_) {
                inserting = false;
                targetReached = true;
                return;
            }
            return;
        }

        if(!inserting && !targetReached){
            obvJointNo_ = obv_angles_.count;
            targetJointNo_ = obvJointNo_ + 1 < des_angles_.count
                                ? obvJointNo_ + 1
                                : des_angles_.count;
            inserting = true;
            return;
        }

        // int obvJointNo = obv_angles_.count;
        // int targetJointNo = obvJointNo + 1 < des_angles_.count
        //                         ? obvJointNo + 1
        //                         : des_angles_.count;

        Vector3d currField = fields_[obvJointNo_ - 2];
        ros_coils::magField fieldMsg;
        fieldMsg.bx = currField.x();
        fieldMsg.by = currField.y();
        fieldMsg.bz = currField.z();

        shapeforming_msgs::rl_angles obv_slice = obv_angles_;
        shapeforming_msgs::rl_angles des_slice;
        des_slice.angles =
            std::vector<float>(des_angles_.angles.begin(),
                               des_angles_.angles.begin() + obvJointNo_);
        des_slice.count = obvJointNo_;
        des_trunc_.publish(des_slice);
        obv_trunc_.publish(obv_slice);
        field_pub_.publish(fieldMsg);

        controllerSpinning = true;
        spinController(true);

        //when controller is done
        // controllerSpinning = false;
        // spinController(controllerSpinning);
        // targetReached = false;
    }
    return;
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
 * @brief recalculates the path by triggering the
 * /planner_ros_node/request_path. Goal and insertion point are fetched via
 * subscribers. Calling this service will eventually cause the path to be
 * republished. Which will in turn cause the des_angles to be recalculated by
 * the appropriate node.
 */
void HighController::recalcPath() {
    path_client_.waitForExistence();
    heuristic_planners::GetPathRequest pathReq;
    heuristic_planners::GetPathResponse pathRes;
    if (insertion_point_.x == 0 && insertion_point_.y == 0 &&
        insertion_point_.z == 0) {
        ROS_WARN("Insertion point is 0,0,0");
        return;
    }
    if (goal_.x == 0 && goal_.y == 0 && goal_.z == 0) {
        ROS_WARN("Goal is 0,0,0");
        return;
    }
    goal_.z = 0;
    // ROS_INFO("Given insertion point: %f, %f, %f", insertion_point_.x,
    // insertion_point_.y, insertion_point_.z); ROS_INFO("Given goal: %f, %f,
    // %f", goal_.x, goal_.y, goal_.z);

    pathReq.start = insertion_point_;
    pathReq.goal = goal_;
    pathReq.goal.z = 0;
    try {
        path_client_.call(pathReq, pathRes);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call get_path service, error %s", e.what());
    }
}

/**
 * @brief recalculates the field by triggering the
 * /precomputation/calc_initial_field. Desired angles are fetched via
 * subscribers. Calling this service will eventually cause the field to be
 * republished.
 */
void HighController::recalcField() {
    // by this point we have already guaranteed that des_angles != 0

    // 1. Initialise a calcinitialfield service
    shapeforming_msgs::CalcInitialFieldRequest precompReq;
    shapeforming_msgs::CalcInitialFieldResponse precompRes;

    ROS_INFO("Recived a path made of %d joints. Will calculate %d fields.",
             des_angles_.count, obv_angles_.count - 1);

    // say our path is 5 joints
    // that means 4 links
    // and 4 fields

    // des_angles_.count = 5
    // des_angles_.angles = [10, 20, 30, 40, 50]

    // for i < des_angles_.count

    // i = 0
    // des_slice.angles = des_angles[begin,begin+0] [10]

    // i = 1
    // des_slice.angles = des_angles[begin,begin+1] [10, 20]

    // i = 2
    // des_slice.angles = des_angles[begin,begin+2] [10, 20, 30]

    // i = 3
    // des_slice.angles = des_angles[begin,begin+3] [10, 20, 30, 40]

    // i = 4
    // des_slice.angles = des_angles[begin,begin+4] [10, 20, 30, 40, 50]

    for (int i = 0; i < des_angles_.count; i++) {
        shapeforming_msgs::rl_angles des_slice;
        des_slice.angles = std::vector<float>(
            des_angles_.angles.begin(), des_angles_.angles.begin() + i + 1);
        des_slice.count = i + 1;
        precompReq.tentacle = des_slice;
        precompReq.orientation = insertion_ori_;
        precompReq.tentacle.header.stamp = ros::Time::now();
        precomputation_client_.waitForExistence();
        precomputation_client_.call(precompReq, precompRes);
        if (!precompRes.success) {
            ROS_ERROR("Failed to call precomputation service at iteration %d",
                      i);
            return;
        }
        fields_.push_back(Vector3d(precompRes.field.bx, precompRes.field.by,
                                   precompRes.field.bz));
    }

    // ROS_INFO("Calculating field, joints found: %d, path length in joints :
    // %d",
    //  obv_angles_.angles.size(), des_angles_.angles.size());

    // // 2. Truncate des_angles_ to whatever subslice is suitable
    // int joints_found = obv_angles_.angles.size();
    // if (joints_found != 0 && des_angles_.angles.size() != 0){
    //     shapeforming_msgs::rl_angles des_slice, obv_slice;
    //     des_slice.count = joints_found + 1;
    //     des_slice.angles =
    //         std::vector<float>(des_angles_.angles.begin(),
    //                            des_angles_.angles.begin() + joints_found);
    //     obv_slice.count = des_slice.count;
    //     obv_slice.angles =
    //         std::vector<float>(obv_angles_.angles.begin(),
    //                            obv_angles_.angles.begin() + joints_found);
    //     precompReq.tentacle.header.stamp = ros::Time::now();
    //     precompReq.tentacle = des_slice;
    //     precompReq.orientation = insertion_ori_;
    //     precomputation_client_.waitForExistence();
    //     precomputation_client_.call(precompReq, precompRes);
    //     if (!precompRes.success) {
    //         ROS_ERROR("Failed to call precomputation service");
    //         return;
    //     }
    //     // ROS_INFO("Truncated angles and recalculated field");
    //     des_trunc_.publish(des_slice);
    //     obv_trunc_.publish(obv_slice);
    // } else
    //     return;
}

void HighController::spinController(bool spin) {
    std_srvs::SetBoolResponse spinRes;
    spinReq_.data = spin;
    try {
        spin_controller_client_.call(spinReq_, spinRes);
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to call spin_controller service, error %s", e.what());
    }
}

void HighController::errorCallback(
    const shapeforming_msgs::error::ConstPtr& msg) {
    ROS_INFO("Received error values");
    error_ = *msg;
}

void HighController::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    ROS_INFO("Received desired angles");
    des_angles_ = *msg;
    if (des_angles_.angles.size() != 0) {
        recalcField();
    }
}

void HighController::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    // ROS_INFO("Received observed angles");
    obv_angles_ = *msg;
}

void HighController::insertionOriCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
    // ROS_INFO("Received insertion orientation");
    insertion_ori_ = *msg;
}

void HighController::insertionPointCallback(
    const geometry_msgs::Point::ConstPtr& msg) {
    // ROS_INFO("Received insertion point");
    insertion_point_ = *msg;
}

void HighController::goalCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
    // ROS_INFO("Received goal");
    goal_ = msg->point;
    recalcPath();
    // recalcField();
}

void HighController::controllerSpinningCallback(
    const std_msgs::Bool::ConstPtr& msg) {
    controllerSpinning = msg->data;
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