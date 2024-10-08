#include "control_loop/control_loop.hpp"

ControlNode::ControlNode() {
    obvAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "obv_trunc", 1, &ControlNode::obvAnglesCallback, this);

    desAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "des_trunc", 1, &ControlNode::desAnglesCallback, this);

    baseFieldSub_ = nh_.subscribe<ros_coils::magField>(
        "base_field", 1, &ControlNode::baseFieldCallback, this);

    errorPub_ = nh_.advertise<shapeforming_msgs::error>("error", 1);
    adjustedField_ = nh_.advertise<ros_coils::magField>("field", 1);

    calcError_ =
        nh_.createTimer(ros::Duration(4), &ControlNode::ComputeError, this);
    calcError_.stop();
    spin_controller_srv_ = nh_.advertiseService(
        "/control_node/spin_controller", &ControlNode::spinController, this);

    spinningPub_ = nh_.advertise<std_msgs::Bool>("controller/spinning", 1);
}

bool ControlNode::spinController(std_srvs::SetBool::Request& req,
                                 std_srvs::SetBool::Response& res) {
    if (req.data) {
        calcError_.start();
        res.success = true;
        res.message = "Controller started";
        controller_spinning_ = true;
    } else {
        calcError_.stop();
        res.success = true;
        res.message = "Controller stopped";
        controller_spinning_ = false;
        adjField_ = Eigen::Vector3d::Zero();
    }
    spinning_msg_.data = controller_spinning_;

    return res.success;
}

void ControlNode::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    desAngles_.clear();
    for (auto i : msg->angles) {
        desAngles_.push_back(Eigen::Vector3d(0, i, 0));
    }
    desCount_ = msg->count;
}

void ControlNode::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    obvAngles_.clear();
    for (auto i : msg->angles) {
        obvAngles_.push_back(Eigen::Vector3d(0, i, 0));
    }
    obvCount_ = msg->count;
}

void ControlNode::baseFieldCallback(const ros_coils::magField::ConstPtr& msg) {
    baseField_ = Eigen::Vector3d(msg->bx, msg->by, msg->bz);
    ROS_INFO("CL: Base field received: %f, %f, %f", baseField_[0],
             baseField_[1], baseField_[2]);
    ros_coils::magField field_msg;

    if (currentField_.norm() == 0) {
        currentField_ = baseField_;
        field_msg.header.stamp = ros::Time::now();
        field_msg.bx = currentField_[0];
        field_msg.by = currentField_[1];
        // field_msg.bz = 12;
        adjustedField_.publish(field_msg);
        return;
    } else {
        ROS_WARN("CL: Base field received, limiting the change");
        ROS_WARN("CL: Base field: %f, %f, %f", baseField_[0], baseField_[1],
                 baseField_[2]);
        baseField_ = limitElementwiseChange(currentField_, baseField_, 15);
        ROS_WARN("CL: Adjusted Base field: %f, %f, %f", baseField_[0], baseField_[1],
                 baseField_[2]);
    }
    field_msg.header.stamp = ros::Time::now();
    field_msg.bx = baseField_[0];
    field_msg.by = baseField_[1];
    field_msg.bz = baseField_[2];
    adjustedField_.publish(field_msg);
    currentField_ = baseField_;
}

void ControlNode::ComputeError(const ros::TimerEvent&) {
    shapeforming_msgs::error error_msg;
    error_msg.header.stamp = ros::Time::now();

    // ROS_INFO("OL: Computing the error. Desired Angles:");
    // for (auto i : desAngles_) {
    //     ROS_INFO("%f, %f, %f ", i.x(), i.y(), i.z());
    // }
    // ROS_INFO("Observed Angles:");
    // for (auto i : obvAngles_) {
    //     ROS_INFO("%f, %f, %f ", i.x(), i.y(), i.z());
    // }

    if (desCount_ != obvCount_) {
        ROS_WARN("Desired count %d and observed count %d are not synchronized",
                 desCount_, obvCount_);
        return;
    }
    if (desCount_ == 0 || obvCount_ == 0) {
        ROS_WARN("Desired or observed angles are not received");
        return;
    }
    ROS_INFO(
        "OL:Desired and observed angles are received, proceeding with loop");

    std::vector<Eigen::Vector3d> diff(desAngles_.size());
    for (int i = 0; i < desAngles_.size(); i++) {
        diff[i] = (obvAngles_[i] - desAngles_[i]) * (1 + i);
        // Eigen::Vector3d((obvAngles_[i].x() - desAngles_[i].x()) * (i + 1),
        //                 (obvAngles_[i].y() - desAngles_[i].y()) * (i + 1),
        //                 (obvAngles_[i].z() - desAngles_[i].z()) * (i + 1));
    }
    Eigen::Vector3d diffCollapsed = Eigen::Vector3d::Zero();
    for (int i = 0; i < diff.size(); i++) {
        diffCollapsed += diff[i];
    }

    error_ = diffCollapsed.norm() / obvCount_;

    error_dot_ = error_ - error_prev_;
    error_msg.error = error_;
    error_msg.error_dot = error_dot_;
    error_prev_ = error_;
    errorPub_.publish(error_msg);
    spinningPub_.publish(spinning_msg_);
    adjustField();
}

void ControlNode::adjustField() {
    if (baseField_.norm() != 0) {
        // error_dot_ = error_dot_ * -1;
        
        if (adjField_.norm() == 0) {
            adjField_ = baseField_;
        }

        ROS_INFO("OL: Base field: %f, %f, %f", baseField_[0], baseField_[1],
                 baseField_[2]);

        ROS_INFO("OL: Adjusted field: %f, %f, %f", adjField_[0], adjField_[1],
                 adjField_[2]);

        ros_coils::magField field_msg;
        field_msg.header.stamp = ros::Time::now();
        field_msg.bx = adjField_[0];
        field_msg.by = adjField_[1];

        // field_msg.bz = desCount_;
        adjustedField_.publish(field_msg);
        spinning_msg_.data = false;
        spinningPub_.publish(spinning_msg_);
    } else {
        ROS_WARN("Base field is not received");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_loop_node");
    ControlNode controlNode;
    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
