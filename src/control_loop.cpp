#include "control_loop/control_loop.hpp"

ControlNode::ControlNode() {
    obvAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "obv_angles", 1, &ControlNode::obvAnglesCallback, this);

    desAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "des_angles", 1, &ControlNode::desAnglesCallback, this);

    baseFieldSub_ = nh_.subscribe<ros_coils::magField>(
        "precomputation/baseField", 1, &ControlNode::baseFieldCallback, this);

    errorPub_ = nh_.advertise<shapeforming_msgs::error>("error", 1);
    adjustedField_ = nh_.advertise<ros_coils::magField>("adjusted_field", 1);

    calcError_ =
        nh_.createTimer(ros::Duration(4), &ControlNode::ComputeError, this);
}

void ControlNode::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    desAngles_ =
        Eigen::Vector3d(msg->angles[0], msg->angles[1], msg->angles[2]);
    desCount_ = msg->count;
}

void ControlNode::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    obvAngles_ =
        Eigen::Vector3d(msg->angles[0], msg->angles[1], msg->angles[2]);
    obvCount_ = msg->count;
}

void ControlNode::baseFieldCallback(const ros_coils::magField::ConstPtr& msg) {
    baseField_ = Eigen::Vector3d(msg->bx, msg->by, msg->bz);
}

void ControlNode::ComputeError(const ros::TimerEvent&) {
    shapeforming_msgs::error error_msg;
    error_msg.header.stamp = ros::Time::now();

    if (desCount_ != obvCount_) {
        ROS_WARN("Desired and observed angles are not synchronized");
        return;
    }
    if (desCount_ == 0 || obvCount_ == 0) {
        ROS_WARN("Desired or observed angles are not received");
        return;
    }
    ROS_INFO("Desired and observed angles are received, proceeding with loop");
    Eigen::Vector3d diff = desAngles_ - obvAngles_;
    error_ = diff.norm();
    for (int i = 0; i < diff.size(); i++) {
        error_ += diff[i] * (i + 1);
    }

    error_dot_ = error_ - error_prev_;
    error_msg.error = error_;
    error_msg.error_dot = error_dot_;
    error_prev_ = error_;
    errorPub_.publish(error_msg);
    adjustField();
}

void ControlNode::adjustField() {
    if (baseField_.norm() != 0) {
        if (error_dot_ == 0) {
            error_dot_ = 0.1;
        }
        error_dot_ = abs(error_dot_);
        if (adjField_.norm() == 0) {
            adjField_ = baseField_;
        }
        // std::cout << "\n---------\n";
        // std::cout << "Desired angles:\n" << desAngles_ << "\nObserved
        // angles:\n" << obvAngles_ << std::endl; std::cout << "Error: " <<
        // error_ << " Error_dot: " << error_dot_ << std::endl; std::cout <<
        // "Overall adjustment would be " << 0.1 * error_ / error_dot_  << "%"
        // << std::endl; std::cout << "Normalised field\n" << adjField_ /
        // adjField_.norm() << std::endl;
        adjField_ = adjField_ +
                    0.1 * error_ / error_dot_ * adjField_ / adjField_.norm();

        ros_coils::magField field_msg;
        field_msg.header.stamp = ros::Time::now();
        field_msg.bx = adjField_[0];
        field_msg.by = adjField_[1];
        field_msg.bz = adjField_[2];
        adjustedField_.publish(field_msg);
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
