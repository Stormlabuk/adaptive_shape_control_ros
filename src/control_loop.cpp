#include "control_loop/control_loop.hpp"

ControlNode::ControlNode() {
    obvAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "obv_angles", 1, &ControlNode::obvAnglesCallback, this);

    desAnglesSub_ = nh_.subscribe<shapeforming_msgs::rl_angles>(
        "des_angles", 1, &ControlNode::desAnglesCallback, this);
    
    baseFieldSub_ = nh_.subscribe<ros_coils::magField>(
        "base_field", 1, &ControlNode::baseFieldCallback, this);

    errorPub_ = nh_.advertise<shapeforming_msgs::error>("error", 1);

    calcError_ = nh_.createTimer(ros::Duration(0.5), &ControlNode::ComputeError, this);
}

void ControlNode::desAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    desAngles_ = Eigen::Vector3d(msg->angles[0], msg->angles[1], msg->angles[2]);
    desCount_ = msg->count;
}

void ControlNode::obvAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr& msg) {
    obvAngles_ = Eigen::Vector3d(msg->angles[0], msg->angles[1], msg->angles[2]);
    obvCount_ = msg->count;
}

void ControlNode::baseFieldCallback(
    const ros_coils::magField::ConstPtr& msg) {
    baseField_ = Eigen::Vector3d(msg->bx, msg->by, msg->bz);
}

void ControlNode::ComputeError(const ros::TimerEvent&) {
    shapeforming_msgs::error error_msg;
    error_msg.header.stamp = ros::Time::now();
    
    if(desCount_ != obvCount_) {
        ROS_WARN("Desired and observed angles are not synchronized");
        return;
    }
    if(desCount_ == 0 || obvCount_ == 0) {
        ROS_WARN("Desired or observed angles are not received");
        return;
    }
    Eigen::Vector3d diff = desAngles_ - obvAngles_;
    error_ = diff.norm();
    error_dot_ = error_ - error_prev_;
    error_msg.error = error_;
    error_msg.error_dot = error_dot_;
    error_prev_ = error_;
    errorPub_.publish(error_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "control_loop_node");
    ControlNode controlNode;
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
