#include <ros/ros.h>
#include <ros_coils/magField.h>
#include <shapeforming_msgs/error.h>
#include <shapeforming_msgs/rl_angles.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>

class ControlNode {
   private:
    ros::NodeHandle nh_;
    ros::Subscriber desAnglesSub_, obvAnglesSub_, baseFieldSub_, errorSub_;
    ros::Publisher errorPub_, adjustedField_;
    ros::Publisher spinningPub_;
    ros::ServiceServer spin_controller_srv_;

    ros::Timer calcError_;

    Eigen::Vector3d baseField_ = Eigen::Vector3d::Zero(),
                    adjField_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d held_field;
    std::vector<Eigen::Vector3d> desAngles_, obvAngles_;

   std_msgs::Bool spinning_msg_;

    int desCount_, obvCount_;
    float error_ = 0, error_prev_ = 0, error_dot_ = 0;
    bool controller_spinning_ = false;

   public:
    ControlNode();
    void desAnglesCallback(const shapeforming_msgs::rl_angles::ConstPtr& msg);
    void obvAnglesCallback(const shapeforming_msgs::rl_angles::ConstPtr& msg);
    void baseFieldCallback(const ros_coils::magField::ConstPtr& msg);
    void ComputeError(const ros::TimerEvent&);
    void adjustField();
    bool spinController(std_srvs::SetBool::Request& req,
                        std_srvs::SetBool::Response& res);
};

int main(int argc, char* argv[]);