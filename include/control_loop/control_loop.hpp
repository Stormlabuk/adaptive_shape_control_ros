#include <ros/ros.h>
#include <ros_coils/magField.h>
#include <shapeforming_msgs/error.h>
#include <shapeforming_msgs/rl_angles.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

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
                    adjField_ = Eigen::Vector3d::Zero(),
                    currentField_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d held_field;
    std::vector<Eigen::Vector3d> desAngles_, obvAngles_;

    std_msgs::Bool spinning_msg_;

    int desCount_, obvCount_;
    int error_ = 0, error_prev_ = 0, error_dot_ = 0;
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

    /**
     * @brief Limits the element-wise change between two vectors.
     *
     * This function takes in two vectors, `current` and `next`, and limits the element-wise change between them
     * by adjusting the `next` vector if the change exceeds the specified `limit`.
     *
     * @param current The current vector.
     * @param next The next vector.
     * @param limit The limit for element-wise change.
     * @return The adjusted `next` vector with limited element-wise change.
     */
    Eigen::Vector3d limitElementwiseChange(const Eigen::Vector3d& current,
                                           const Eigen::Vector3d& next,
                                           double limit) {
        Eigen::Vector3d adjustedNext = next;
        ROS_WARN("Limiting change. Current field: %f %f %f", current[0], current[1], current[2]);
        ROS_WARN("Limiting change. Next field: %f %f %f", next[0], next[1], next[2]);
        for (int i = 0; i < 3; ++i) {
            double change = next[i] - current[i];
            if (std::abs(change) > limit) {
                adjustedNext[i] = current[i] + (change > 0 ? limit : -limit);
            }
        }
        return adjustedNext;
    }
};

int main(int argc, char* argv[]);