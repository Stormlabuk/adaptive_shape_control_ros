#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros_coils/magField.h>
#include <shapeforming_msgs/CalcInitialField.h>
#include <shapeforming_msgs/rl_angles.h>

#include <eigen3/Eigen/Dense>

#include "DataTypes.hpp"

class Precomputation {
   private:
    int E_;
    float len_, d_, v_;
    int obvJointNo_ = 0;
    std::vector<float> desiredAngles_;
    std::vector<float> magX_, magY_, magZ_;

    std::vector<std::shared_ptr<Joint>> joints_;
    std::vector<std::shared_ptr<Link>> links_;
    
    ros::ServiceServer preCalcService_;
    ros::Subscriber desiredAnglesSub_;
    ros::NodeHandle nh_;

   public:
    Precomputation();
    bool calculateField(shapeforming_msgs::CalcInitialField::Request &req,
                        shapeforming_msgs::CalcInitialField::Response &res);
    void desiredAnglesCallback(const shapeforming_msgs::rl_angles::ConstPtr& msg);
    void populateStructs();
    MatrixXd evaluateStiffnessMatrix();
    void evaluateDirectKinematics();
    void evaluateGeometricJacobian();
    MatrixXd MagnetisationToFieldMap();

};

int main(int argc, char *argv[]);