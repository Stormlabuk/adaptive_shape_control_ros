#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <ros_coils/magField.h>
#include <shapeforming_msgs/CalcInitialField.h>
#include <shapeforming_msgs/rl_angles.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "DataTypes.hpp"

class Precomputation {
   private:
    int E_;
    float len_, d_, v_;
    int obvJointNo_ = 0;
    int obvLinkNo = 0;
    std::vector<float> desiredAngles_;
    std::vector<float> magX_, magY_, magZ_;
    ros::ServiceServer preCalcService_;
    ros::Publisher baseFieldPub_;
    ros::NodeHandle nh_;
    /**
     * @brief Base transform from world frame to coil frame. ZYX euler angles in rad.
     * 
     */
    Vector3d baseTransform_;
    float baseX, baseY, baseZ;

   public:
    Precomputation();
    bool calculateField(shapeforming_msgs::CalcInitialField::Request &req,
                        shapeforming_msgs::CalcInitialField::Response &res);
    Vector3d rotateField(Vector3d field, Vector3d angle);
    void populateStructs(std::vector<Joint> &joints_,
                         std::vector<Link> &links_);
    MatrixXd evaluateStiffnessMatrix(std::vector<Link> &links_);
    void evaluateDirectKinematics(std::vector<Joint> &joints_,
                                  std::vector<Link> &links_);
    MatrixXd evaluateGeometricJacobian(std::vector<Joint> &joints_);
    MatrixXd MagnetisationToFieldMap(std::vector<Joint> &joints_);
    MatrixXd stackedDeformation(std::vector<Joint> &joints_);
};

int main(int argc, char *argv[]);