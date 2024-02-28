#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros_coils/magField.h>
#include <shapeforming_msgs/rl_angles.h>
#include "DataTypes.hpp"

class Precomputation {
   private:
    int E;
    float len, d, v;
   public:
    Precomputation();

};

int main(int argc, char* argv[]);