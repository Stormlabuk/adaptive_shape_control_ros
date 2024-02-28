#include "precomputation/precomputation.hpp"


Precomputation::Precomputation(){
    ros::NodeHandle nh;
    nh.param<int>("precomputation/E", E, 100e3);
    nh.param<float>("precomputation/len", len, 10e-3);
    nh.param<float>("precomputation/d", d, 2e-3);
    nh.param<float>("precomputation/v", v, 0.43);
    ROS_INFO("E: %d, len: %f, d: %f, v: %f", E, len, d, v);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "precomputation");
    Precomputation precomputation;
    ros::spin();
    return 0;
}
