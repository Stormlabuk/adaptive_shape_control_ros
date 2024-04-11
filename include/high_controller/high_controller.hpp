// ros includes
#include <ros/exception.h>
#include <ros/ros.h>

// std includes
#include <iostream>
#include <vector>

// ros messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <shapeforming_msgs/rl_angles.h>
#include <shapeforming_msgs/error.h>

// ros services
#include <heuristic_planners/GetPath.h>
#include <shapeforming_msgs/CalcInitialField.h>
#include <std_srvs/SetBool.h>

/**
 * @todo
 * Class will control:
 *      1. Costmap publishing through the /initial_imgproc service
 *      2. Path calculation (listen to insertion_point and insertion_ori)
 *      3. Figure out current number of inserted links
 *      4. Get base field for number + 1
 *      6. Apply and insert until number+1 is reached
 *      5. Match sizes of obv and des angles
 *      6. Get controller spinning
 *      7. Repeat when controller is done
 */

class HighController {
   public:
    HighController();

    void desAnglesCallback(const shapeforming_msgs::rl_angles::ConstPtr& msg);
    void obvAnglesCallback(const shapeforming_msgs::rl_angles::ConstPtr& msg);
    void insertionOriCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void insertionPointCallback(const geometry_msgs::Point::ConstPtr& msg);
    void goalCallback(const geometry_msgs::Point::ConstPtr& msg);
    void errorCallback(const shapeforming_msgs::error::ConstPtr& msg);

    void highLoop();
    void reinitMap();
    void recalcField();
    void recalcPath();
    void spinController(bool spin);

    double error_lb = 0, error_dot_lb = 0;

   private:
    ros::NodeHandle nh_;
    ros::ServiceClient initial_imgproc_, path_client_, precomputation_client_,
        spin_controller_client_;

    ros::Subscriber insertion_point_sub_, insertion_ori_sub_, goal_sub_;
    ros::Subscriber des_angles_sub_, obv_angles_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber error_sub_;

    ros::Publisher des_trunc_, obv_trunc_;
    ros::Publisher inserter_pub_;

    geometry_msgs::Point goal_ = geometry_msgs::Point(),
                         insertion_point_ = geometry_msgs::Point();
    geometry_msgs::Vector3 insertion_ori_ = geometry_msgs::Vector3();

    shapeforming_msgs::rl_angles des_angles_, obv_angles_;
    shapeforming_msgs::error error_;

};

int main(int argc, char* argv[]);