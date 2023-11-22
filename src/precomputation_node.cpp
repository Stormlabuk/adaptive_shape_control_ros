#include <adaptive_ctrl/rl_angles.h>
#include <ros/ros.h>
#include <ros_coils/magField.h>
#include <std_srvs/Trigger.h>

class PrecomputationNode {
   public:
    PrecomputationNode() {
        // Initialize ROS
        

        // Subscribe to the "des_angles" topic
        rlAnglesSub = nh.subscribe("des_angles", 10,
                                   &PrecomputationNode::rlAnglesCallback, this);

        precompServer = nh.advertiseService("precomp", &PrecomputationNode::precompCallback, this);


        // Advertise the "magField" topic
        magFieldPub = nh.advertise<ros_coils::magField>("/base_field", 10);
    }

    void rlAnglesCallback(const adaptive_ctrl::rl_angles::ConstPtr& msg) {
        // Create a new "magField" message
        ROS_INFO("Received new angles");
        precompClient = nh.serviceClient<std_srvs::Trigger>("/precomp");
        // precompClient.waitForExistence();
        std_srvs::Trigger srv;
        srv.request = std_srvs::Trigger::Request();
        bool success = precompClient.call(srv);
        if (!success) {
            ROS_ERROR("Failed to call service precomp");
        }

    }

    bool precompCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        ROS_INFO("Received precomp request");
        ros_coils::magField msg;
        msg.bx = 0;
        msg.by = 0;
        msg.bz = 0;
        magFieldPub.publish(msg);
        res.success = true;
        res.message = "Precomp done";
        return true;
    }


    void run() {
        // Start the ROS node
        ros::spin();
    }

   private:
    ros::Subscriber rlAnglesSub;
    ros::Publisher magFieldPub;
    ros::ServiceClient precompClient;
    ros::ServiceServer precompServer;
    ros::NodeHandle nh;
};

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "precomputation_node");

    // Create an instance of the PrecomputationNode class
    PrecomputationNode node;

    // Run the node
    node.run();

    return 0;
}


