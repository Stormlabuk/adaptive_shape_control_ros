#include <ros/ros.h>

#include <iostream>
#include <vector>

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

   private:
    ros::NodeHandle nh_;
};

int main(int argc, char* argv[]);