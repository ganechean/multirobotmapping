/**
 * @author nconlon
 */

#include "grid_fusion/Fuser.hpp"

using namespace robot;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Fuser_Runner");

    ros::NodeHandle nh;

    Fuser* ptr = new Fuser(nh, "/fused_map", "/array", "/int");

    ros::spin();

    return EXIT_SUCCESS;
}
