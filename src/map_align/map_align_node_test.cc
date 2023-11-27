#include "map_align/map_align_node.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "map_align_node");
    ros::NodeHandle nh("~");

    TestNode test(nh);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}