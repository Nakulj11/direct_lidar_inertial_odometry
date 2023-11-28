#include "map_align/map_align.h"

class TestNode{
    public:
        TestNode(ros::NodeHandle node_handle);

    private:

        void scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan);

        ros::NodeHandle nh;
        ros::Subscriber pcl_sub;
        map_align::MapAlign::State state;
        std::string map_link = "";
        pcl::PointCloud<PointType>::Ptr scan_;

    
};