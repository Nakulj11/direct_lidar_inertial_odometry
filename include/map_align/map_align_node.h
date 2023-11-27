#include "map_align/map_align.h"

class TestNode{
    public:
        TestNode(ros::NodeHandle node_handle);

    private:

        void scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan);

        ros::NodeHandle nh;
        ros::Subscriber pcl_sub;
        pcl::PointCloud<PointType>::Ptr scan_;

    
};