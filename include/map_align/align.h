#include "map_align/map_align.h"

class MapAlignNode{
    public:
        MapAlignNode(ros::NodeHandle node_handle);

    private:

        void scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan);

        ros::NodeHandle nh;
        ros::Subscriber pcl_sub;
        map_align::MapAlign::State state;
        std::string map_link = "";
        pcl::PointCloud<PointType>::Ptr scan_;

        ros::Timer publish_timer;

        void publishMaps(const ros::TimerEvent& e);

        ros::Publisher map_pub;
        ros::Publisher aligned_scan_pub;

        pcl::PointCloud<PointType>::Ptr map;
        pcl::PointCloud<PointType>::Ptr alignedScan;
    
};