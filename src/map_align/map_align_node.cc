#include "map_align/map_align_node.h"

MapAlignNode::MapAlignNode(ros::NodeHandle node_handle): nh(node_handle){
    this->pcl_sub = this->nh.subscribe("/robot/os_cloud_node/points", 100, &MapAlignNode::scanCallback, this, ros::TransportHints().tcpNoDelay());

    float w, x, y, z;

    ros::param::param<float>("~dlio/state/quat/w", w, 0.);
    ros::param::param<float>("~dlio/state/quat/x", x, 0.);
    ros::param::param<float>("~dlio/state/quat/y", y, 0.);
    ros::param::param<float>("~dlio/state/quat/z", z, 0.);

    this->state.q = Eigen::Quaternionf(w, x, y, z);

    ros::param::param<float>("~dlio/state/pose/x", this->state.p[0], 0.);
    ros::param::param<float>("~dlio/state/pose/y", this->state.p[1], 0.);
    ros::param::param<float>("~dlio/state/pose/z", this->state.p[2], 0.);

    ros::param::param<std::string>("~dlio/map_link", this->map_link , "/dlio_map.pcd");

    this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("loaded_map", 100);
    this->aligned_scan_pub = this->nh.advertise<sensor_msgs::PointCloud2>("aligned_scan", 100);

    this->map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
    this->alignedScan = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

    this->publish_timer = this->nh.createTimer(ros::Duration(0.5), &MapAlignNode::publishMaps, this);


}

void MapAlignNode::publishMaps(const ros::TimerEvent& e){
    if(this->map != nullptr){
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(*this->map, map_ros);
        map_ros.header.stamp = ros::Time::now();
        map_ros.header.frame_id = "map";
        this->map_pub.publish(map_ros);
    }

    if(this->alignedScan != nullptr){
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(*this->alignedScan, map_ros);
        map_ros.header.stamp = ros::Time::now();
        map_ros.header.frame_id = "map";
        this->aligned_scan_pub.publish(map_ros);
    }
    
}

void MapAlignNode::scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan){

    this->scan_ = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

    pcl::fromROSMsg(*scan, *scan_);

    nano_gicp::NanoGICP<PointType, PointType> gicp;
    map_align::MapAlign mapAlign(gicp);

    pcl::PointCloud<PointType>::Ptr map_ = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

    if (pcl::io::loadPCDFile<PointType> (this->map_link, *map_) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file map_pcd.pcd \n");
    return;
    }

    map_align::MapAlign::State state = mapAlign.align(map_, this->scan_, this->state);

    std::cout << "Quat: " << state.q.w() << std::endl << state.q.vec() << std::endl;
    std::cout << "Pos: " << state.p << std::endl;

    this->map = map_;
    this->alignedScan = mapAlign.getAlignedScan();

    this->pcl_sub.shutdown();
}