#include "map_align/map_align_node.h"

TestNode::TestNode(ros::NodeHandle node_handle): nh(node_handle){
            this->pcl_sub = this->nh.subscribe("/robot/os_cloud_node/points", 100, &TestNode::scanCallback, this, ros::TransportHints().tcpNoDelay());

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

        }

void TestNode::scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan){

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


        }