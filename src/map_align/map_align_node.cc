#include "map_align/map_align_node.h"

TestNode::TestNode(ros::NodeHandle node_handle): nh(node_handle){
            this->pcl_sub = this->nh.subscribe("/robot/os_cloud_node/points", 100, &TestNode::scanCallback, this, ros::TransportHints().tcpNoDelay());
        }

void TestNode::scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan){

            std::cout << "Callback Activated" << std::endl;

            this->scan_ = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

            pcl::fromROSMsg(*scan, *scan_);

            nano_gicp::NanoGICP<PointType, PointType> gicp;
            map_align::MapAlign mapAlign(gicp);


            map_align::MapAlign::State guess;

            guess.q = Eigen::Quaternionf(0., 0., 0., 1);

            guess.p[0] = 7.88;
            guess.p[1] = 6.72;
            guess.p[2] = 1.82;

            std::cout << "Reading File" << std::endl;

            pcl::PointCloud<PointType>::Ptr map_ = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

            if (pcl::io::loadPCDFile<PointType> ("/home/nakul/Desktop/vectr/ws/src/dlio_map.pcd", *map_) == -1) //* load the file
            {
            PCL_ERROR ("Couldn't read file map_pcd.pcd \n");
            return;
            }

            std::cout << "File Read" << std::endl;

            map_align::MapAlign::State state = mapAlign.align(map_, this->scan_, guess);

            std::cout << "Quat: " << state.q.w() << std::endl << state.q.vec() << std::endl;
            std::cout << "Pos: " << state.p << std::endl;


        }