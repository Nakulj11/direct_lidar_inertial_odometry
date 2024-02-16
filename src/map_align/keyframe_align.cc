#include "map_align/keyframe_align.h"

KeyframeAlign::KeyframeAlign(ros::NodeHandle node_handle): nh(node_handle){


    base_pub =
      nh.advertise<sensor_msgs::PointCloud2>("base_cloud", 1);
    new_pub =
      nh.advertise<sensor_msgs::PointCloud2>("new_cloud", 1);
  


    float w, x, y, z;

    ros::param::param<float>("~dlio/state/quat/w", w, 0.);
    ros::param::param<float>("~dlio/state/quat/x", x, 0.);
    ros::param::param<float>("~dlio/state/quat/y", y, 0.);
    ros::param::param<float>("~dlio/state/quat/z", z, 0.);

    map_alignment_quat = Eigen::Quaternionf(w, x, y, z);


    ros::param::param<float>("~dlio/state/pose/x", map_alignment_pose[0], 0.);
    ros::param::param<float>("~dlio/state/pose/y", map_alignment_pose[1], 0.);
    ros::param::param<float>("~dlio/state/pose/z", map_alignment_pose[2], 0.);


    ros::param::param<std::string>("~dlio/map_link", file_location , " ");


    base_world = pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>());
    new_world = pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>());

    // Load the point clouds
    if (pcl::io::loadPCDFile<PointType>(file_location ,
                                            *base_world) == -1) {
        ROS_ERROR("Couldn't read base_world file");
    }

    this->publish_timer = this->nh.createTimer(ros::Duration(0.5), &KeyframeAlign::publishMaps, this);

    
    keyframe_cloud_sub = this->nh.subscribe("/robot/dlio/odom_node/pointcloud/keyframe", 100, &KeyframeAlign::scanCallback, this, ros::TransportHints().tcpNoDelay());


    this->save_pcd_srv = this->nh.advertiseService("save_pcd", &KeyframeAlign::savePcd, this);
}

void KeyframeAlign::scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan){
    pcl::PointCloud<PointType>::Ptr keyframe(boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*scan, *keyframe);

    Eigen::Matrix4f keyframe_guess = Eigen::Matrix4f::Identity();
    keyframe_guess.block<3, 3>(0, 0) = map_alignment_quat.toRotationMatrix();
    keyframe_guess.block<3, 1>(0, 3) = map_alignment_pose;

    this->gicp.setInputSource(keyframe);
    this->gicp.setInputTarget(base_world);

    pcl::PointCloud<PointType>::Ptr aligned(boost::make_shared<pcl::PointCloud<PointType>>(*keyframe));
    this->gicp.align(*aligned, keyframe_guess);

    (*new_world) += (*aligned);

}

void KeyframeAlign::publishMaps(const ros::TimerEvent& e){
    if(this->base_world != nullptr){
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(*this->base_world, map_ros);
        map_ros.header.stamp = ros::Time::now();
        map_ros.header.frame_id = "map";
        this->base_pub.publish(map_ros);
    }

    if(this->new_world != nullptr){
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(*this->new_world, map_ros);
        map_ros.header.stamp = ros::Time::now();
        map_ros.header.frame_id = "map";
        this->new_pub.publish(map_ros);
    }

}

bool KeyframeAlign::savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
                            direct_lidar_inertial_odometry::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->new_world));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}
 

int main(int argc, char **argv) {

  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "keyframe_alignment");
  ros::NodeHandle nh("~");


  KeyframeAlign test(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();


}