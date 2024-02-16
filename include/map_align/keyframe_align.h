
#include <dlio/dlio.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class KeyframeAlign{
    public:
        KeyframeAlign(ros::NodeHandle node_handle);

    private:

        void scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan);
        void publishMaps(const ros::TimerEvent& e);

        ros::NodeHandle nh;
        ros::Subscriber keyframe_cloud_sub;
        // std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseArray>> keyframe_pose_sub;
        // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseArray>> sync;
        std::string file_location = "";

        ros::Timer publish_timer;

        Eigen::Quaternionf map_alignment_quat;

        Eigen::Vector3f map_alignment_pose;

        ros::Publisher base_pub;
        ros::Publisher new_pub;

        pcl::PointCloud<PointType>::Ptr base_world;
        pcl::PointCloud<PointType>::Ptr new_world;
    
        nano_gicp::NanoGICP<PointType, PointType> gicp;

        bool savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
               direct_lidar_inertial_odometry::save_pcd::Response& res);

        ros::ServiceServer save_pcd_srv;

        
};