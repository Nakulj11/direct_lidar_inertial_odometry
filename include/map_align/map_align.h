#include <dlio/odom.h>

namespace map_align{


class MapAlign{
    public:
        MapAlign(nano_gicp::NanoGICP<PointType, PointType>& gicp_input);

        struct State {
            Eigen::Vector3f p; // position in world frame
            Eigen::Quaternionf q; // orientation in world frame
        };

        State align(pcl::PointCloud<PointType>::ConstPtr map, pcl::PointCloud<PointType>::ConstPtr instantaneousCloud, State& guess);

        pcl::PointCloud<PointType>::Ptr getAlignedScan();
        

    private:
        nano_gicp::NanoGICP<PointType, PointType>& gicp;

        Eigen::Matrix4f stateToMatrix(State state);
        MapAlign::State matrixToState(Eigen::Matrix4f mat);

        pcl::PointCloud<PointType>::Ptr aligned;

};
}