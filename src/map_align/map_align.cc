#include "map_align/map_align.h"

namespace map_align{

    MapAlign::MapAlign(nano_gicp::NanoGICP<PointType, PointType>& gicp_input):gicp(gicp_input){}

    MapAlign::State MapAlign::align(pcl::PointCloud<PointType>::ConstPtr map, pcl::PointCloud<PointType>::ConstPtr instantaneousCloud, MapAlign::State& guess){

        Eigen::Matrix4f guessTransform = stateToMatrix(guess);

        this->gicp.setInputSource(instantaneousCloud);
        this->gicp.setInputTarget(map);

        pcl::PointCloud<PointType>::Ptr aligned (boost::make_shared<pcl::PointCloud<PointType>>());
        this->gicp.align(*aligned, guessTransform);
        

        return matrixToState(this->gicp.getFinalTransformation() * Eigen::Matrix4f::Identity());

    }

    

    Eigen::Matrix4f MapAlign::stateToMatrix(map_align::MapAlign::State state){
        
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        transform.block(0,0,3,3) = state.q.toRotationMatrix();

        transform(0,3) = state.p[0];
        transform(1,3) = state.p[1];
        transform(2,3) = state.p[2];

        return transform;
    }

    MapAlign::State MapAlign::matrixToState(Eigen::Matrix4f mat){
        Eigen::Matrix3f rotMat = mat.block(0,0,3,3);
        Eigen::Quaternionf q(rotMat);

        Eigen::Vector3f p;
        p << mat(0,3), mat(1,3), mat(2,3);

        map_align::MapAlign::State state;
        state.q = q;
        state.p = p;

        return state;
    }


}