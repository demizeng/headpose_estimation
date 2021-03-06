#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <Eigen/Core>
#include <pcl/registration/super4pcs.h>
#include <super4pcs/shared4pcs.h>
#include <super4pcs/utils/logger.h>
#include <time.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/file_io.h>

class registration
{
protected:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef PointCloudT::Ptr PointCloudTPtr;
    PointCloudTPtr src_cloud_tmp;//if needs voxel_grid,use src_cloud_tmp will avoid destroying src_cloud
    PointCloudTPtr tgt_cloud_tmp;
    pcl::VoxelGrid<PointT> voxel_filter;
    pcl::IterativeClosestPoint<PointT,PointT> icp;
//    pcl::SampleConsensusInitialAlignment<PointT,PointT, pcl::FPFHSignature33> sacia;
//    pcl::SampleConsensusPrerejective<PointT,PointT,pcl::FPFHSignature33> sacpre;
    pcl::NormalDistributionsTransform<PointT,PointT> ndt;
    pcl::Super4PCS<PointT,PointT> s4pcs;

public:
    registration();
    ~registration();
//    double do_sacia(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform,int sacmode);
    double do_sacpre(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform,int sacmode);
    double do_icp(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform,int icpmode);
    double do_ndt(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform);
    double do_s4pcs(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform);

};

#endif // REGISTRATION_H
