#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <time.h>


class preprocess
{
protected:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef PointCloudT::Ptr PointCloudTPtr;
    pcl::PassThrough<PointT> pass;
    pcl::StatisticalOutlierRemoval<PointT> Static;
    pcl::ConditionalRemoval<PointT> condition;

public:
    preprocess();
//    preprocess(PointCloudTPtr &preprocess_cloud);
    ~preprocess();
    double process(PointCloudTPtr &preprocess_cloud);

private:
    Eigen::Vector4f minPT,maxPT;

};

#endif // PREPROCESS_H
