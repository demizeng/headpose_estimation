#include "preprocess.h"

preprocess::preprocess()
{
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.7,1.3);
    Static.setMeanK(100);
    Static.setStddevMulThresh(1.5);

}
double preprocess::process(PointCloudTPtr &preprocess_cloud)
{
    clock_t start=clock();
    pass.setInputCloud(preprocess_cloud);
    pass.filter(*preprocess_cloud);
    Static.setInputCloud(preprocess_cloud);
    Static.filter(*preprocess_cloud);
    pcl::getMinMax3D(*preprocess_cloud,minPT,maxPT);
    std::cout<<"ymin: "<<minPT[1]<<" ymax: "<<maxPT[1]<<" xmin: "<<minPT[0]<<" xmax: "<<maxPT[0]<<std::endl;
    pcl::ConditionAnd<PointT>::Ptr and_cond(new pcl::ConditionAnd<PointT>());
    and_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y",pcl::ComparisonOps::GT,minPT[1])));
    and_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y",pcl::ComparisonOps::LT,minPT[1]+0.14)));//0.16 for person3
    and_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x",pcl::ComparisonOps::GT,(minPT[0]+maxPT[0])/2-0.12)));//0.18 for person3
    and_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x",pcl::ComparisonOps::LT,(minPT[0]+maxPT[0])/2+0.12)));//person3 has not
    condition.setCondition(and_cond);
    condition.setInputCloud(preprocess_cloud);
    condition.filter(*preprocess_cloud);
    clock_t end=clock();
    std::cout<<"after filtered: "<<preprocess_cloud->size()<<" points."<<std::endl;
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}

preprocess::~preprocess()
{

}
