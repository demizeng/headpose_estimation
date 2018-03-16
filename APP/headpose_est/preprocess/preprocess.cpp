#include "preprocess.h"

preprocess::preprocess()
{

    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5,1.1);//0.5,1.1
//  pass.setFilterLimits(0.5,1.0);//for data_zjw series
//  pass.setFilterLimits(0.5,1.1);//for person1 2 3
    Static.setMeanK(200);//400(for data_zjw series)
    Static.setStddevMulThresh(0.8);//数值越小留下的点越少,0.7

}
//preprocess::preprocess(PointCloudTPtr &preprocess_cloud)
//{

//}
void preprocess::process(PointCloudTPtr &preprocess_cloud)
{
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
    std::cout<<"after filtered: "<<preprocess_cloud->size()<<" points."<<std::endl;

}

preprocess::~preprocess()
{

}
