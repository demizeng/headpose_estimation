#include "registration.h"

registration::registration()
{
    src_cloud_tmp.reset(new PointCloudT);
    tgt_cloud_tmp.reset(new PointCloudT);

    sacia.setMaximumIterations(500);
    //sacia.setMinSampleDistance(1);
    sacia.setNumberOfSamples(4);
    sacia.setRANSACOutlierRejectionThreshold(0.04);
    /*Instead of matching each object FPFH descriptor to its nearest matching feature in the scene,
    we can choose between the N best matches at random. This increases the iterations necessary,
    but also makes the algorithm robust towards outlier matches.*/
    sacia.setCorrespondenceRandomness(3);
    //sacia.setMaxCorrespondenceDistance (0.06);

    icp.setMaxCorrespondenceDistance (0.02);//0.04
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (100);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-10);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.01);//0.2

    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.001);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);//0.07,0.1
    //设置NDT网格结构的分辨率（VoxelGridCovariance）（体素格的大小）
    ndt.setResolution(0.03);//1.5,0.03
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(100);

    if(! s4pcs.options_.configureOverlap(0.8) )  {
        std::cout<<"warning:overlap is not suitable!"<<std::endl;
    }
    s4pcs.options_.sample_size = 150;//100
    //s4pcs.options_.max_normal_difference = 0.000001;
    s4pcs.options_.max_color_distance = -1; //-1 means don't use it
    s4pcs.options_.max_time_seconds = 100;
    s4pcs.options_.delta = 0.004;//0.005

}

registration::~registration()
{

}

double registration::do_sacia(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform)
{
    src_cloud_tmp->clear();
    pcl::copyPointCloud(*src_cloud,*src_cloud_tmp);
    tgt_cloud_tmp->clear();
    pcl::copyPointCloud(*tgt_cloud,*tgt_cloud_tmp);
    clock_t start=clock();

    //float leaf=0.01;
    voxel_filter.setLeafSize(0.02,0.02,0.02);
    voxel_filter.setInputCloud(src_cloud_tmp);
    voxel_filter.filter(*src_cloud_tmp);
    voxel_filter.setLeafSize(0.01,0.01,0.01);
    voxel_filter.setInputCloud(tgt_cloud_tmp);
    voxel_filter.filter(*tgt_cloud_tmp);
    std::cout<<"src_cloud: "<<src_cloud_tmp->size()<<" points;tgt_cloud: "<<tgt_cloud_tmp->size()<<" points."<<std::endl;

    pcl::NormalEstimation<PointT,pcl::Normal> nomal;
    pcl::search::KdTree<PointT>::Ptr tree_nomal(new pcl::search::KdTree<PointT>());
    //nomal.setNumberOfThreads(4);
    nomal.setRadiusSearch(0.02);
    nomal.setInputCloud(src_cloud_tmp);
    nomal.setSearchMethod(tree_nomal);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
    nomal.compute(*cloud_src_normals);
    nomal.setInputCloud(tgt_cloud_tmp);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
    nomal.compute(*cloud_tgt_normals);

    pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh_est;
    pcl::search::KdTree<PointT>::Ptr tree_fpfh (new pcl::search::KdTree<PointT>);
    //fpfh_est.setNumberOfThreads(4);
    fpfh_est.setRadiusSearch(0.05);
    fpfh_est.setSearchMethod(tree_fpfh);
    fpfh_est.setInputCloud(src_cloud_tmp);
    fpfh_est.setInputNormals(cloud_src_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_est.compute(*fpfhs_src);
    fpfh_est.setInputCloud(tgt_cloud_tmp);
    fpfh_est.setInputNormals(cloud_tgt_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_est.compute(*fpfhs_tgt);

    sacia.setInputSource(src_cloud_tmp);
    sacia.setInputTarget(tgt_cloud_tmp);
    sacia.setSourceFeatures(fpfhs_src);
    sacia.setTargetFeatures(fpfhs_tgt);
    sacia.align(*final_cloud);
    clock_t end=clock();
    final_transform=sacia.getFinalTransformation();
    pcl::transformPointCloud(*src_cloud, *final_cloud, final_transform);
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}

double registration::do_icp(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform)
{
    clock_t start=clock();   
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tgt_cloud);
    icp.align(*final_cloud);
    clock_t end=clock();
    final_transform=icp.getFinalTransformation();
    return (double)(end-start)/(double)CLOCKS_PER_SEC;

}

//ndt has not completed yet
double registration::do_ndt(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform)
{
    src_cloud_tmp->clear();
    pcl::copyPointCloud(*src_cloud,*src_cloud_tmp);
    tgt_cloud_tmp->clear();
    pcl::copyPointCloud(*tgt_cloud,*tgt_cloud_tmp);
    clock_t start=clock();
    voxel_filter.setLeafSize(0.01,0.01,0.01);
    voxel_filter.setInputCloud(src_cloud_tmp);
    voxel_filter.filter(*src_cloud_tmp);
    voxel_filter.setInputCloud(tgt_cloud_tmp);
    voxel_filter.filter(*tgt_cloud_tmp);
    std::cout<<"src_cloud: "<<src_cloud_tmp->size()<<" points;tgt_cloud: "<<tgt_cloud_tmp->size()<<" points."<<std::endl;

    // 设置要配准的点云
    //ndt.setInputSource(cloud_src_o);
    ndt.setInputSource(src_cloud_tmp);
    //设置点云配准目标
    ndt.setInputTarget(tgt_cloud_tmp);
    //配准
    ndt.align(*final_cloud);
    clock_t end=clock();
    final_transform=ndt.getFinalTransformation();
    pcl::transformPointCloud(*src_cloud, *final_cloud, final_transform);
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}

double registration::do_s4pcs(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform)
{
    src_cloud_tmp->clear();
    pcl::copyPointCloud(*src_cloud,*src_cloud_tmp);
    tgt_cloud_tmp->clear();
    pcl::copyPointCloud(*tgt_cloud,*tgt_cloud_tmp);
    clock_t start=clock();
    voxel_filter.setLeafSize(0.01,0.01,0.01);
    voxel_filter.setInputCloud(src_cloud_tmp);
    voxel_filter.filter(*src_cloud_tmp);
    voxel_filter.setInputCloud(tgt_cloud_tmp);
    voxel_filter.filter(*tgt_cloud_tmp);
    std::cout<<"src_cloud: "<<src_cloud_tmp->size()<<" points;tgt_cloud: "<<tgt_cloud_tmp->size()<<" points."<<std::endl;

    s4pcs.setInputSource (src_cloud_tmp);
    s4pcs.setInputTarget (tgt_cloud_tmp);
    s4pcs.align (*final_cloud);
    clock_t end=clock();
    final_transform=s4pcs.getFinalTransformation();
    pcl::transformPointCloud(*src_cloud, *final_cloud, final_transform);
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}
