#include "registration.h"

registration::registration()
{
    src_cloud_tmp.reset(new PointCloudT);
    tgt_cloud_tmp.reset(new PointCloudT);

    icp.setMaxCorrespondenceDistance (0.1);//0.04
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (1000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-10);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.00001);//0.2

    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.0000001);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.07);//0.07,0.1
    //设置NDT网格结构的分辨率（VoxelGridCovariance）（体素格的大小）
    ndt.setResolution(0.018);//1.5,0.03
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(1000);

    if(! s4pcs.options_.configureOverlap(0.8) )  {
        std::cout<<"warning:overlap is not suitable!"<<std::endl;
    }
    s4pcs.options_.sample_size = 200;//100
    //s4pcs.options_.max_normal_difference = 0.000001;
    s4pcs.options_.max_color_distance = -1; //-1 means don't use it
    s4pcs.options_.max_time_seconds = 100;
    s4pcs.options_.delta = 0.004;//0.005

}

registration::~registration()
{

}

double registration::do_sacpre(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform,int sacmode)
{
    clock_t start=clock();

    std::vector<int> indices; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*src_cloud,*src_cloud, indices);
    pcl::removeNaNFromPointCloud(*tgt_cloud,*tgt_cloud, indices);
    pcl::NormalEstimation<PointT,pcl::Normal> nomal;
    pcl::search::KdTree<PointT>::Ptr tree_nomal(new pcl::search::KdTree<PointT>());
    //nomal.setNumberOfThreads(4);
    if(sacmode==2)
        nomal.setRadiusSearch(0.008);
    else
        nomal.setRadiusSearch(0.01);
    nomal.setInputCloud(src_cloud);
    nomal.setSearchMethod(tree_nomal);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
    nomal.compute(*cloud_src_normals);
    nomal.setInputCloud(tgt_cloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
    nomal.compute(*cloud_tgt_normals);

    pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh_est;
    pcl::search::KdTree<PointT>::Ptr tree_fpfh (new pcl::search::KdTree<PointT>);
    //fpfh_est.setNumberOfThreads(4);
    if(sacmode==2)
        fpfh_est.setRadiusSearch(0.032);
    else
        fpfh_est.setRadiusSearch(0.02);
    fpfh_est.setSearchMethod(tree_fpfh);
    fpfh_est.setInputCloud(src_cloud);
    fpfh_est.setInputNormals(cloud_src_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_est.compute(*fpfhs_src);
    fpfh_est.setInputCloud(tgt_cloud);
    fpfh_est.setInputNormals(cloud_tgt_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_est.compute(*fpfhs_tgt);

    pcl::SampleConsensusPrerejective<PointT,PointT,pcl::FPFHSignature33> sacpre;
    sacpre.setMaximumIterations(4000);
    if(sacmode==2)
    {
        sacpre.setNumberOfSamples (4);
        sacpre.setCorrespondenceRandomness (2);
    }
    else
    {
        sacpre.setNumberOfSamples(3);
        // Number of points to sample for generating/prerejecting a pose
        sacpre.setCorrespondenceRandomness(5);
        // Number of nearest features to use
    }
    sacpre.setSimilarityThreshold (0.8);
    /*The alignment class uses the CorrespondenceRejectorPoly class for early elimination of bad poses based on pose-invariant
     * geometric consistencies of the inter-distances between sampled points on the object and the scene. The closer this value
     * is set to 1, the more greedy and thereby fast the algorithm becomes. However, this also increases the risk of eliminating
     * good poses when noise is present.
     * */
    sacpre.setMaxCorrespondenceDistance (3.0f*0.001);
//    std::cout<<&sacpre<<std::endl;
    sacpre.setInlierFraction (0.3f);
    // Required inlier fraction for accepting a pose hypothesis
    sacpre.setInputSource(src_cloud);
    sacpre.setInputTarget(tgt_cloud);
    sacpre.setSourceFeatures(fpfhs_src);
    sacpre.setTargetFeatures(fpfhs_tgt);
    sacpre.align(*final_cloud);
    clock_t end=clock();
    final_transform=sacpre.getFinalTransformation();
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}

double registration::do_icp(PointCloudTPtr &src_cloud,PointCloudTPtr &tgt_cloud,PointCloudTPtr &final_cloud,Eigen::Matrix4f &final_transform,int icpmode)
{
    clock_t start=clock();
    if(icpmode==2)
        icp.setMaxCorrespondenceDistance (0.003);
    else icp.setMaxCorrespondenceDistance (0.1);
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tgt_cloud);
    icp.align(*final_cloud);
    clock_t end=clock();
    final_transform=icp.getFinalTransformation();
    return (double)(end-start)/(double)CLOCKS_PER_SEC;

}

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
    std::cout<<"src_cloud: "<<src_cloud_tmp->size()<<" points"<<std::endl;

    // 设置要配准的点云
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
    clock_t start=clock();
    s4pcs.setInputSource (src_cloud);
    s4pcs.setInputTarget (tgt_cloud);
    s4pcs.align (*final_cloud);
    clock_t end=clock();
    final_transform=s4pcs.getFinalTransformation();
    return (double)(end-start)/(double)CLOCKS_PER_SEC;
}
