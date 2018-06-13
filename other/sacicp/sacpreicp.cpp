#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <time.h>
#include <pcl/common/time.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <stdlib.h>
#include <fstream>

//注释里面的那一套参数对较小的角度的适应性较好，现在这一套参数对较大的角度的适应性较好，还需要调整，可以考虑x-27,x+29,y-30,z-20这几个特殊角度用大参数，其他的用小参数
//using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> colorhandlert;

//由旋转平移矩阵计算姿态角
void matrix2angle (Eigen::Matrix4f &result_trans,Eigen::Vector3f &result_angle)
{
  double ax,ay,az;
  if (result_trans(2,0)==1 || result_trans(2,0)==-1)
  {
      az=0;
      double dlta;
      dlta=atan2(result_trans(0,1),result_trans(0,2));
      if (result_trans(2,0)==-1)
      {
          ay=M_PI/2;
          ax=az+dlta;
      }
      else
      {
          ay=-M_PI/2;
          ax=-az+dlta;
      }
  }
  else
  {
      ay=-asin(result_trans(2,0));
      ax=atan2(result_trans(2,1)/cos(ay),result_trans(2,2)/cos(ay));
      az=atan2(result_trans(1,0)/cos(ay),result_trans(0,0)/cos(ay));
  }
  ax=ax*180/M_PI;
  ay=ay*180/M_PI;
  az=az*180/M_PI;
  result_angle<<ax,ay,az;
}

int
   main (int argc, char** argv)
{
    if(argc<4)
    {
        std::cout<<"not enough arguments!"<<std::endl;
        return 0;
    }
   //加载点云文件(原点云，待配准)
   PointCloud::Ptr cloud_src (new PointCloud);//将长的转成短的//读取点云数据
   pcl::io::loadPCDFile (argv[1],*cloud_src);
   PointCloud::Ptr cloud_tgt (new PointCloud);
   pcl::io::loadPCDFile (argv[2],*cloud_tgt);

   std::vector<int> indices; //保存去除的点的索引
   pcl::removeNaNFromPointCloud(*cloud_src,*cloud_src, indices);
   pcl::removeNaNFromPointCloud(*cloud_tgt,*cloud_tgt, indices);
   float normalradius,fpfhradius,sacnos,saccr;
   int special_num=atoi(argv[3]);
//   std::cout<<special_num<<std::endl;
   if(special_num==2)
   {
       normalradius=0.008;
       fpfhradius=0.032;
       sacnos=4;
       saccr=2;
   }
   else
   {
       normalradius=0.01;
       fpfhradius=0.02;
       sacnos=3;
       saccr=5;
   }
   clock_t start=clock();
   //计算表面法线
   pcl::NormalEstimation<PointT,pcl::Normal> nomal;
   pcl::search::KdTree< PointT>::Ptr tree_nomal(new pcl::search::KdTree<PointT>());
   nomal.setRadiusSearch(normalradius);
   nomal.setInputCloud(cloud_src);
   nomal.setSearchMethod(tree_nomal);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
   nomal.compute(*cloud_src_normals);
   nomal.setInputCloud(cloud_tgt);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
   nomal.compute(*cloud_tgt_normals);

   //计算FPFH
   pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh_est;
   pcl::search::KdTree<PointT>::Ptr tree_fpfh (new pcl::search::KdTree<PointT>);
   fpfh_est.setRadiusSearch(fpfhradius);
   fpfh_est.setSearchMethod(tree_fpfh);
   fpfh_est.setInputCloud(cloud_src);
   fpfh_est.setInputNormals(cloud_src_normals);
   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
   fpfh_est.compute(*fpfhs_src);
   fpfh_est.setInputCloud(cloud_tgt);
   fpfh_est.setInputNormals(cloud_tgt_normals);
   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
   fpfh_est.compute(*fpfhs_tgt);

   //SampleConsensusPrerejective姿态估计
   pcl::SampleConsensusPrerejective<PointT,PointT,pcl::FPFHSignature33> align;
   align.setInputSource (cloud_src);
   align.setSourceFeatures (fpfhs_src);
   align.setInputTarget (cloud_tgt);
   align.setTargetFeatures (fpfhs_tgt);
   align.setMaximumIterations (4000); // Number of RANSAC iterations
   align.setNumberOfSamples (sacnos); // Number of points to sample for generating/prerejecting a pose,3,4(big)
   align.setCorrespondenceRandomness (saccr); // Number of nearest features to use,5,2(big)
   align.setSimilarityThreshold (0.8); // Polygonal edge length similarity threshold 数值越小时间越长（接近线性），效果非线性,0.8
   align.setMaxCorrespondenceDistance (3.0f*0.001); // Inlier threshold 会影响inliers参数， 值越大inliers越大,3*0.001
   align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis,当配准的总点数/待配准点云总点数超过了inliers fraction比例，则认为配准有效。0.3
   PointCloud::Ptr sac_result (new PointCloud);
   align.align (*sac_result);
   Eigen::Matrix4f sac_trans;
   sac_trans=align.getFinalTransformation();
//   std::cout<<sac_trans<<endl;

   pcl::IterativeClosestPoint<PointT, PointT> icp;
   icp.setInputSource(sac_result);
//   icp.setInputSource(cloud_src);
   icp.setInputTarget(cloud_tgt);
   icp.setMaxCorrespondenceDistance (0.003);//0.04
   // Set the maximum number of iterations (criterion 1)
   icp.setMaximumIterations (1000);
   // Set the transformation epsilon (criterion 2)
   icp.setTransformationEpsilon (1e-10);
   // Set the euclidean distance difference epsilon (criterion 3),点对之间的平均欧拉距离误差,就是d(k)-d(k-1)< delta 那个条件
   icp.setEuclideanFitnessEpsilon (0.00001);//0.2
   PointCloud::Ptr cloud_final (new PointCloud);
   icp.align(*cloud_final);
   clock_t end=clock();
   Eigen::Matrix4f icp_trans;
   icp_trans=icp.getFinalTransformation();
   icp_trans=sac_trans*icp_trans;
//   std::cout<<icp_trans<<endl;
   //时间
   double total_time=(double)(end-start)/(double)CLOCKS_PER_SEC;
//   cout<<"total time: "<<total_time<<" s"<<endl;
   ofstream outfile;
   outfile.open("sacicp.txt");
   if(!outfile.is_open())
       std::cout<<"open file failure!"<<std::endl;
   else
   {
       for (int m=0;m<4;m++)
           for(int n=0;n<4;n++)
               outfile<<icp_trans(m,n)<<endl;
       outfile<<total_time<<endl;

   }
   outfile.close();
//   //计算角度和位移
//   Eigen::Vector3f ANGLE_result;
//   matrix2angle(icp_trans,ANGLE_result);
//   cout<<" angle in x y z:\n"<<ANGLE_result(0)<<","<<ANGLE_result(1)<<","<<ANGLE_result(2)<<endl;
//   cout<<" offset in x y z:\n"<<sac_trans(0,3)<<","<<sac_trans(1,3)<<","<<sac_trans(2,3)<<endl;
   return (0);
}
