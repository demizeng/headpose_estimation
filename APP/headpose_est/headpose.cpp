#include "headpose.h"
#include "../release/ui_headpose.h"

headpose::headpose(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::headpose)
{
    ui->setupUi(this);
    datapath="../../data/";
    fileindex=0;
    pcl::getAllPcdFilesInDirectory(datapath,filesname);
    std::cout<<"file size: "<<filesname.size()<<std::endl;
    cloud.reset(new PointCloudT);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,0,0,0,-1,0);
    ui->qvtkWidget->update();
}

headpose::~headpose()
{
    delete ui;
}


//void headpose::on_pushButton_clicked()
//{
//    std::cout<<"load pcd file: "<<filesname[fileindex]<<std::endl;
//    pcl::io::loadPCDFile(datapath+filesname[fileindex],*cloud);
//    viewer->removeAllPointClouds();
//    viewer->addPointCloud<PointT>(cloud,"cloud");
//    ui->qvtkWidget->update();
//    fileindex++;
//}

void headpose::on_button_collect_clicked()
{


}
