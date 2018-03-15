#include "headpose.h"
#include "../release/ui_headpose.h"
#include <cstdio>

headpose::headpose(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::headpose)
{
    ui->setupUi(this);
    dir.cd("../../");
    datapath="../../data/";
//    fileindex=0;
//    pcl::getAllPcdFilesInDirectory(datapath,filesname);
//    std::cout<<"file size: "<<filesname.size()<<std::endl;
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
    QString dataname=ui->edit_dataname->text();
    if(dataname.isEmpty())
    {
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("请先输入数据名称，例如：preson"),QMessageBox::Yes);
        return;
    }
    dir.cd("./data");
    if(!dir.exists(dataname))
    {
        dir.mkdir(dataname);
    }

//    std::string namestring=qstr2str(dataname);
//    std::cout<<namestring<<std::endl;
//    char collect_index[3];
//    sprintf(collect_index,"%03d",30);
//    std::string collect_pcd_name=datapath+namestring+"/"+namestring+"_"+std::string(collect_index)+".pcd";
//    std::cout<<collect_pcd_name<<std::endl;



}
