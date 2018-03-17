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
    collect_cloud.reset(new PointCloudT);
    preprocess_cloud.reset(new PointCloudT);
    src_cloud.reset(new PointCloudT);
    tgt_cloud.reset(new PointCloudT);
    final_cloud.reset(new PointCloudT);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,0,0,0,-1,0);
    viewer->addText("headpose estimation",10,10,1,0,0,0,"notice");
    viewer->addText("pitch              yaw               roll",10,20,1,0,0,0,"angle title");
    viewer->addText("ANGLE_result",10,30,1,0,0,0,"angle");
    ui->qvtkWidget->update();
}

headpose::~headpose()
{
    delete ui;
}


//void headpose::on_pushButton_clicked()
//{
//    std::cout<<"load pcd file: "<<filesname[fileindex]<<std::endl;
//    pcl::io::loadPCDFile(datapath+filesname[fileindex],*collect_cloud);
//    viewer->removeAllPointClouds();
//    viewer->addPointCloud<PointT>(collect_cloud,"collect_cloud");
//    ui->qvtkWidget->update();
//    fileindex++;
//}

//data_collect has not completed
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
        dir.cd("./"+dataname);
        dir.mkdir("filtered");
        dir.cd("../");
    }
    dir.cd("../");

//    std::string namestring=qstr2str(dataname);
//    std::cout<<namestring<<std::endl;
//    char collect_index[3];
//    sprintf(collect_index,"%03d",30);
//    std::string collect_pcd_name=datapath+namestring+"/"+namestring+"_"+std::string(collect_index)+".pcd";
//    std::cout<<collect_pcd_name<<std::endl;



}

void headpose::on_button_choosePCD_clicked()
{
    QFileDialog *choose_dialog=new QFileDialog(this);
    choose_dialog->setWindowTitle(QString::fromUtf8("选择点云数据"));
    choose_dialog->setDirectory("../../data/");
    choose_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    choose_dialog->setViewMode(QFileDialog::Detail);
    choose_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(choose_dialog->exec())
    {
        choose_pcd_name=choose_dialog->selectedFiles();
    }
    for(int i=0;i<choose_pcd_name.size();i++)
    {
        ui->label_choosePCD->setText(choose_pcd_name[i].section("/",-2));
        pcl::io::loadPCDFile(qstr2str(choose_pcd_name[i]),*preprocess_cloud);
        viewer->removeAllPointClouds();
        viewer->updateText("headpose estimation",10,10,1,0,0,0,"notice");
        viewer->updateText("pitch              yaw               roll",10,20,1,0,0,0,"angle title");
        viewer->updateText("ANGLE_result",10,30,1,0,0,0,"angle");
        viewer->addPointCloud<PointT>(preprocess_cloud,"preprocess_cloud");
        ui->qvtkWidget->update();
    }
}

void headpose::on_button_preprocess_clicked()
{
    if(choose_pcd_name.empty())
    {
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("请先选择待处理点云数据！"),QMessageBox::Yes);
        return;
    }
    preprocess preprocessPCD;
    QString filtered_name=choose_pcd_name[0].section("/",-1);
    std::string filtered_folder=datapath+qstr2str(filtered_name.section("_",0,0))+"/";
    for(int i=0;i<choose_pcd_name.size();i++)
    {
         pcl::io::loadPCDFile(qstr2str(choose_pcd_name[i]),*preprocess_cloud);
         preprocessPCD.process(preprocess_cloud);
         filtered_name=choose_pcd_name[i].section("/",-1);
         filtered_name.insert(filtered_name.length()-7,"filtered_");
         pcl::io::savePCDFile(filtered_folder+"filtered/"+qstr2str(filtered_name),*preprocess_cloud);
//         std::cout<<filtered_folder+"filtered/"+qstr2str(filtered_name)<<std::endl;
         viewer->removeAllPointClouds();
         viewer->updateText("headpose estimation",10,10,1,0,0,0,"notice");
         viewer->updateText("pitch              yaw               roll",10,20,1,0,0,0,"angle title");
         viewer->updateText("ANGLE_result",10,30,1,0,0,0,"angle");
         viewer->addPointCloud<PointT>(preprocess_cloud,"preprocess_cloud");
         ui->qvtkWidget->update();
    }

}

void headpose::on_button_src_clicked()
{
    QFileDialog *src_dialog=new QFileDialog(this);
    src_dialog->setWindowTitle(QString::fromUtf8("选择源点云"));
    src_dialog->setDirectory("../../data/");
    src_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    src_dialog->setViewMode(QFileDialog::Detail);
    src_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(src_dialog->exec())
    {
        src_name=src_dialog->selectedFiles();
    }
    ui->label_src->setText(src_name[0].section("/",-2));
    pcl::io::loadPCDFile(qstr2str(src_name[0]),*src_cloud);
    viewer->removeAllPointClouds();
    viewer->updateText("headpose estimation",10,10,1,0,0,0,"notice");
    viewer->updateText("pitch              yaw               roll",10,20,1,0,0,0,"angle title");
    viewer->updateText("ANGLE_result",10,30,1,0,0,0,"angle");
    viewer->addPointCloud<PointT>(src_cloud,"src_cloud");
    ui->qvtkWidget->update();

}

void headpose::on_button_tgt_clicked()
{
    QFileDialog *tgt_dialog=new QFileDialog(this);
    tgt_dialog->setWindowTitle(QString::fromUtf8("选择目标点云"));
    tgt_dialog->setDirectory("../../data/");
    tgt_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    tgt_dialog->setViewMode(QFileDialog::Detail);
    tgt_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(tgt_dialog->exec())
    {
        tgt_name=tgt_dialog->selectedFiles();
    }
    ui->label_tgt->setText(tgt_name[0].section("/",-2));
    pcl::io::loadPCDFile(qstr2str(tgt_name[0]),*tgt_cloud);
    viewer->removeAllPointClouds();
    viewer->updateText("headpose estimation",10,10,1,0,0,0,"notice");
    viewer->updateText("pitch              yaw               roll",10,20,1,0,0,0,"angle title");
    viewer->updateText("ANGLE_result",10,30,1,0,0,0,"angle");
    viewer->addPointCloud<PointT>(tgt_cloud,"tgt_cloud");
    ui->qvtkWidget->update();

}

void headpose::on_button_registration_clicked()
{
    double reg_time=0.0;
    int method_index=ui->comboBox->currentIndex();
//    std::cout<<method_index<<" "<<qstr2str(ui->comboBox->currentText())<<std::endl;
    switch (method_index)
    {
    case 0:
        reg_time=myreg.do_sacia(src_cloud,tgt_cloud,final_cloud,final_transformation);
        break;
    case 1:
        reg_time=myreg.do_icp(src_cloud,tgt_cloud,final_cloud,final_transformation);
        break;
    case 2:
        reg_time=myreg.do_ndt(src_cloud,tgt_cloud,final_cloud,final_transformation);
        break;
    case 3:
        reg_time=myreg.do_s4pcs(src_cloud,tgt_cloud,final_cloud,final_transformation);
        break;
    default:
    {
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("请先选择配准算法！"),QMessageBox::Yes);
        return;
    }
        break;
    }
    if(!(reg_time==0.0))
    {
        std::cout<<qstr2str(ui->comboBox->currentText())<<" takes "<<reg_time<<" seconds."<<std::endl;
        matrix2angle(final_transformation,ANGLE_result);
        std::string angles=std::to_string(ANGLE_result(0))+"  "+std::to_string(ANGLE_result(1))+"  "+std::to_string(ANGLE_result(2));
        viewer->removeAllPointClouds();
        viewer->addPointCloud<PointT>(tgt_cloud,ColorHandlerT(tgt_cloud,255,0,0),"tgt_cloud");
        viewer->addPointCloud<PointT>(src_cloud,ColorHandlerT(src_cloud,0,255,0),"src_cloud");
        viewer->addPointCloud<PointT>(final_cloud,ColorHandlerT(final_cloud,0,0,255),"final_cloud");
        viewer->updateText("red:source pointcloud ; green:target pointcloud ; blue:pointcloud after registration", 20,100,20,1,1,1,"notice");
        viewer->updateText("pitch              yaw               roll",110,280,25,1,1,1,"angle title");
        viewer->updateText(angles,110,260,25,1,1,1,"angle");
        ui->qvtkWidget->update();
    }
}
