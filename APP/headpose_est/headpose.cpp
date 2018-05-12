#include "headpose.h"
#include "../release/ui_headpose.h"
#include <cstdio>

headpose::headpose(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::headpose)
{
    ui->setupUi(this);
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    dir.cd("../../");
    datapath="../../data/";
    sacmode=1;
    icpmode=1;
    src_number=0;
    tgt_number=0;
    view_switch=0;
    collect_cloud.reset(new PointCloudT);
    preprocess_cloud.reset(new PointCloudT);
    src_cloud.reset(new PointCloudT);
    tgt_cloud.reset(new PointCloudT);
    final_cloud.reset(new PointCloudT);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    viewer->setCameraFieldOfView (1.02259994f);
    viewer->setPosition (0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,0,0,0,-1,0);
//    viewer->addCoordinateSystem(0.1);
    ui->qvtkWidget->update();
}

headpose::~headpose()
{
    delete ui;
}

void headpose::cloud_callback(const PointCloudT::ConstPtr &cloud)
{
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_=cloud;
}

void headpose::on_button_collect_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
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
    std::string namestring=qstr2str(dataname);
    char collect_index[3];
    std::string collect_pcd_name;
    int pcd_collected=0;
    try
    {
        pcl::io::OpenNI2Grabber grabber;
        boost::function<void (const PointCloudT::ConstPtr&) > cloud_cb = boost::bind (&headpose::cloud_callback,this, _1);
        boost::signals2::connection cloud_connection = grabber.registerCallback (cloud_cb);
        grabber.start ();
        while (pcd_collected<100)//100
        {
            QCoreApplication::processEvents();
            PointCloudT::ConstPtr cloud;
            if (cloud_mutex_.try_lock ())
            {
                cloud_.swap (cloud);
                cloud_mutex_.unlock ();
            }
            if(cloud)
            {
                sprintf(collect_index,"%03d",pcd_collected++);
                collect_pcd_name=datapath+namestring+"/"+namestring+"_"+std::string(collect_index)+".pcd";
                PointCloudT cloud_pp;
                cloud_pp=*cloud;//pcl::PointCloud::ConstPtr ----> pcl::PointCloud
                pcl::io::savePCDFileASCII(collect_pcd_name,cloud_pp);
                if (!viewer->updatePointCloud (cloud, "OpenNICloud"))
                {
                   viewer->addPointCloud (cloud, "OpenNICloud");
                   viewer->resetCameraViewpoint ("OpenNICloud");
                   viewer->setCameraPosition (
                     0,0,0,		// Position
                     0,0,1,		// Viewpoint
                     0,-1,0);	// Up
                }
                ui->qvtkWidget->update();
            }
        }
        grabber.stop ();
        cloud_connection.disconnect ();
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("采集完成！数据位于")+str2qstr("data/"+namestring),QMessageBox::Yes);
     }
     catch (pcl::IOException& e)
     {
        pcl::console::print_error ("Failed to create a grabber: %s\n", e.what ());
        return;
     }
}

void headpose::on_button_choosePCD_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
    QFileDialog *choose_dialog=new QFileDialog(this);
    choose_dialog->setWindowTitle(QString::fromUtf8("选择点云数据"));
    choose_dialog->setDirectory("../../data/");
    choose_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    choose_dialog->setViewMode(QFileDialog::Detail);
    choose_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(choose_dialog->exec()==QFileDialog::Accepted)
    {
        choose_pcd_name=choose_dialog->selectedFiles();
    }
    else return;
    for(int i=0;i<choose_pcd_name.size();i++)
    {
        ui->label_choosePCD->setText(choose_pcd_name[i].section("/",-2));
        pcl::io::loadPCDFile(qstr2str(choose_pcd_name[i]),*preprocess_cloud);
        viewer->removeAllPointClouds();
        viewer->addPointCloud<PointT>(preprocess_cloud,"preprocess_cloud");
        ui->qvtkWidget->update();
    }
}

void headpose::on_button_preprocess_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    if(choose_pcd_name.empty())
    {
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("请先选择待处理点云数据！"),QMessageBox::Yes);
        return;
    }
    double preprocess_time=0.0;
    preprocess preprocessPCD;
    QString filtered_name=choose_pcd_name[0].section("/",-1);
    std::string filtered_folder=datapath+qstr2str(filtered_name.section("_",0,0))+"/";
    for(int i=0;i<choose_pcd_name.size();i++)
    {
         QCoreApplication::processEvents();
         pcl::io::loadPCDFile(qstr2str(choose_pcd_name[i]),*preprocess_cloud);
         preprocess_time=preprocessPCD.process(preprocess_cloud);
         filtered_name=choose_pcd_name[i].section("/",-1);
         filtered_name.insert(filtered_name.length()-7,"filtered_");
         pcl::io::savePCDFile(filtered_folder+"filtered/"+qstr2str(filtered_name),*preprocess_cloud);
//         std::cout<<filtered_folder+"filtered/"+qstr2str(filtered_name)<<std::endl;
         ui->label_time->setText(QString::number(preprocess_time,10,6));
         viewer->removeAllPointClouds();
         viewer->addPointCloud<PointT>(preprocess_cloud,"preprocess_cloud");
         ui->qvtkWidget->update();
    }
}

void headpose::on_button_src_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
    QFileDialog *src_dialog=new QFileDialog(this);
    src_dialog->setWindowTitle(QString::fromUtf8("选择源点云"));
    src_dialog->setDirectory("../../data/");
    src_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    src_dialog->setViewMode(QFileDialog::Detail);
    src_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(src_dialog->exec()==QFileDialog::Accepted)
    {
        src_name=src_dialog->selectedFiles();
    }
    else return;
    ui->label_src->setText(src_name[0].section("/",-2));
    QString src_name_f=src_name[0].section("/",-1);
    int src_number_index=src_name_f.lastIndexOf("_");
    src_name_f=src_name_f.mid(src_number_index+1,3);
    bool ok;
    src_number=src_name_f.toInt(&ok,10);
    pcl::io::loadPCDFile(qstr2str(src_name[0]),*src_cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<PointT>(src_cloud,"src_cloud");
    ui->qvtkWidget->update();

}

void headpose::on_button_tgt_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
    QFileDialog *tgt_dialog=new QFileDialog(this);
    tgt_dialog->setWindowTitle(QString::fromUtf8("选择目标点云"));
    tgt_dialog->setDirectory("../../data/");
    tgt_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    tgt_dialog->setViewMode(QFileDialog::Detail);
    tgt_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(tgt_dialog->exec()==QFileDialog::Accepted)
    {
        tgt_name=tgt_dialog->selectedFiles();
    }
    else return;
    ui->label_tgt->setText(tgt_name[0].section("/",-2));
    QString tgt_name_f=tgt_name[0].section("/",-1);
    int tgt_number_index=tgt_name_f.lastIndexOf("_");
    tgt_name_f=tgt_name_f.mid(tgt_number_index+1,3);
    bool ok;
    tgt_number=tgt_name_f.toInt(&ok,10);
    pcl::io::loadPCDFile(qstr2str(tgt_name[0]),*tgt_cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<PointT>(tgt_cloud,"tgt_cloud");
    ui->qvtkWidget->update();

}

void headpose::on_button_registration_clicked()
{
    double reg_time=0.0;
    icpmode=1;
    if(tgt_number==3 || tgt_number==6 || tgt_number==9 || tgt_number==14)
        sacmode=2;
    else sacmode=1;
    int method_index=ui->comboBox->currentIndex();
//    std::cout<<method_index<<" "<<qstr2str(ui->comboBox->currentText())<<std::endl;
    switch (method_index)
    {
    case 0:
        reg_time=myreg.do_sacpre(src_cloud,tgt_cloud,final_cloud,final_transformation,sacmode);
        break;
    case 1:
        reg_time=myreg.do_icp(src_cloud,tgt_cloud,final_cloud,final_transformation,icpmode);
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
        Eigen::Vector3f real_angle,est_angle;
        float error;
        real_angle(0)=angle_table[tgt_number][0]-angle_table[src_number][0];
        real_angle(1)=angle_table[tgt_number][1]-angle_table[src_number][1];
        real_angle(2)=angle_table[tgt_number][2]-angle_table[src_number][2];
        est_angle(0)=ANGLE_result(0)-angle_table[src_number][0];
        est_angle(1)=ANGLE_result(1)-angle_table[src_number][1];
        est_angle(2)=ANGLE_result(2)-angle_table[src_number][2];
        error=computeerror(real_angle,est_angle);
        std::string realangles=std::to_string(real_angle(0))+" , "+std::to_string(real_angle(1))+" , "+std::to_string(real_angle(2));
        std::string estangles=std::to_string(est_angle(0))+" , "+std::to_string(est_angle(1))+" , "+std::to_string(est_angle(2));
        std::string errors=std::to_string(error);
//        ui->label_red->setVisible(true);
//        ui->label_green->setVisible(true);
//        ui->label_blue->setVisible(true);
        ui->label_time->setText(QString::number(reg_time,10,6));
        viewer->removeAllPointClouds();
//        viewer->addPointCloud<PointT>(tgt_cloud,ColorHandlerT(tgt_cloud,255,0,0),"tgt_cloud");
//        viewer->addPointCloud<PointT>(src_cloud,ColorHandlerT(src_cloud,0,255,0),"src_cloud");
//        viewer->addPointCloud<PointT>(final_cloud,ColorHandlerT(final_cloud,0,0,255),"final_cloud");
        viewer->addPointCloud<PointT>(tgt_cloud,"tgt_cloud");
        viewer->addPointCloud<PointT>(final_cloud,"final_cloud");
        ui->qvtkWidget->update();
        ui->edit_realangle->setText(str2qstr(realangles));
        ui->edit_estangle->setText(str2qstr(estangles));
        ui->edit_error->setText(str2qstr(errors));
    }
}

//void headpose::on_pushButton_clicked()
//{
//    QFileDialog *datas_dialog=new QFileDialog(this);
//    datas_dialog->setWindowTitle(QString::fromUtf8("选择点云数据集"));
//    datas_dialog->setDirectory("../../data/");
//    datas_dialog->setFileMode(QFileDialog::Directory);
//    datas_dialog->setViewMode(QFileDialog::Detail);
//    if(datas_dialog->exec()==QFileDialog::Accepted)
//    {
//        datas_name=datas_dialog->selectedFiles();
//    }
//    else return;
//    ui->label_datasname->setText(datas_name[0].section("/",-3));
//}

void headpose::on_baseposeButton_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
    QFileDialog *base_dialog=new QFileDialog(this);
    base_dialog->setWindowTitle(QString::fromUtf8("选择基准姿态"));
    base_dialog->setDirectory("../../data/");
    base_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    base_dialog->setViewMode(QFileDialog::Detail);
    base_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(base_dialog->exec()==QFileDialog::Accepted)
    {
        src_name=base_dialog->selectedFiles();
    }
    else return;
    QString src_name_f=src_name[0].section("/",-1);
    ui->label_bposename->setText(src_name_f);
    int src_number_index=src_name_f.lastIndexOf("_");
    src_name_f=src_name_f.mid(src_number_index+1,3);
    bool ok;
    src_number=src_name_f.toInt(&ok,10);
    pcl::io::loadPCDFile(qstr2str(src_name[0]),*src_cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<PointT>(src_cloud,"bpose_cloud");
    ui->qvtkWidget->update();
}

void headpose::on_objposeButton_clicked()
{
    ui->label_red->setVisible(false);
    ui->label_green->setVisible(false);
    ui->label_blue->setVisible(false);
    ui->label_time->setText("");
    QFileDialog *opose_dialog=new QFileDialog(this);
    opose_dialog->setWindowTitle(QString::fromUtf8("选择目标姿态"));
    opose_dialog->setDirectory("../../data/");
    opose_dialog->setNameFilter(QString::fromUtf8("PCD files(*.pcd)"));
    opose_dialog->setViewMode(QFileDialog::Detail);
    opose_dialog->setFileMode(QFileDialog::ExistingFiles);
    if(opose_dialog->exec()==QFileDialog::Accepted)
    {
        tgt_name=opose_dialog->selectedFiles();
    }
    else return;
    QString tgt_name_f=tgt_name[0].section("/",-1);
    ui->label_oposename->setText(tgt_name_f);
    int tgt_number_index=tgt_name_f.lastIndexOf("_");
    tgt_name_f=tgt_name_f.mid(tgt_number_index+1,3);
    bool ok;
    tgt_number=tgt_name_f.toInt(&ok,10);
    pcl::io::loadPCDFile(qstr2str(tgt_name[0]),*tgt_cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<PointT>(tgt_cloud,"opose_cloud");
    ui->qvtkWidget->update();
}

void headpose::on_button_show_clicked()
{
    if(src_name.empty() || tgt_name.empty())
    {
        QMessageBox::information(this,QString::fromUtf8("提示"),QString::fromUtf8("请先选择点云数据！"),QMessageBox::Yes);
        return;
    }
    ui->edit_realangle->setText("");
    ui->edit_estangle->setText("");
    ui->edit_error->setText("");
    double reg_time=0.0;
    view_switch=0;
    icpmode=2;
    if(tgt_number==3 || tgt_number==2)
        sacmode=1;
    else sacmode=2;
    PointCloudT::Ptr middle_cloud (new PointCloudT);
    Eigen::Matrix4f sac_trans;
    reg_time=myreg.do_sacpre(src_cloud,tgt_cloud,middle_cloud,sac_trans,sacmode);
    reg_time+=myreg.do_icp(middle_cloud,tgt_cloud,final_cloud,final_transformation,icpmode);
    final_transformation=sac_trans*final_transformation;
    if(!(reg_time==0.0))
    {
        std::cout<<qstr2str(ui->comboBox->currentText())<<" takes "<<reg_time<<" seconds."<<std::endl;
        matrix2angle(final_transformation,ANGLE_result);
        Eigen::Vector3f real_angle,est_angle;
        float error;
        real_angle(0)=angle_table[tgt_number][0]-angle_table[src_number][0];
        real_angle(1)=angle_table[tgt_number][1]-angle_table[src_number][1];
        real_angle(2)=angle_table[tgt_number][2]-angle_table[src_number][2];
        est_angle(0)=ANGLE_result(0)-angle_table[src_number][0];
        est_angle(1)=ANGLE_result(1)-angle_table[src_number][1];
        est_angle(2)=ANGLE_result(2)-angle_table[src_number][2];
        error=computeerror(real_angle,est_angle);
        std::string realangles=std::to_string(real_angle(0))+" , "+std::to_string(real_angle(1))+" , "+std::to_string(real_angle(2));
        std::string estangles=std::to_string(est_angle(0))+" , "+std::to_string(est_angle(1))+" , "+std::to_string(est_angle(2));
        std::string errors=std::to_string(error);
//        ui->label_red->setVisible(true);
//        ui->label_green->setVisible(true);
//        ui->label_blue->setVisible(true);
        ui->label_time->setText(QString::number(reg_time,10,6));
        viewer->removeAllPointClouds();
//        viewer->addPointCloud<PointT>(tgt_cloud,ColorHandlerT(tgt_cloud,255,0,0),"tgtpose_cloud");
//        viewer->addPointCloud<PointT>(src_cloud,ColorHandlerT(src_cloud,0,255,0),"srcpose_cloud");
//        viewer->addPointCloud<PointT>(final_cloud,ColorHandlerT(final_cloud,0,0,255),"finalpose_cloud");
        viewer->addPointCloud<PointT>(tgt_cloud,"tgtpose_cloud");
        viewer->addPointCloud<PointT>(final_cloud,"finalpose_cloud");
        ui->qvtkWidget->update();
        ui->edit_realangle->setText(str2qstr(realangles));
        ui->edit_estangle->setText(str2qstr(estangles));
        ui->edit_error->setText(str2qstr(errors));
    }

//     PointCloudT::Ptr show_cloud (new PointCloudT);
//     std::vector<std::string> filesname;
//     std::vector<std::string> show_filesname;
//     std::string datas_folder=qstr2str(datas_name[0].section("/",-2,-2));
//     std::string dataspath=datapath+datas_folder+"/filtered/";
//     double icp_time=0.0;
//     pcl::getAllPcdFilesInDirectory(dataspath,filesname);
//     pcl::getAllPcdFilesInDirectory(datapath+datas_folder+"/",show_filesname);
//     pcl::io::loadPCDFile(dataspath+filesname[0],*src_cloud);
//     pcl::io::loadPCDFile(dataspath+filesname[0],*tgt_cloud);
//     Eigen::Matrix4f icp_trans;
//     final_transformation<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
//     for (int i=0;i<filesname.size();i++)
//     {
//         QCoreApplication::processEvents();
//         src_cloud->clear();
//         pcl::copyPointCloud(*tgt_cloud,*src_cloud);
//         pcl::io::loadPCDFile(dataspath+filesname[i],*tgt_cloud);
//         icp_time=myreg.do_icp(src_cloud,tgt_cloud,final_cloud,icp_trans);
//         final_transformation=icp_trans*final_transformation;
//         matrix2angle(final_transformation,ANGLE_result);
//         std::string icp_angles=std::to_string(ANGLE_result(0))+"  "+std::to_string(ANGLE_result(1))+"  "+std::to_string(ANGLE_result(2));
//         pcl::io::loadPCDFile(datapath+datas_folder+"/"+show_filesname[i],*show_cloud);
//         viewer->removeAllPointClouds();
//         viewer->addPointCloud<PointT>(show_cloud,"show_cloud");
//         ui->label_time->setText(QString::number(icp_time,10,6));
//         ui->qvtkWidget->update();
//     }
}

void headpose::on_button_switch_clicked()
{
    if(tgt_name.empty())
        return;
    QString switch_data_path=tgt_name[0];
    int filter_index=switch_data_path.indexOf("filtered");
    switch_data_path.remove(filter_index,9);
    filter_index=switch_data_path.indexOf("filtered");
    switch_data_path.remove(filter_index,9);
    if(view_switch==0)
    {
        PointCloudT::Ptr switch_cloud (new PointCloudT);
        pcl::io::loadPCDFile(qstr2str(switch_data_path),*switch_cloud);
        viewer->removeAllPointClouds();
        viewer->addPointCloud<PointT>(switch_cloud,"switch_cloud");
        ui->qvtkWidget->update();
        view_switch=1;
    }
    else
    {
        viewer->removeAllPointClouds();
//        viewer->addPointCloud<PointT>(tgt_cloud,ColorHandlerT(tgt_cloud,255,0,0),"tgtpose_cloud");
//        viewer->addPointCloud<PointT>(src_cloud,ColorHandlerT(src_cloud,0,255,0),"srcpose_cloud");
//        viewer->addPointCloud<PointT>(final_cloud,ColorHandlerT(final_cloud,0,0,255),"finalpose_cloud");
        viewer->addPointCloud<PointT>(tgt_cloud,"tgtpose_cloud");
        viewer->addPointCloud<PointT>(final_cloud,"finalpose_cloud");
        ui->qvtkWidget->update();
        view_switch=0;
    }
}
