#ifndef HEADPOSE_H
#define HEADPOSE_H

#include <iostream>

#include <QMainWindow>
#include <QDir>
#include <QMessageBox>
#include <QFileDialog>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/common/file_io.h>
#include <pcl/console/print.h>
#include <pcl/registration/transforms.h>

#include <vtkRenderWindow.h>
#include <Eigen/Core>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/signals2.hpp>

#include "useful_tools.hpp"
#include "preprocess/preprocess.h"
#include "registration/registration.h"
#include <time.h>
#include <fstream>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

namespace Ui {
class headpose;
}

class headpose : public QMainWindow
{
    Q_OBJECT

public:
    explicit headpose(QWidget *parent = 0);
    ~headpose();

private slots:
//    void on_pushButton_clicked();

    void on_button_collect_clicked();

    void on_button_choosePCD_clicked();

    void on_button_preprocess_clicked();

    void on_button_src_clicked();

    void on_button_tgt_clicked();

    void on_button_registration_clicked();

//    void on_pushButton_clicked();

    void on_button_show_clicked();

    void on_baseposeButton_clicked();

    void on_objposeButton_clicked();

    void on_button_switch_clicked();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr collect_cloud;
    PointCloudT::Ptr preprocess_cloud;
    PointCloudT::Ptr src_cloud;
    PointCloudT::Ptr tgt_cloud;
    PointCloudT::Ptr final_cloud;
    boost::mutex cloud_mutex_;
    PointCloudT::ConstPtr cloud_;
    QDir dir;
    std::string datapath;
    QStringList choose_pcd_name;
    QStringList src_name;
    QStringList tgt_name;
    Eigen::Matrix4f final_transformation;
    Eigen::Vector3f ANGLE_result;
    registration myreg;
//    std::vector<std::string> filesname;
//    int fileindex;
    void cloud_callback(const PointCloudT::ConstPtr& cloud);
    int angle_table[19][3]={
        {0,0,0},
        {-9,0,0},
        {-19,0,1},
        {-27,0,2},
        {11,0,0},
        {21,1,0},
        {30,1,3},
        {2,-12,3},
        {2,-19,3},
        {2,-28,3},
        {3,9,1},
        {4,20,1},
        {3,29,2},
        {2,1,-12},
        {0,1,-20},
        {1,1,-30},
        {3,0,15},
        {3,1,21},
        {3,1,33}
    };
    int sacmode,icpmode,src_number,tgt_number;
    int view_switch;

private:
    Ui::headpose *ui;
};

#endif // HEADPOSE_H
