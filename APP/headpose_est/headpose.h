#ifndef HEADPOSE_H
#define HEADPOSE_H

#include <iostream>

#include <QMainWindow>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/file_io.h>

#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;
//    std::string datapath;
//    std::vector<std::string> filesname;
//    int fileindex;

private:
    Ui::headpose *ui;
};

#endif // HEADPOSE_H
