#ifndef CLOUDDISPLAY_H
#define CLOUDDISPLAY_H

#include <QWidget>
#include <QVTKWidget.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>

//typedef pcl::PointXYZ PointT;
 typedef pcl::PointXYZRGB PointT;

class CloudDisplay : public QVTKWidget
{

public:
    //CloudDisplay();

    //Constructor
    CloudDisplay(QWidget *parent = 0);

    //QVTKWidget to render the point cloud
    QVTKWidget* getVTKWidget();

    //Function to dislplay the point cloud
    void displayCloud(pcl::PointCloud<PointT>::Ptr &cloud);

private:
    pcl::visualization::PCLVisualizer *m_visualizer;
    QVTKWidget *displayWidget_;
    //vtkSmartPointer<vtkRenderWindow> renderWindow;
};

#endif // CLOUDDISPLAY_H
