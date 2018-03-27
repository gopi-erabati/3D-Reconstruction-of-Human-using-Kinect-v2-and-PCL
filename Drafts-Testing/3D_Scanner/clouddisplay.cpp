#include "cloudDisplay.h"

CloudDisplay::CloudDisplay(QWidget *parent) : QVTKWidget(parent)
{
    displayWidget_ = new QVTKWidget(this, Qt::Widget);
    m_visualizer = new pcl::visualization::PCLVisualizer ("3D Viewer");
    m_visualizer->setBackgroundColor(0,0,0);
    //renderWindow = m_visualizer->getRenderWindow();
    displayWidget_->SetRenderWindow(m_visualizer->getRenderWindow());
    displayWidget_->update();
}

QVTKWidget* CloudDisplay::getVTKWidget()
{
    return displayWidget_;
}

void CloudDisplay::displayCloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
    m_visualizer->addPointCloud(cloud);
}
