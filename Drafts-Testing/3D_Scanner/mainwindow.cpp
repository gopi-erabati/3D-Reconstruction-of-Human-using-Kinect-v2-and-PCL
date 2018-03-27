#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cloudregister.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Intialize point cloud pointer
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupUi(MainWindow &w)
{
    //Get the screen resolution
    size = getSize.size();

    //Compute the length and breadth
    x_ = size.width();
    y_ = size.height();
    w.setFixedSize(x_, y_);

    //initialize central widget
    centralWidget_ = new QWidget(this);
    //Call the centralWidget function passing the dimensions as parameters
    centralWidget_ = widget.setupCentralWidget(x_, y_);
    setCentralWidget(centralWidget_);
}
