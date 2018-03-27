#ifndef FILEGRABBER_H
#define FILEGRABBER_H

#include <QDirIterator>
#include <iostream>
#include <vector>
#include <string>
#include <QString>
#include <QDir>
#include <QDebug>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include "cfilter.h"
#include <QFileDialog>
#include <algorithm>
#include <QStringList>

using namespace std;

//typedef pcl::PointXYZRGBA PointT;
 //typedef pcl::PointXYZRGBA PointT;

class FileGrabber
{
public:
    //Function to initialize the list of PCD files
    void initializeFileList();

    //Function to return the vector containing pcd paths
    vector<string> getPaths();

    //Function to return the vector containing pcd names
    vector<QString> getNames();

    //Function to return the no. of files in directory
    //int getCount();

    //Function to return vector containing point clouds
    vector<pcl::PointCloud<PointT>::Ptr> getPointClouds();

private:
    //Path for the folder
    QString path;

    //vector containing the list of file names
    vector<QString> pcdNames;

    //vector containing list of file paths
    vector<string> pcdPaths;

    //vector containing all the point clouds
    vector<pcl::PointCloud<PointT>::Ptr> cloudVector;

    //point cloud
    pcl::PointCloud<PointT>::Ptr cloud;

    //variable to hold the count of files in the directory
    //uint cloudCounter;
};

using namespace std;

#endif // FILEGRABBER_H
