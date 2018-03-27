#ifndef CFILTER_H
#define CFILTER_H

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <QDebug>
using namespace std;

#include <string>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class cfilter
{
public:

    //typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    //typedef pcl::PointXYZ PointT;

    cfilter();
    ~cfilter();

    PointCloud::Ptr VoxelGridDownSample(PointCloud::Ptr , float);
    PointCloud::Ptr OutlierRemoval(PointCloud::Ptr , float);
    PointCloud::Ptr PassThrough(PointCloud::Ptr, float, float, float, float, float, float);
    PointCloud::Ptr Smoothing(PointCloud::Ptr, float );
    //void Filter (std::vector<PointCloud::Ptr> &, std::vector<PointCloud::Ptr> &, float , float , float , float , float , float );




private:
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

};

#endif // CFILTER_H
