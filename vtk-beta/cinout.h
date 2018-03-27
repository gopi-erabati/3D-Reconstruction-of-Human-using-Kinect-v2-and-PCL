#ifndef CINOUT_H
#define CINOUT_H

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
#include <string>
#include "cfilter.h"

using namespace std;

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class cinout
{
public:
    cinout();
    //~cinout();

    void ReadDirectory (string , std::vector<PointCloud::Ptr> & );

};

#endif // CINOUT_H
