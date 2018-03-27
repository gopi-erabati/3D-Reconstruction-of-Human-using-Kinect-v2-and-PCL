#ifndef REGMESH_H
#define REGMESH_H

#include "cfilter.h"
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
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class regmesh
{
public:
    regmesh();
    ~regmesh();

    PointCloud::Ptr ICP(PointCloud::Ptr, PointCloud::Ptr, float, float, float);
    PointCloud::Ptr ICP2(PointCloud::Ptr, PointCloud::Ptr, float, float, float, float, float, float);
    PointCloud::Ptr ICP3(PointCloud::Ptr, PointCloud::Ptr, float, float, float, float, float, float);
    PointCloud::Ptr Register(std::vector<PointCloud::Ptr> & );



};

#endif // REGMESH_H
