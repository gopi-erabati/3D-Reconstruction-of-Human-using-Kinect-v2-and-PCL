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
#include <pcl/console/parse.h>
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
	// The PassThrough filter cuts the given point cloud and just keep the points within the given X Y Z dimensions
    // First argument is a PointCloud pointer to the point cloud to be filtered
    // the rest of the input arguments are the dimensions in Z dimension (Zmin, and Zmax), in Y dimension (Ymin, Ymax), 
    // then in X dimension (Xmin, Xmax) >> In our software this arguments are determined by the cube in the GUI
    // The output of this function is a PointCloud pointer to the filtered point cloud
    PointCloud::Ptr PassThrough(PointCloud::Ptr, float, float, float, float, float, float);

	// The OutlierRemoval is the method for removing the oulier of a given point cloud using PCL StatisticalOutlierRemoval filter
    // It takes 2 arguments, a pointer to the pointcloud to be filtered, and the standard deviation threshold
    // The output is a PointCloud pointer to the filtered point cloud
    PointCloud::Ptr OutlierRemoval(PointCloud::Ptr , float);

    // The VoxelGridDownSample is the method for downsampling a point cloud using the PCL Voxelgrid filter
    // It takes 2 arguments, a pointer to the pointcloud to be downsampled, and the leaf size defining the dimensions
    // of the Voxel used in the downsamplind >> Bigger the leaf size, less points you get in output (more downsampling)
    // The output is a PointCloud pointer to the downsampled point cloud    
    PointCloud::Ptr VoxelGridDownSample(PointCloud::Ptr , float);

    // The smoothing function smoothes the given point cloud using PCL Moving Least squares smoothing filter
    // it takes the pointer to the input point cloud, and the smoothing search radius parameter 
    // Higher the parameter, more smoothe the output will be >> BE CAREFUL as it can remove all the details
    PointCloud::Ptr Smoothing(PointCloud::Ptr, float );

    //void Filter (std::vector<PointCloud::Ptr> &, std::vector<PointCloud::Ptr> &, float , float , float , float , float , float );


private:
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

};

#endif // CFILTER_H
