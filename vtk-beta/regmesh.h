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
#include <cstddef>
#include <cstdint>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/mls.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class regmesh
{
private:
    //upsampling point cloud object
    pcl::MovingLeastSquares<PointT, PointT> mls;

    //normal cloud object
    pcl::NormalEstimation<PointT, pcl::Normal> n;

    //Greedy Projection Triangulation object
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    //Polygon Mesh Pointer
    //pcl::PolygonMesh triangles;

    //output Pointer
    pcl:: PolygonMesh output;

    //Laplacian object
    pcl:: MeshSmoothingLaplacianVTK vtk;

public:
    regmesh();
    ~regmesh();

    PointCloud::Ptr ICP(PointCloud::Ptr, PointCloud::Ptr, float, float, float);
    PointCloud::Ptr ICP2(PointCloud::Ptr, PointCloud::Ptr, float, float, float, float, float, float);
    PointCloud::Ptr ICP3(PointCloud::Ptr, PointCloud::Ptr, float, float, float, float, float, float);
    PointCloud::Ptr Register(std::vector<PointCloud::Ptr> & );
    PointCloud::Ptr ICPNormal(PointCloud::Ptr, PointCloud::Ptr, float, float, float);

    //Meshing Function
    pcl::PolygonMesh generateMesh(PointCloud::Ptr cloud);

    //no. of triangles getter function
    size_t getTriangles();



};

#endif // REGMESH_H
