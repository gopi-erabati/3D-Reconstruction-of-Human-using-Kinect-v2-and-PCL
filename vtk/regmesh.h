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

    //Normal cloud OMP object
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;

    //Greedy Projection Triangulation object
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    //Grid Projection Triangulation object
    pcl::GridProjection<pcl::PointNormal> gp;

    //Poission Triangulation
    pcl::Poisson<pcl::PointNormal>poi;


    //Polygon Mesh Pointer
    //pcl::PolygonMesh triangles;

    //output Pointer
    pcl:: PolygonMesh output;

    //Laplacian object
    pcl:: MeshSmoothingLaplacianVTK vtk;

public:
    regmesh();
    ~regmesh();

    // This method implements the traditional Iterative Closest Point, that uses the distances
    // between correspondences as its cost function and try to minimize it
    // It takes 2 point clouds pointers, source and target, in addition to 3 variables, for
    // Maximum correspondence distance, Ransac Variable, and number of Iterations
    // The output is a point cloud pointer
    PointCloud::Ptr ICP(PointCloud::Ptr, PointCloud::Ptr, float, float, float);

    // This method implements the ICP with normals, that uses the distance between source correspondences,
    // and the plane of target's points normals.
    // It takes the same arguments as traditional ICP mentioned above
    PointCloud::Ptr ICPNormal(PointCloud::Ptr, PointCloud::Ptr, float, float, float);

    // This method implements 2 ICP functions using the ICP function mentioned above,
    // the first ICP is considered as initial alignment, and the ssecond is fine alignment
    // The arguments are the same as before but doubled, first arguments for the first ICP,
    // and the second for the seciond ICP.
    PointCloud::Ptr ICP2(PointCloud::Ptr, PointCloud::Ptr, float, float, float, float, float, float);

    // * The next functions Register, RegisterNormal, Register2, implements the registration using
    // 1 traditional ICP, 1 normal ICP, 2 traditional ICPs, respectively.
    // * For all of them, they take vector of point cloud pointers, and produce a pointer to the registered point cloud
    // * Registration is done using incremental ICP concept, that adds the aligned point cloud to the target,
    // this will finally produce a complete closed loop of the scanned object.
    // For each iteration, Voxel grid doewnsample is used, in addition to Outlier removal filter
    PointCloud::Ptr Register(std::vector<PointCloud::Ptr> & );
    PointCloud::Ptr RegisterNormal(std::vector<PointCloud::Ptr> & );
    PointCloud::Ptr Register2(std::vector<PointCloud::Ptr> & );

    //Meshing Function
    // All meshing functions take point cloud pointer as an input

    //Greedy Triangulation
    pcl::PolygonMesh generateMeshGreedy(PointCloud::Ptr cloud);

    //Grid Projection
    pcl::PolygonMesh generateMeshGrid(PointCloud::Ptr cloud);

    //Poisson Reconstruction
    pcl::PolygonMesh generateMeshPoisson(PointCloud::Ptr cloud);

    //no. of triangles getter function
    size_t getTriangles();



};

#endif // REGMESH_H
