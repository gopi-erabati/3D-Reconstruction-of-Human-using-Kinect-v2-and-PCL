#include "regmesh.h"
#include "cfilter.h"

regmesh::regmesh(){}
regmesh::~regmesh(){}


PointCloud::Ptr regmesh::ICP(PointCloud::Ptr src, PointCloud::Ptr tgt, float MaxDistance, float RansacVar, float Iterations){
    // Start first ICP
    PointCloud::Ptr Final (new PointCloud);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance (MaxDistance); //0.10 //0.015
    icp.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
    icp.setTransformationEpsilon (1e-8);
    icp.setMaximumIterations (Iterations);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*Final);



    //std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;

    return Final;

}


PointCloud::Ptr regmesh::ICP2(PointCloud::Ptr src, PointCloud::Ptr tgt, float MaxDistance1, float RansacVar1, float Iterations1, float MaxDistance2, float RansacVar2, float Iterations2){
    PointCloud::Ptr out1 (new PointCloud);
    PointCloud::Ptr out2 (new PointCloud);

    out1 = ICP(src, tgt, MaxDistance1, RansacVar1, Iterations1);
    out2 = ICP(out1, tgt, MaxDistance2, RansacVar2, Iterations2);

    //*out2 += *tgt;
    return out2;
}


PointCloud::Ptr regmesh::ICPNormal(PointCloud::Ptr src, PointCloud::Ptr tgt, float MaxDistance=0.015, float RansacVar = 0.01, float Iterations = 100){

    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr normals_icp (new PointCloudWithNormals);
    PointCloud::Ptr cloud_norm (new PointCloud);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (12);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-8);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of the datasets
    reg.setMaxCorrespondenceDistance (MaxDistance);
    reg.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
    reg.setMaximumIterations (Iterations);


    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);
    reg.align (*normals_icp);

    std::cout << "Normals converged with score: " << reg.getFitnessScore() << std::endl;
    Eigen::Matrix4f transform_normals = reg.getFinalTransformation ();
    pcl::transformPointCloud (*src, *cloud_norm, transform_normals);

    //*cloud_norm += *tgt;
    return cloud_norm;

}


PointCloud::Ptr regmesh::Register (std::vector<PointCloud::Ptr> &Data ) {

    PCL_INFO ("Loaded %d datasets ... \n", (int)Data.size ());

    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    PointCloud::Ptr cloud_src (new PointCloud);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    PointCloud::Ptr result1 (new PointCloud);
    PointCloud::Ptr result2 (new PointCloud);
    PointCloud::Ptr result3 (new PointCloud);
    PointCloud::Ptr result4 (new PointCloud);


    float MaxDistance = 0.015, RansacVar= 0.01;
    int Iterations= 100;
    float VoxelGridLeafSize = 0.002; // 0.004
    float OutlierRemovalThreshold = 5.0;

    cfilter FilterObj;

    // initialization of source to first element of the data vector, and starting the loop from 1
    result2 = Data[0];
    for (size_t i = 1; i < Data.size (); ++i)
    {
        std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;

        cloud_src = result2; // source
        cloud_tgt = Data[i]; // target

        src = FilterObj.VoxelGridDownSample(cloud_src, VoxelGridLeafSize);
        tgt = FilterObj.VoxelGridDownSample(cloud_tgt, VoxelGridLeafSize);

        //result1 = ICP2(src, tgt, MaxDistance1, RansacVar1, Iterations1, MaxDistance2, RansacVar2, Iterations2);
        result1 = ICP(src, tgt, MaxDistance, RansacVar, Iterations);
        *result1 += *cloud_tgt;

        result2 = FilterObj.OutlierRemoval(result1, OutlierRemovalThreshold);

        std::cout << std::endl;
    }

    result3 = FilterObj.OutlierRemoval(result2, 4.0);
    result4 = FilterObj.Smoothing(result3, 0.02);

    return result4;
}


PointCloud::Ptr regmesh::RegisterNormal (std::vector<PointCloud::Ptr> &Data ) {

    PCL_INFO ("Loaded %d datasets ... \n", (int)Data.size ());

    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    PointCloud::Ptr cloud_src (new PointCloud);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    PointCloud::Ptr result1 (new PointCloud);
    PointCloud::Ptr result2 (new PointCloud);
    PointCloud::Ptr result3 (new PointCloud);
    PointCloud::Ptr result4 (new PointCloud);

    float MaxDistance = 0.015, RansacVar= 0.01;
    int Iterations = 100;
    float VoxelGridLeafSize = 0.002; // 0.004
    float OutlierRemovalThreshold = 5.0;

    cfilter FilterObj;

    // initialization of source to first element of the data vector and starting the loop from 1
    result2 = Data[0];
    for (size_t i = 1; i < Data.size (); ++i)
    {
        std::cout << "ICP with Normals between frame " << i << " and " << i+1 << std::endl;

        cloud_src = result2; // source
        cloud_tgt = Data[i]; // target

        src = FilterObj.VoxelGridDownSample(cloud_src, VoxelGridLeafSize);
        tgt = FilterObj.VoxelGridDownSample(cloud_tgt, VoxelGridLeafSize);

        result1 = ICPNormal(src, tgt, MaxDistance, RansacVar, Iterations);
        *result1 += *cloud_tgt; // incremental

        result2 = FilterObj.OutlierRemoval(result1, OutlierRemovalThreshold);

        std::cout << std::endl;
    }

    result3 = FilterObj.OutlierRemoval(result2, 4.0);
    result4 = FilterObj.Smoothing(result3, 0.02);

    return result4;
}


PointCloud::Ptr regmesh::Register2 (std::vector<PointCloud::Ptr> &Data ) {

    PCL_INFO ("Loaded %d datasets ... \n", (int)Data.size ());

    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    PointCloud::Ptr cloud_src (new PointCloud);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    PointCloud::Ptr result1 (new PointCloud);
    PointCloud::Ptr result2 (new PointCloud);
    PointCloud::Ptr result3 (new PointCloud);
    PointCloud::Ptr result4 (new PointCloud);


    float MaxDistance1 = 0.05, RansacVar1= 0.01; int Iterations1= 100;
    float MaxDistance2 = 0.015, RansacVar2= 0.01; int Iterations2 = 100;
    float VoxelGridLeafSize = 0.002; // 0.004
    float OutlierRemovalThreshold = 5.0;

    cfilter FilterObj;

    // initialization of source to first element of the data vector, and starting the loop from 1
    result2 = Data[0];
    for (size_t i = 1; i < Data.size (); ++i)
    {
        std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;

        cloud_src = result2; // source
        cloud_tgt = Data[i]; // target

        src = FilterObj.VoxelGridDownSample(cloud_src, VoxelGridLeafSize);
        tgt = FilterObj.VoxelGridDownSample(cloud_tgt, VoxelGridLeafSize);

        result1 = ICP2(src, tgt, MaxDistance1, RansacVar1, Iterations1, MaxDistance2, RansacVar2, Iterations2);
        *result1 += *cloud_tgt;

        result2 = FilterObj.OutlierRemoval(result1, OutlierRemovalThreshold);

        std::cout << std::endl;
    }

    result3 = FilterObj.OutlierRemoval(result2, 4.0);
    result4 = FilterObj.Smoothing(result3, 0.02);

    return result4;
}

pcl::PolygonMesh regmesh::generateMeshGreedy(PointCloud::Ptr cloud)
{
    //UpSampling using MLS VoxelGrid Dilation
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.03);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (4);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
    //mls.setUpsamplingRadius (0.005);
    //mls.setUpsamplingStepSize (0.003);
    mls.setDilationVoxelSize(0.002);
    PointCloud::Ptr cloud_smoothed (new PointCloud);
    mls.process (*cloud_smoothed);

    //Normal Estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_smoothed);
    n.setInputCloud (cloud_smoothed);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*cloud_normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    boost::shared_ptr<pcl::PolygonMesh> triangles (new pcl::PolygonMesh);
    //Greedy Projection Triangulation
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (3);//3
    gp3.setMaximumNearestNeighbors (1200);
    gp3.setMaximumSurfaceAngle(M_PI);
    gp3.setMinimumAngle(M_PI/36);
    gp3.setMaximumAngle(2*M_PI/3 + M_PI/6);
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    //Laplacian Smoothing of mesh
    vtk.setInputMesh(triangles);
    vtk.setNumIter(20000);
    vtk.setConvergence(0.0001);
    vtk.setRelaxationFactor(0.0001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI/5);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);
    qDebug()<<"Triangulation Finished";
    pcl::io::saveVTKFile ("../Mesh/outputMeshGreedy.vtk", output);

    return output;
}

size_t regmesh::getTriangles()
{
    return output.polygons.size();
}


pcl::PolygonMesh regmesh::generateMeshGrid(PointCloud::Ptr cloud)
{
    //UpSampling using MLS VoxelGrid Dilation
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.03);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (4);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
    //mls.setUpsamplingRadius (0.005);
    //mls.setUpsamplingStepSize (0.003);
    mls.setDilationVoxelSize(0.002);
    PointCloud::Ptr cloud_smoothed (new PointCloud);
    mls.process (*cloud_smoothed);

    //Normal Estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_smoothed);
    n.setInputCloud (cloud_smoothed);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*cloud_normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    boost::shared_ptr<pcl::PolygonMesh> triangles (new pcl::PolygonMesh);

    //Grid Projection Triangulation

      gp.setInputCloud(cloud_with_normals);
      gp.setSearchMethod(tree2);
      gp.setResolution(0.005);
      gp.setPaddingSize(3);
      gp.reconstruct(*triangles);

      //Laplacian Smoothing of mesh
      vtk.setInputMesh(triangles);
      vtk.setNumIter(20000);
      vtk.setConvergence(0.0001);
      vtk.setRelaxationFactor(0.0001);
      vtk.setFeatureEdgeSmoothing(true);
      vtk.setFeatureAngle(M_PI/5);
      vtk.setBoundarySmoothing(true);
      vtk.process(output);
      qDebug()<<"Triangulation Finished";
      pcl::io::saveVTKFile ("../Mesh/outputMeshGrid.vtk", output);

      return output;


}

pcl::PolygonMesh regmesh::generateMeshPoisson(PointCloud::Ptr cloud)
{
    //UpSampling using MLS VoxelGrid Dilation
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.03);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (4);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
    //mls.setUpsamplingRadius (0.005);
    //mls.setUpsamplingStepSize (0.003);
    mls.setDilationVoxelSize(0.002);
    PointCloud::Ptr cloud_smoothed (new PointCloud);
    mls.process (*cloud_smoothed);

    //Normal estimation OMP
    ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud_smoothed);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud_smoothed, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);


    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    for(size_t i = 0; i < cloud_normals->size(); ++i){
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1;
      cloud_normals->points[i].normal_z *= -1;
    }

    // Concatenate the XYZ and normal fields*

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

     //Poisson Reconstruction


       poi.setInputCloud(cloud_with_normals);
       poi.setSearchMethod(tree2);
       //poi.setConfidence(false);
       //poi.setManifold(false);
       //poi.setOutputPolygons(false);
       poi.setDepth(12);
       poi.setScale(1.2);
       poi.setSolverDivide(8);
       poi.setIsoDivide(8);
       poi.setSamplesPerNode(2);
        boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
       poi.reconstruct(*mesh);

       //Laplacian Smoothing of mesh
       vtk.setInputMesh(mesh);
       vtk.setNumIter(20000);
       vtk.setConvergence(0.0001);
       vtk.setRelaxationFactor(0.0001);
       vtk.setFeatureEdgeSmoothing(true);
       vtk.setFeatureAngle(M_PI/5);
       vtk.setBoundarySmoothing(true);
       vtk.process(output);


}
