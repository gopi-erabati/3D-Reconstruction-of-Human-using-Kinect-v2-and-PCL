#include "cfilter.h"

cfilter::cfilter() {}
cfilter::~cfilter() {}


PointCloud::Ptr cfilter::PassThrough(PointCloud::Ptr cloud, float z1, float z2 , float y1, float y2, float x1, float x2) {
    
    // Defining local PoinCloud pointer variables to be used internally within the method
    PointCloud::Ptr cloud_filteredz (new PointCloud); 
    PointCloud::Ptr cloud_filteredy (new PointCloud);
    PointCloud::Ptr cloud_filtered (new PointCloud);

    // Create the passthrough filtering object
    pcl::PassThrough<PointT> pass;

    qDebug()<<"inside pass through";

    pass.setInputCloud (cloud);  // passing the input cloud pointer
    pass.setFilterFieldName ("z"); // defining the dimension to be filtered
    pass.setFilterLimits (z1,z2);  // setting the limits 
    pass.filter (*cloud_filteredz); // filter and put the output in a point cloud variable

    qDebug()<<"z filtered";

    pass.setInputCloud (cloud_filteredz); // passing the input cloud pointer
    pass.setFilterFieldName ("y"); // defining the dimension to be filtered
    pass.setFilterLimits (y1,y2);  // setting the limits
    pass.filter (*cloud_filteredy); // filter and put the output in a point cloud variable

    pass.setInputCloud (cloud_filteredy); // passing the input cloud pointer
    pass.setFilterFieldName ("x"); // defining the dimension to be filtered
    pass.setFilterLimits (x1, x2); // setting the limits
    pass.filter (*cloud_filtered); // filter and put the output in a point cloud variable

    qDebug()<<"x and y filtered";

    return cloud_filtered;  // Return Pointer of the filtered point cloud
}


PointCloud::Ptr cfilter::OutlierRemoval(PointCloud::Ptr cloud, float Threshold) {
    
    // Defining local PoinCloud pointer variables to be used internally within the method
    PointCloud::Ptr cloud_filtered (new PointCloud);

    //Create the statistical-outlier-removal filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    qDebug()<<"inside outlier";

    sor.setInputCloud (cloud); // passing the input cloud pointer
    sor.setMeanK (50);         // Setting number of neighbors 
    sor.setStddevMulThresh (Threshold); // set the standard deviation threshold
    sor.filter (*cloud_filtered);       // filter and put the output in a point cloud variable

    return cloud_filtered; // Return Pointer of the filtered point cloud
}


PointCloud::Ptr cfilter::VoxelGridDownSample(PointCloud::Ptr cloud, float LeafSize) {
    
    // Defining local PoinCloud pointer variables to be used internally within the method
    PointCloud::Ptr cloud_filtered (new PointCloud);
    
    //Create the sVoxelGrid filtering object
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud); // passing the input cloud pointer
    
    // Dimensions of the Voxels used in the downsamplind >>
    // Bigger the leaf size, less points you get in output (more downsampling)
    vox.setLeafSize (LeafSize, LeafSize, LeafSize);
    vox.filter (*cloud_filtered); // filter and put the output in a point cloud variable

    return cloud_filtered;   // Return Pointer of the filtered point cloud
}


PointCloud::Ptr cfilter::Smoothing(PointCloud::Ptr cloud, float SearchRadius) {

    // Defining local PoinCloud pointer variables to be used internally within the method
    PointCloud::Ptr cloud_smoothed (new PointCloud);

    // Start MLS
    // Create a KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

    mls.setComputeNormals (true);
    // Setting parameters
    mls.setInputCloud (cloud); // passing the input cloud pointer
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);  // KD-Tree search method
    // Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting.
    mls.setSearchRadius (SearchRadius); 
    // Reconstruct
    mls.process (mls_points);

    pcl::copyPointCloud(mls_points, *cloud_smoothed); // To convert from XYZNormals to XYZ

    return cloud_smoothed; // Return Pointer of the smoothed point cloud
}

/*void cfilter::Filter(std::vector<PointCloud::Ptr> &DataVector, float z1, float z2, float y1, float y2, float x1, float x2)
{

    qDebug()<<"entered filter function";

    PointCloud::Ptr voxel_out (new PointCloud);
    PointCloud::Ptr pass_out (new PointCloud);
    PointCloud::Ptr outlier_out (new PointCloud);
    PointCloud::Ptr smoothing_out (new PointCloud);

    float LeafSize = 0.006;
    //float z1= 0.1, z2=2.0 ,y1=-0.85 , y2=1.2 , x1=-0.4 , x2=0.4 ;
    float Threshold = 5.0;
    //float SearchRadius = 0.03;

    for (int i=0; i< DataVector.size(); i++){

        voxel_out = VoxelGridDownSample( DataVector[i], LeafSize);
        qDebug()<<"Voxel Finished";

        pass_out = PassThrough(voxel_out, z1, z2, y1, y2, x1, x2);
        qDebug()<<"Pass through done";
        outlier_out = OutlierRemoval(pass_out, Threshold);
        qDebug()<<"outliers removed";
        //smoothing_out = Smoothing(outlier_out, SearchRadius);

        OutVector.push_back(outlier_out);
    }

}*/
