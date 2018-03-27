#include "cfilter.h"

cfilter::cfilter() {}
cfilter::~cfilter() {}


PointCloud::Ptr cfilter::VoxelGridDownSample(PointCloud::Ptr cloud, float LeafSize) {
    // Downsample using voxelgrid
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (LeafSize, LeafSize, LeafSize);
    vox.filter (*cloud_filtered);
    return cloud_filtered;
}

PointCloud::Ptr cfilter::OutlierRemoval(PointCloud::Ptr cloud, float Threshold) {
    //Create the statistical-outlier-removal filtering object
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    qDebug()<<"inside outlier";
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (Threshold); // Threshold = 5.0
    sor.filter (*cloud_filtered);

    //pcl::io::savePCDFileASCII ("test_outlier.pcd", *cloud_filtered);

    return cloud_filtered;
}

PointCloud::Ptr cfilter::PassThrough(PointCloud::Ptr cloud, float z1, float z2 , float y1, float y2, float x1, float x2) {

    PointCloud::Ptr cloud_filteredz (new PointCloud);
    PointCloud::Ptr cloud_filteredy (new PointCloud);
    PointCloud::Ptr cloud_filtered (new PointCloud);

    // Create the passthrough filtering object
    pcl::PassThrough<PointT> pass;

    qDebug()<<"inside pass through";

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z1,z2); //0.1,2.0
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filteredz);

    qDebug()<<"z filtered";

    pass.setInputCloud (cloud_filteredz);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y1,y2); // -0.85,1.2
    pass.filter (*cloud_filteredy);

    pass.setInputCloud (cloud_filteredy);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x1, x2); // -0.4,0.4
    pass.filter (*cloud_filtered);

    qDebug()<<"x and y filtered";

    //pcl::io::savePCDFileASCII ("test_filtered.pcd", *cloud_filtered);
    return cloud_filtered;
}

PointCloud::Ptr cfilter::Smoothing(PointCloud::Ptr cloud, float SearchRadius) {

    PointCloud::Ptr cloud_smoothed (new PointCloud);

    // Start MLS
    // Create a KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

    mls.setComputeNormals (true);
    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (SearchRadius);
    // Reconstruct
    mls.process (mls_points);

    pcl::copyPointCloud(mls_points, *cloud_smoothed); // convert from normals to XYZ
    return cloud_smoothed;
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
