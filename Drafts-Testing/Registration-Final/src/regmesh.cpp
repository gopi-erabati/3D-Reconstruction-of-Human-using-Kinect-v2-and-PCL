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

    //std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;

    std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
    //std::cout << "first Icp converged:" << icp.hasConverged() << " score: " <<
    //             icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;
    //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    //pcl::transformPointCloud (*src, *Final, transformation);
    return Final;

}


PointCloud::Ptr regmesh::ICP2(PointCloud::Ptr src, PointCloud::Ptr tgt, float MaxDistance1, float RansacVar1, float Iterations1, float MaxDistance2, float RansacVar2, float Iterations2){
    PointCloud::Ptr out1 (new PointCloud);
    PointCloud::Ptr out2 (new PointCloud);

    out1 = ICP(src, tgt, MaxDistance1, RansacVar1, Iterations1);
    out2 = ICP(out1, tgt, MaxDistance2, RansacVar2, Iterations2);

    *out2 += *tgt;
    return out2;
}


PointCloud::Ptr regmesh::ICP3(PointCloud::Ptr src, PointCloud::Ptr tgt, float MaxDistance1, float RansacVar1, float Iterations1, float MaxDistance2, float RansacVar2, float Iterations2){
    PointCloud::Ptr out1 (new PointCloud);
    PointCloud::Ptr out2 (new PointCloud);
    PointCloud::Ptr out3 (new PointCloud);

    out1 = ICP(src, tgt, MaxDistance1, RansacVar1, Iterations1);
    out2 = ICP(out1, tgt, MaxDistance2, RansacVar2, Iterations2);
    out3 = ICP(out2, tgt, MaxDistance2, RansacVar2, Iterations2);

    *out3 += *tgt;
    return out3;
}


PointCloud::Ptr regmesh::Register(std::vector<PointCloud::Ptr> &Data ) {

    PCL_INFO (" \n Loaded %d datasets ... \n", (int)Data.size ());

    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    PointCloud::Ptr cloud_src (new PointCloud);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    PointCloud::Ptr result1 (new PointCloud);
    PointCloud::Ptr result2 (new PointCloud);
    PointCloud::Ptr result3 (new PointCloud);
    PointCloud::Ptr result4 (new PointCloud);
    PointCloud::Ptr result5 (new PointCloud);


    float MaxDistance1 = 0.05, RansacVar1= 0.01; int Iterations1= 80;
    float MaxDistance2 = 0.015, RansacVar2= 0.01; int Iterations2 = 100;
    float VoxelGridLeafSize = 0.004; // 0.004
    float OutlierRemovalThreshold = 5.0;

    cfilter FilterObj;
    // initialization of source to first element of the data vector,
    // and starting the loop from 1
    result2 = Data[0];
    for (size_t i = 1; i < Data.size (); ++i)
    {
        std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;

        cloud_src = result2; // source
        cloud_tgt = Data[i]; // target

        //src = FilterObj.VoxelGridDownSample(cloud_src, VoxelGridLeafSize);
        //tgt = FilterObj.VoxelGridDownSample(cloud_tgt, VoxelGridLeafSize);

        //result1 = ICP2(src, tgt, MaxDistance1, RansacVar1, Iterations1, MaxDistance2, RansacVar2, Iterations2);
        result1 = ICP(cloud_src, cloud_tgt, MaxDistance2, RansacVar2, Iterations2);
        *result1 += *cloud_tgt;

        result2 = FilterObj.OutlierRemoval(result1, OutlierRemovalThreshold);

        std::cout << std::endl;
    }

    //result3 = FilterObj.OutlierRemoval(result2, 4.0);
    //result4 = FilterObj.OutlierRemoval(result3, 4.0);
    //result5 = FilterObj.Smoothing(result4, 0.03);

    return result2;

}

