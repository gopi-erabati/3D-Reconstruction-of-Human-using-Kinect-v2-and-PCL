#include "cloudregister.h"

//Constructor to initialize the pcl::PointCloud pointer.

CloudRegister::CloudRegister()
{
    //cloud = new pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
}

//Function to initialize cloud with the file located at path.

pcl::PointCloud<PointT>::Ptr CloudRegister::getCloud(const std::string& path)
{
    //CloudRegister cloudReg;

    //pcl::PointCloud<PointT>::Ptr cloud1 (new pcl::PointCloud<PointT>);
    //cloud1 = new pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);

    int ret = pcl::io::loadPCDFile(path, *cloud);

    if (ret < 0)
    {
        //PCL_ERROR("Couldn't read file %s\n", argv[1]);
        //return -1;
        cout<<"Could not read file";
    }
    return cloud;
}
