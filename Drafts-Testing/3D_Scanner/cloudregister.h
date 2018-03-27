#ifndef CLOUDREGISTER_H
#define CLOUDREGISTER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

using namespace std;

//typedef pcl::PointXYZ PointT;
 typedef pcl::PointXYZRGB PointT;

/*-------------------------------------------------------
 * class Cloud Register
 *
 * Description : To read and initialize .pcd files into a working
 * point cloud.
 *
 * operators : pcl::PointCloud pointer
 *
 * functions : pcl::PointCloud getCloud ->
 *             input parameter -> path of type std::string
 *             return type : pcl::PointCloud pointer
*/

class CloudRegister
{
    private:
    pcl::PointCloud<PointT>::Ptr cloud;

public:
    CloudRegister();
    pcl::PointCloud<PointT>::Ptr getCloud(const std::string& path);
};

#endif // CLOUDREGISTER_H
