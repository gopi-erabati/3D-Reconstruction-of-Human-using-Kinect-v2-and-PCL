#include "cinout.h"
#include "regmesh.h"
#include "cfilter.h"

cinout::cinout() { }
//cinout::~cinout(){}


void cinout::ReadDirectory (string Directory, std::vector<PointCloud::Ptr> &DataVector) {
    int fileNumber = 1;

    for (size_t i =0; i<36; ++i )
    {
        // data.push_back(cloud);
        PointCloud::Ptr cloud (new PointCloud);

        //std::cout<<data.size() << endl;

        stringstream stream;
        stream << Directory << fileNumber << ".pcd";
        string filename = stream.str();

        // int ret = pcl::io::loadPCDFile (filename, *data[i]);
        int ret = pcl::io::loadPCDFile (filename, *cloud);
        if (ret < 0) {
        PCL_ERROR("Couldn't read file %s\n");
        //return -1;
        }

        DataVector.push_back(cloud);
        fileNumber++;
    }
    // data.push_back(*cloud);

    //PointCloud::Ptr ptr (new PointCloud);
    //ptr = &data[0];
}
