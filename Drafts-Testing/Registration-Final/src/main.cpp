//#include <QCoreApplication>
#include "cfilter.h"
#include "regmesh.h"
#include "cinout.h"


int main(int argc, char *argv[])
{   
    string PCdirectory = "../Ziyang-full/cloud";

    std::vector<PointCloud::Ptr> data;
    std::vector<PointCloud::Ptr> outvect;

    PointCloud::Ptr RegisteredPointCloud (new PointCloud);

    cinout inoutObj;
    cfilter filterObj;
    regmesh RegisterObj;

    std::cout<< "Reading all pointclouds ... " << endl;
    inoutObj.ReadDirectory(PCdirectory, data);

    std::cout<< "Filtering all pointclouds ... " << endl;
    filterObj.Filter(data, outvect);

    std::cout<< "Registration started ... " << endl;
    RegisteredPointCloud = RegisterObj.Register(outvect);

    // writing the results to o/p pcd file
    pcl::io::savePCDFileASCII ("cloud_registered.pcd", *RegisteredPointCloud);



    cout << "The code is running"<< endl;

    return 0;
}
