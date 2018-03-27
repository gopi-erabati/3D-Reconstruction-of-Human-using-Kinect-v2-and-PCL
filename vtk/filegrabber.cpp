#include "filegrabber.h"

void FileGrabber::initializeFileList(int originalSize, int filteredSize)
{
    //Initialize the path
    QFileDialog folderDialog;
    folderDialog.setFileMode(QFileDialog::Directory);
    folderDialog.setOption(QFileDialog::ShowDirsOnly);

    if(folderDialog.exec())
        path = folderDialog.selectedFiles()[0];
    else
        path = "../Original/";

    qDebug() << "Path found";

    //Count the number of files in the directory

    //initialize point cloud
    cloud.reset(new pcl::PointCloud<PointT>);


    QStringList myPath = path.split('/');
    QString newPath = myPath.last();

    if(newPath == "Original")
    {
        qDebug()<<"inside if";
        for(int i = 1; i<=originalSize; i++)
        {
            stringstream stream;
            stream <<path.toStdString()<< "/InputCloud"<<i<<".pcd";
            stringstream name;
            name<<"InputCloud"<<i<<".pcd";
            pcl::io::loadPCDFile(stream.str(), *cloud);

            //add the point cloud to the cloud vector
            cloudVector.push_back(cloud);

            pcdNames.push_back(QString::fromStdString(name.str()));

            //Reset the point cloud for next iteration
            cloud.reset(new pcl::PointCloud<PointT>);
        }
    }
    else if(newPath == "Filtered")
    {
        qDebug()<<"inside if";
        for(int i = 1; i<=13; i++)
        {
            stringstream stream;
            stream <<path.toStdString()<< "/FilteredCloud"<<i<<".pcd";
            stringstream name;
            name<<"FilteredCloud"<<i<<".pcd";
            pcl::io::loadPCDFile(stream.str(), *cloud);

            //add the point cloud to the cloud vector
            cloudVector.push_back(cloud);

            pcdNames.push_back(QString::fromStdString(name.str()));

            //Reset the point cloud for next iteration
            cloud.reset(new pcl::PointCloud<PointT>);
        }
    }
    else if(newPath == "Registered")
    {
        stringstream stream;
        stream <<path.toStdString()<< "/RegisteredCloud"<<".pcd";
        stringstream name;
        name<<"RegisteredCloud"<<".pcd";
        pcl::io::loadPCDFile(stream.str(), *cloud);

        //add the point cloud to the cloud vector
        cloudVector.push_back(cloud);

        pcdNames.push_back(QString::fromStdString(name.str()));

        //Reset the point cloud for next iteration
        cloud.reset(new pcl::PointCloud<PointT>);
    }
}

vector<string> FileGrabber::getPaths()
{   
    return pcdPaths;
}

vector<QString> FileGrabber::getNames()
{
    sort(pcdNames.begin(), pcdNames.end());
    return pcdNames;
}

/*int FileGrabber::getCount()
{
    qDebug() << "in cloudGrabber";
    qDebug() << cloudCounter;
    return cloudCounter;
}*/

vector<PointCloud::Ptr> FileGrabber::getPointClouds()
{
    return cloudVector;
}
