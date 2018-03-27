#include "filegrabber.h"

void FileGrabber::initializeFileList()
{
    //Initialize the path
    QFileDialog folderDialog;
    folderDialog.setFileMode(QFileDialog::Directory);
    folderDialog.setOption(QFileDialog::ShowDirsOnly);

    if(folderDialog.exec())
        path = folderDialog.selectedFiles()[0];

    qDebug() << "Path found";

    //Count the number of files in the directory
    /*QDir dir(path);
    dir.setFilter( QDir::AllEntries | QDir::NoDotAndDotDot );
    cloudCounter = dir.count();
    qDebug() << "counter from file grabber" << cloudCounter;*/

    //initialize point cloud
    cloud.reset(new pcl::PointCloud<PointT>);

    /*/Initialize the point cloud Vector
    for(int i=0; i < cloudCounter; i++)
    {

        cloudVector.push_back(cloud);
    }*/

    //Iterate over the directory getting files with .pcd extension
    QDirIterator it(path, QStringList() << "*.pcd", QDir::Files);


    while (it.hasNext())
    {
        //Save the paths to the files
        pcdPaths.push_back(it.next().toStdString());

        //Save the file names
        pcdNames.push_back(it.fileName());

        qDebug()<<it.fileName();

        //read the point clouds and add to the vector
        pcl::io::loadPCDFile(it.filePath().toStdString(), *cloud);

        //add the point cloud to the cloud vector
        cloudVector.push_back(cloud);

        //Reset the point cloud for next iteration
        cloud.reset(new pcl::PointCloud<PointT>);
        //i++;

        qDebug() << it.filePath();
        //qDebug() << cloudVector(i);
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
