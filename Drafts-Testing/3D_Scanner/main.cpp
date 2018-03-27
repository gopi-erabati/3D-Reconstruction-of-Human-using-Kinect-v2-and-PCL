#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.setupUi(w);

    w.show();

    //new pcl::PointCloud<PointT>::Ptr

    //cloudReg.CloudRegister();
    //CloudRegister cloudReg;
    /*path = "../bunny1.pcd";
    cout<<path;
    cloud = cloudReg.getCloud(path);*/

    return a.exec();
}
