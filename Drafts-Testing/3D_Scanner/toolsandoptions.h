#ifndef TOOLSANDOPTIONS_H
#define TOOLSANDOPTIONS_H

#include <QPushButton>
#include <QObject>
#include <QWidget>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include "cloudregister.h"
#include "cloudDisplay.h"

using namespace std;

//typedef pcl::PointXYZ PointT;
 typedef pcl::PointXYZRGB PointT;

class ToolsAndOptions : public QWidget
{
    Q_OBJECT

private:
    //start button
    QPushButton *startButton_;

    //objects
    CloudRegister cloudReg;
    CloudDisplay disp;

    //-------------Data variables------------>
    std::string path;
    //pointer to read point cloud
    pcl::PointCloud<PointT>::Ptr cloud;

public:
    explicit ToolsAndOptions(QWidget *parent = 0);
    //getStartButton Function
    QPushButton* getStartButton();

public slots:

    //start button pressed() handler
    void on_startButton_pressed();
};

#endif // TOOLSANDOPTIONS_H
