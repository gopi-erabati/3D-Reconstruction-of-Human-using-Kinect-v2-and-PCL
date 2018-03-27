#ifndef MYVTKWIDGET_H
#define MYVTKWIDGET_H

#include <QWidget>
#include <QVTKWidget.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <QDebug>
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>
#include <iostream>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSize>
#include "screensetup.h"
#include <QDebug>
#include <QProgressBar>
#include "filegrabber.h"
#include <vector>
#include <QString>
#include <QDebug>
#include "kinect2_grabber.h"
#include "cfilter.h"
#include "cinout.h"
#include <QStackedWidget>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include "cfilter.h"
#include <pcl/common/time.h>
#include "regmesh.h"
#include <QPixmap>
#include <QIcon>

#include <QCheckBox>
#include <QRadioButton>

#include <Windows.h>
#include <QElapsedTimer>
#include <QSerialPort>

using namespace std;

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointXYZRGBA PointT;

class MyVTKWidget : public QWidget
{
    Q_OBJECT

public:
    MyVTKWidget(QWidget *parent = 0);
    QWidget* getCentralWidget();

private:
    QPushButton *readButton_;
    QPushButton *scanButton;
    QPushButton *loadKinect;
    QPushButton *backButton;
    QPushButton *registerButton;
    QPushButton *meshingButton;
    QPushButton *back2Button;
    QPushButton *back3Button;
    QPushButton *stopButton;
    QPushButton *saveButton;
    QPushButton *doneButton;
    QPushButton *done2Button;

    QRadioButton *icp1Button;
    QRadioButton *icp2Button;
    QRadioButton *icp3Button;

    QRadioButton *meshing1Button;
    QRadioButton *meshing2Button;
    QRadioButton *meshing3Button;



    QCheckBox *checkbox;

    //-------------Data variables------------>
    vector<QString> pcdNames;
    //uint count;
    float x1_,x2_,y1_,y2_,z1_,z2_;
    bool sliderChangedFlag;
    int vectorSelectorFlag;
    unsigned int fileSaved;
    string filename;
    bool save_one;
    bool var;
    float LeafSize;
    float Threshold;

    //Serial communication
    QSerialPort serial;
    QElapsedTimer timer;
    QByteArray y;


    //vector containing all the point clouds
    vector<PointCloud::Ptr> cloudVector;
    vector<PointCloud::Ptr> filteredCloudVector;

    //Mesh Container
    pcl::PolygonMesh outMesh;

    //Number of triangles
    size_t triangleNumber;

    //point cloud
    PointCloud::Ptr cloud;
    PointCloud::Ptr registeredCloud;

    //PCL Viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;

    //widgets and layouts
    QVTKWidget *displayWidget_;
    QWidget *centralWidget_;
    QHBoxLayout *mainLayout_;
    QVBoxLayout *sideLayout_;
    QVBoxLayout *buttonLayout;
    QVBoxLayout *page1Layout;
    QGridLayout *page2Layout;
    QGridLayout *page3Layout;
    QGridLayout *page4Layout;
    QWidget *sideContainer;
    QStackedWidget *buttonContainer;
    QWidget *page1;
    QWidget *page2;
    QWidget *page3;
    QWidget *page4;
    QProgressBar *progressBar;
    QListWidget *cloudList;

    QSlider *xSlider1;
    QSlider *xSlider2;
    QSlider *ySlider1;
    QSlider *ySlider2;
    QSlider *zSlider1;
    QSlider *zSlider2;

    QLabel *xLabel1;
    QLabel *xLabel2;
    QLabel *yLabel1;
    QLabel *yLabel2;
    QLabel *zLabel1;
    QLabel *zLabel2;

    QLabel *xMinimum;
    QLabel *xMaximum;
    QLabel *yMinimum;
    QLabel *yMaximum;
    QLabel *zMinimum;
    QLabel *zMaximum;
    QLabel *empty;

    QLabel *icpLabel;
    QLabel *meshingLabel;

    //Objects
    FileGrabber fileGrab;
    cfilter myCfilter;
    regmesh myRegmesh;

public slots:

    //startButton pressed slot
    void on_readButton_pressed();

    //Scan button clicked slot
    void on_scanButton_clicked();

    //stop button clicked slot
    void on_stopButton_clicked();

    //point cloud list slot
    void on_cloudListItem_clicked(QListWidgetItem* item);

    //Load Kinect button handler
    void on_loadKinect_pressed();

    //Back Button Handler
    void on_backButton_pressed();

    //Registrer button handler
    void on_registerButton_pressed();

    //Triangulate button handler
    void on_meshingButton_pressed();

    // Checkbox slot
    void clickedCheckBox();

    //Slider Slots
    void xSlider1ValueChanged(int x1);
    void xSlider2ValueChanged(int x2);
    void ySlider1ValueChanged(int y1);
    void ySlider2ValueChanged(int y2);
    void zSlider1ValueChanged(int z1);
    void zSlider2ValueChanged(int z2);

    //done button slots
    void on_doneButton_pressed();
    void on_doneButton2_pressed();
};

#endif // MYVTKWIDGET_H
