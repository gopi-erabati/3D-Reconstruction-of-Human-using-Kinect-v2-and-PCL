#include "myvtkwidget.h"
#include "kinect2_grabber.h"
#include <string.h>

MyVTKWidget::MyVTKWidget(QWidget *parent) : QWidget(parent)
{
    //initialize widgets and layouts
    centralWidget_ = new QWidget(this);
    sideContainer = new QWidget(this);
    buttonContainer = new QStackedWidget(this);
    page1 = new QWidget(this);
    page2 = new QWidget(this);
    page3 = new QWidget(this);
    page4 = new QWidget(this);
    mainLayout_ = new QHBoxLayout;
    sideLayout_ = new QVBoxLayout;
    buttonLayout = new QVBoxLayout;
    page1Layout = new QVBoxLayout;
    page2Layout = new QGridLayout;
    page3Layout = new QGridLayout;
    page4Layout = new QGridLayout;

    //set background color to white
    buttonContainer->setStyleSheet("background-color: white;");

    //Sliders
    xSlider1 = new QSlider(Qt::Horizontal, this);
    xSlider2 = new QSlider(Qt::Horizontal, this);
    ySlider1 = new QSlider(Qt::Horizontal, this);
    ySlider2 = new QSlider(Qt::Horizontal, this);
    zSlider1 = new QSlider(Qt::Horizontal, this);
    zSlider2 = new QSlider(Qt::Horizontal, this);
    setStyleSheet("QSlider::handle:vertical{background-color:red;}");
    qDebug()<< "slider initialized";

    //Slider value initialization
    xSlider1->setRange(-3000,0);
    xSlider2->setRange(0,3000);
    ySlider1->setRange(-3000,0);
    ySlider2->setRange(0,3000);
    zSlider1->setRange(0,1000);
    zSlider2->setRange(0,4000);

    qDebug()<< "Range set";

    xSlider1->setSliderPosition(-1000);
    xSlider2->setSliderPosition(1000);
    ySlider1->setSliderPosition(-1000);
    ySlider2->setSliderPosition(2000);
    zSlider1->setSliderPosition(0);
    zSlider2->setSliderPosition(3000);

    //xSlider1->se

    xSlider1->setSingleStep(1);
    xSlider2->setSingleStep(1);
    ySlider1->setSingleStep(1);
    ySlider2->setSingleStep(1);
    zSlider2->setSingleStep(1);

    //Slider Connections (Signals and Slots)
    QObject::connect(xSlider1, SIGNAL(valueChanged(int)), this, SLOT(xSlider1ValueChanged(int)));
    QObject::connect(xSlider2, SIGNAL(valueChanged(int)), this, SLOT(xSlider2ValueChanged(int)));
    QObject::connect(ySlider1, SIGNAL(valueChanged(int)), this, SLOT(ySlider1ValueChanged(int)));
    QObject::connect(ySlider2, SIGNAL(valueChanged(int)), this, SLOT(ySlider2ValueChanged(int)));
    QObject::connect(zSlider1, SIGNAL(valueChanged(int)), this, SLOT(zSlider1ValueChanged(int)));
    QObject::connect(zSlider2, SIGNAL(valueChanged(int)), this, SLOT(zSlider2ValueChanged(int)));

    qDebug()<<"connected";

    //Labels
    xLabel1 = new QLabel(this);
    xLabel2 = new QLabel(this);
    yLabel1 = new QLabel(this);
    yLabel2 = new QLabel(this);
    zLabel1 = new QLabel(this);
    zLabel2 = new QLabel(this);

    //Label Initialize
    xLabel1->setText("-1");
    xLabel2->setText("1");
    yLabel1->setText("-1");
    yLabel2->setText("2");
    zLabel1->setText("0");
    zLabel2->setText("3");

    //Minimum and maximum lablse
    xMinimum = new QLabel(this);
    xMaximum = new QLabel(this);
    yMinimum = new QLabel(this);
    yMaximum = new QLabel(this);
    zMinimum = new QLabel(this);
    zMaximum = new QLabel(this);
    empty = new QLabel(this);

    xMinimum->setText("  Minimum X");
    xMaximum->setText("  Maximum X");
    yMinimum->setText("  Minimum Y");
    yMaximum->setText("  Maximum Y");
    zMinimum->setText("  Minimum Z");
    zMaximum->setText("  Maximum Z");
    empty->setText(" ");

    //Label for ICP options
    icpLabel = new QLabel("Choose an ICP option", this);

    //Label for ICP options
    meshingLabel = new QLabel("Choose a meshing option", this);


    //Customizing the font for the labels
    QFont font = xLabel1->font();
    font.setFamily(QString::fromUtf8("Arial"));
    font.setPointSize(12);
    xLabel1->setFont(font);
    xLabel2->setFont(font);
    yLabel1->setFont(font);
    yLabel2->setFont(font);
    zLabel1->setFont(font);
    zLabel2->setFont(font);

    font.setPointSize(15);
    icpLabel->setFont(font);
    icpLabel->setStyleSheet("font-weight: bold;");

    font.setPointSize(15);
    meshingLabel->setFont(font);
    meshingLabel->setStyleSheet("font-weight: bold;");

    font.setFamily(QString::fromUtf8("Arial"));
    font.setPointSize(12);
    xMinimum->setFont(font);
    xMaximum->setFont(font);
    yMinimum->setFont(font);
    yMaximum->setFont(font);
    zMinimum->setFont(font);
    zMaximum->setFont(font);

    //Initialize points for generating planes
    x1_ = -0.354;
    x2_ = 0.456;
    y1_ = -1.118;
    y2_ = 0.593;
    z1_ = 1;
    z2_ = 2.433;
    sliderChangedFlag = false;

    //Initialize Progress Bar
    progressBar = new QProgressBar(this);

    //Initialize Checkbox
    checkbox = new QCheckBox("Confirm Parameters",this);

    checkbox->setFont(font);
    checkbox->setStyleSheet(
                "QCheckBox{"
                "width: 100px;"
                "height: 100px;"
                "border-radius: 5px;"
                "}");
    //PushButtons
    //Initialize Read a point clod Button
    readButton_ = new QPushButton(this);
    //QPixmap buttonimage("D:\\QT\\vtk\\Images\\read.jpg");
    QPixmap buttonimage("..\\Images\\read.jpg");
    QIcon Icon;
    Icon.addPixmap(buttonimage, QIcon::Normal,QIcon::Off);
    readButton_->setIcon(Icon);
    readButton_->setFixedSize(400,250);
    readButton_->setIconSize(QSize(220, 220));

    //Initialize Load Kinect Button
    loadKinect = new QPushButton(this);
    //QPixmap buttonimage1("D:\\QT\\vtk\\Images\\kinect.jpg");
    QPixmap buttonimage1("..\\Images\\kinect.jpg");
    QIcon Icon1;
    Icon1.addPixmap(buttonimage1, QIcon::Normal,QIcon::Off);
    loadKinect->setIcon(Icon1);
    loadKinect->setFixedSize(400,250);
    loadKinect->setIconSize(QSize(220, 220));

    //Initialize Register Point Clouds Button
    registerButton = new QPushButton(this);
    //QPixmap buttonimage2("D:\\QT\\vtk\\Images\\register.jpg");
    QPixmap buttonimage2("..\\Images\\register.jpg");
    QIcon Icon2;
    Icon2.addPixmap(buttonimage2, QIcon::Normal,QIcon::Off);
    registerButton->setIcon(Icon2);
    registerButton->setFixedSize(400,250);
    registerButton->setIconSize(QSize(220, 220));

    //Initialize Meshing Button
    meshingButton = new QPushButton(this);
    //QPixmap buttonimage3("D:\\QT\\vtk\\Images\\meshing.png");
    QPixmap buttonimage3("..\\Images\\meshing.png");
    QIcon Icon3;
    Icon3.addPixmap(buttonimage3, QIcon::Normal,QIcon::Off);
    meshingButton->setIcon(Icon3);
    meshingButton->setFixedSize(400,250);
    meshingButton->setIconSize(QSize(220, 220));

    //Initialize Save file Button
    saveButton = new QPushButton(this);
    //QPixmap buttonimage4("D:\\QT\\vtk\\Images\\save.jpg");
    QPixmap buttonimage4("..\\Images\\save.jpg");
    QIcon Icon4;
    Icon4.addPixmap(buttonimage4, QIcon::Normal,QIcon::Off);
    saveButton->setIcon(Icon4);
    saveButton->setFixedSize(400,250);
    saveButton->setIconSize(QSize(220, 220));


    //Initialize Scanning Button
    scanButton = new QPushButton(this);
    //QPixmap buttonimage5("D:\\QT\\vtk\\Images\\start.jpg");
    QPixmap buttonimage5("..\\Images\\start.jpg");
    QIcon Icon5;
    Icon5.addPixmap(buttonimage5, QIcon::Normal,QIcon::Off);
    scanButton->setIcon(Icon5);
    scanButton->setFixedSize(100,100);
    scanButton->setIconSize(QSize(100, 100));

    //Style sheet for button borders
    scanButton->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize Stop Button
    stopButton = new QPushButton(this);
    //QPixmap buttonimage6("D:\\QT\\vtk\\Images\\stop.jpg");
    QPixmap buttonimage6("..\\Images\\stop.jpg");
    QIcon Icon6;
    Icon6.addPixmap(buttonimage6, QIcon::Normal,QIcon::Off);
    stopButton->setIcon(Icon6);
    stopButton->setFixedSize(100,100);
    stopButton->setIconSize(QSize(100, 100));
    stopButton->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize go to the previous page Button
    backButton = new QPushButton(this);
    //QPixmap buttonimage7("D:\\QT\\vtk\\Images\\back.jpg");
    QPixmap buttonimage7("..\\Images\\back.jpg");
    QIcon Icon7;
    Icon7.addPixmap(buttonimage7, QIcon::Normal,QIcon::Off);
    backButton->setIcon(Icon7);
    backButton->setFixedSize(100,100);
    backButton->setIconSize(QSize(100, 100));
    backButton->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize second back Button
    back2Button = new QPushButton(this);
    //QPixmap buttonimage8("D:\\QT\\vtk\\Images\\back.jpg");
    QPixmap buttonimage8("..\\Images\\back.jpg");
    QIcon Icon8;
    Icon8.addPixmap(buttonimage8, QIcon::Normal,QIcon::Off);
    back2Button->setIcon(Icon8);
    back2Button->setFixedSize(100,100);
    back2Button->setIconSize(QSize(100, 100));
    back2Button->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize third back Button
    back3Button = new QPushButton(this);
    //QPixmap buttonimage9("D:\\QT\\vtk\\Images\\back.jpg");
    QPixmap buttonimage9("..\\Images\\back.jpg");
    QIcon Icon9;
    Icon9.addPixmap(buttonimage9, QIcon::Normal,QIcon::Off);
    back3Button->setIcon(Icon9);
    back3Button->setFixedSize(100,100);
    back3Button->setIconSize(QSize(100, 100));
    back3Button->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize done Button
    doneButton = new QPushButton(this);
    //QPixmap buttonimage10("D:\\QT\\vtk\\Images\\done.jpg");
    QPixmap buttonimage10("..\\Images\\done.jpg");
    QIcon Icon10;
    Icon10.addPixmap(buttonimage10, QIcon::Normal,QIcon::Off);
    doneButton->setIcon(Icon10);
    doneButton->setFixedSize(100,100);
    doneButton->setIconSize(QSize(100, 100));
    doneButton->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");

    //Initialize second done Button
    done2Button = new QPushButton(this);
    //QPixmap buttonimage11("D:\\QT\\vtk\\Images\\done.jpg");
    QPixmap buttonimage11("..\\Images\\done.jpg");
    QIcon Icon11;
    Icon11.addPixmap(buttonimage11, QIcon::Normal,QIcon::Off);
    done2Button->setIcon(Icon11);
    done2Button->setFixedSize(100,100);
    done2Button->setIconSize(QSize(100, 100));
    done2Button->setStyleSheet(
                "QPushButton{"
                "border-style: outset;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: white;"
                "}"

                "QPushButton:pressed{"
                "border-style: inset;"
                "border-style: solid;"
                "border-width: 2px;"
                "border-radius: 10px;"
                "border-color: black;"
                "}");


    //RadioButtons
    //Initialize ICP  RadioButton
    icp1Button = new QRadioButton("ICP with normals",this);
    icp2Button = new QRadioButton("One ICP ",this);
    icp3Button = new QRadioButton("Two ICPs ",this);

    //cutomize the font and size
    font.setPointSize(14);
    icp1Button->setFont(font);
    icp2Button->setFont(font);
    icp3Button->setFont(font);

    //Initializing meshing RadioButtons
    meshing1Button = new QRadioButton("Greedy Projection Triangulation",this);
    meshing2Button = new QRadioButton("Poisson Reconstruction",this);
    meshing3Button = new QRadioButton("Grid Projection",this);

    //cutomize the font and size
    font.setPointSize(14);
    meshing1Button->setFont(font);
    meshing2Button->setFont(font);
    meshing3Button->setFont(font);

    //Intialize VTK Widget
    //startButton_ = new QPushButton(this);
    displayWidget_ = new QVTKWidget;
    vtkObject::GlobalWarningDisplayOff();

    //Initialize the cloud list widget and connect it to the slot
    cloudList = new QListWidget(this);
    QObject::connect(cloudList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(on_cloudListItem_clicked(QListWidgetItem*)));

    //Connect the pushButton pressed/clicked signal to its slot
    QObject::connect(readButton_, SIGNAL(pressed()), this, SLOT(on_readButton_pressed()));
    QObject::connect(scanButton, SIGNAL(clicked()), this, SLOT(on_scanButton_clicked()));
    QObject::connect(stopButton, SIGNAL(clicked()), this, SLOT(on_stopButton_clicked()));
    QObject::connect(loadKinect, SIGNAL(pressed()), this, SLOT(on_loadKinect_pressed()));
    QObject::connect(backButton, SIGNAL(pressed()), this, SLOT(on_backButton_pressed()));
    QObject::connect(back2Button, SIGNAL(pressed()), this, SLOT(on_backButton_pressed()));
    QObject::connect(back3Button, SIGNAL(pressed()), this, SLOT(on_backButton_pressed()));
    QObject::connect(registerButton, SIGNAL(pressed()), this, SLOT(on_registerButton_pressed()));
    QObject::connect(meshingButton, SIGNAL(pressed()), this, SLOT(on_meshingButton_pressed()));
    QObject::connect(doneButton, SIGNAL(pressed()), this, SLOT(on_doneButton_pressed()));
    QObject::connect(done2Button, SIGNAL(pressed()), this, SLOT(on_doneButton2_pressed()));

    //Connect the checkbox pressed signal to its slot
    QObject::connect(checkbox, SIGNAL(clicked()),  this, SLOT(clickedCheckBox()));

    //Make Scan button checkable
    //scanButton->setCheckable(true);
    stopButton->setCheckable(true);

    //initialize point cloud
    cloud.reset(new PointCloud);
    registeredCloud.reset(new PointCloud);



    //initialize PCL m_visualizer
    m_visualizer.reset(new pcl::visualization::PCLVisualizer ("3D m_visualizer", false));

    //m_visualizer->resetStoppedFlag();
    m_visualizer->setBackgroundColor(0.1,0.1,0.1);



    //renderWindow = m_visualizer->getRenderWindow();
    displayWidget_->SetRenderWindow(m_visualizer->getRenderWindow());
    m_visualizer->setupInteractor(displayWidget_->GetInteractor(), displayWidget_->GetRenderWindow());


    //Set representation to wire frame
    //Set it to surface later using --- setRepresentationToSurfaceForAllActors()
    m_visualizer->setRepresentationToWireframeForAllActors();
}

QWidget* MyVTKWidget::getCentralWidget()
{
    //get the screen resolution
    ScreenSetup screenSetup;
    QSize size = screenSetup.size();
    int x = size.width();
    int y = size.height();

    //xSlider1->di

    //set the dimensions and positions for widgets
    //displayWidget_->setFixedSize(x,(3*(y/4)));
    //displayWidget_->setGeometry((x/4),0,x,y);
    //startButton_->setFixedSize(x,(y/4));
    //sideContainer->setGeometry(0,0, (x/4), y);
    sideContainer->setFixedWidth((x/5));
    buttonContainer->setFixedWidth((x/5));

    //setup central widget
    mainLayout_->setSpacing(5);
    mainLayout_->setContentsMargins(5,0,5,0);

    buttonLayout->setSpacing(5);
    sideLayout_->setSpacing(5);
    sideLayout_->setContentsMargins(5,5,5,5);

    page1Layout->addWidget(readButton_);
    page1Layout->addWidget(loadKinect);
    page1Layout->addWidget(registerButton);
    page1Layout->addWidget(meshingButton);

    //Adding widgets to Page 2 - when Register isPressed
    //Adding the sliders
    page2Layout->addWidget(xSlider1, 1,0);
    page2Layout->addWidget(xSlider2, 3,0);
    page2Layout->addWidget(xMinimum, 1,1);
    page2Layout->addWidget(xMaximum, 3,1);
    page2Layout->addWidget(xLabel1, 0,0);
    page2Layout->addWidget(xLabel2, 2,0);
    page2Layout->addWidget(empty, 4,0);

    page2Layout->addWidget(ySlider1, 6,0);
    page2Layout->addWidget(ySlider2, 8,0);
    page2Layout->addWidget(yMinimum, 6,1);
    page2Layout->addWidget(yMaximum, 8,1);
    page2Layout->addWidget(yLabel1, 5,0);
    page2Layout->addWidget(yLabel2, 7,0);
    page2Layout->addWidget(empty, 9,0);

    page2Layout->addWidget(zSlider1, 11,0);
    page2Layout->addWidget(zSlider2, 13,0);
    page2Layout->addWidget(zMinimum, 11,1);
    page2Layout->addWidget(zMaximum, 13,1);
    page2Layout->addWidget(zLabel1, 10,0);
    page2Layout->addWidget(zLabel2, 12,0);
    page2Layout->addWidget(empty, 14,0);
    page2Layout->addWidget(checkbox, 15,0,1,2);
    page2Layout->addWidget(empty, 16,0);

    //Adding the buttons
    page2Layout->addWidget(scanButton, 17,0);
    page2Layout->addWidget(stopButton, 17,1);
    page2Layout->addWidget(backButton, 17,2);

    //Widgets to Page 3 - when Meshing isPressed
    //Adding the labels and buttons
    page3Layout->addWidget(icpLabel, 1,0,1,3);
    page3Layout->addWidget(icp1Button, 2,0,2,3);
    page3Layout->addWidget(icp2Button, 4,0,2,3);
    page3Layout->addWidget(icp3Button, 6,0,2,3);
    page3Layout->addWidget(doneButton, 17,1);
    page3Layout->addWidget(back2Button, 17,2);

    page4Layout->addWidget(meshingLabel, 1,0,1,3);
    page4Layout->addWidget(meshing1Button, 2,0,2,3);
    page4Layout->addWidget(meshing2Button, 4,0,2,3);
    page4Layout->addWidget(meshing3Button, 6,0,2,3);

    page4Layout->addWidget(done2Button, 17,1);
    page4Layout->addWidget(back3Button, 17,2);

    page1->setLayout(page1Layout);
    page2->setLayout(page2Layout);
    page3->setLayout(page3Layout);
    page4->setLayout(page4Layout);

    buttonContainer->addWidget(page1);
    buttonContainer->addWidget(page2);
    buttonContainer->addWidget(page3);
    buttonContainer->addWidget(page4);
    //buttonContainer->setLayout(buttonLayout);

    sideLayout_->addWidget(cloudList);
    sideLayout_->addWidget(progressBar);
    sideContainer->setLayout(sideLayout_);

    mainLayout_->addWidget(buttonContainer);
    mainLayout_->addWidget(displayWidget_);
    mainLayout_->addWidget(sideContainer);
    centralWidget_->setLayout(mainLayout_);

    return centralWidget_;
}

void MyVTKWidget::on_readButton_pressed()
{
    //remove point cloud
    //m_visualizer->removePointCloud("cloud");

    //Read and display the point cloud
    qDebug() << "read pressed";

    //Clear the list to avoid duplicates
    cloudList->clear();

    fileGrab.initializeFileList(cloudVector.size(), filteredCloudVector.size());
    pcdNames = fileGrab.getNames();

    //access the names
    vector<QString>::iterator it = pcdNames.begin();
    while(it != pcdNames.end())
    {
        cloudList->addItem(*it);
        qDebug()<< *it;
        //qDebug () << *it <<" ";
        it++;
    }

    cloudVector = fileGrab.getPointClouds();

    //Set the flag to select vectors hen displaying
    vectorSelectorFlag = 1; //access original point clouds

    //Display the first point cloud in the list
    m_visualizer->addPointCloud(cloudVector[0], "cloud_read");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_read");
    m_visualizer->addCoordinateSystem();
    displayWidget_->update();
    /*
    pcl::io::loadPCDFile(path, *cloud);
    qDebug() << cloud->points.size();
    m_visualizer->addPointCloud(cloud, "ccc");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ccc");
    m_visualizer->addCoordinateSystem();
    displayWidget_->update();*/
}

void MyVTKWidget::on_cloudListItem_clicked(QListWidgetItem *item)
{
    vector<PointCloud::Ptr> temp;

    switch(vectorSelectorFlag)
    {
    case 1: //For unprocessed point clouds
        temp = cloudVector;
        break;

    case 2: //For Pass Through Filtered Point clouds
        temp = filteredCloudVector;
        //m_visualizer->removeAllPointClouds();
        //displayWidget_->update();
        qDebug() << "in case 2";
        break;

    case 3: //Add the mesh
        m_visualizer->setBackgroundColor(1.0,1.0,1.0);
        m_visualizer->removeAllPointClouds();
        m_visualizer->addPolygonMesh(outMesh, "polygon");
        displayWidget_->update();
        break;


    default: qDebug()<<"defalut not required";
    }

    qDebug() << vectorSelectorFlag;
    //qDebug() << temp.size();
    for(int i = 0; i < temp.size(); i++)
    {
        //displayWidget_->update();
        if(cloudList->item(i) == item)
        {
            //displayWidget_->update();
            qDebug() << cloudList->item(i);
            qDebug() << "in if checklist";
            qDebug() << item;
            m_visualizer->removePointCloud("cloud_scan");
            //m_visualizer->removeAll
            m_visualizer->addPointCloud(temp[i], "cloud_scan");
            //m_visualizer->spinOnce();
            displayWidget_->update();
        }
    }
}

void MyVTKWidget::on_scanButton_clicked()
{
    //Turntable serial setting

    /*serial.setPortName("COM3");
    serial.open(QIODevice::ReadWrite);
    serial.setBaudRate(QSerialPort::Baud4800);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    //while(!serial.isOpen()) serial.open(QIODevice::ReadWrite);*/


    m_visualizer->removeShape("cube");
    //m_visualizer->removePointCloud("cloud_load");
    m_visualizer->addCoordinateSystem();
    //m_visualizer->setCameraPosition( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );


    // pcl::StopWatch clock;

    fileSaved = 1;
    save_one = false;
    pcl::PointCloud<PointT>::Ptr cloud_scan;
    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;

    qDebug()<<"inside scan function";

    // Point Cloud callback function
    boost::function<void( const pcl::PointCloud<PointT>::ConstPtr& )> pointcloud_function =
            [&cloud_scan, &mutex]( const pcl::PointCloud<PointT>::ConstPtr& ptr ){
        boost::mutex::scoped_lock lock( mutex );
        cloud_scan = ptr->makeShared();
};




    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( pointcloud_function );


    // Start Grabber
    grabber->start();
    //clock.reset();


    cloudVector.clear();

    qDebug () << "started";

    // for(int i=0; i <= 10000; i++){

    //qDebug() << stopButton->isChecked();
    while( !stopButton->isChecked() )
    {

        displayWidget_->update();



        /*while(!serial.isOpen()) serial.open(QIODevice::ReadWrite);

                if (serial.isOpen() && serial.isWritable())

                {
                    var = 1;
                    y[0] = var;
                    serial.write(y);
                    serial.flush();
                    timer.start();
                    qDebug()<<timer.elapsed();
                    serial.waitForBytesWritten(100);

                }*/


        boost::mutex::scoped_try_lock lock( mutex );
        //qDebug() << (cloud && lock.owns_lock());
        if ( cloud_scan && lock.owns_lock() ){
            if( cloud_scan->size() != 0 ){
                /* Processing Point Cloud */

                // Update Point Cloud
                if( !m_visualizer->updatePointCloud( cloud_scan, "cloud_scan" ) ){


                    m_visualizer->addPointCloud( cloud_scan, "cloud_scan" );
                    m_visualizer->resetCameraViewpoint( "cloud_scan" );

                }


                stringstream stream;
                stream << "../Original/InputCloud" << fileSaved << ".pcd";
                stringstream stream1;
                stream1 << "Cloud" << fileSaved << ".pcd";


                filename = stream.str();
                pcl::io::savePCDFileASCII(filename,*cloud_scan);
                qDebug()<<"converting to qstring";
                QString qFilename = QString::fromStdString(stream1.str());
                pcdNames.push_back(qFilename);
                cloudList->addItem(qFilename);
                int ret = pcl::io::loadPCDFile (filename, *cloud);
                if (ret < 0) {
                    PCL_ERROR("Couldn't read file %s\n");
                }
                cloudVector.push_back(cloud);

                //reset the cloud to read the next file
                cloud.reset(new PointCloud);


                //counter for saving files
                fileSaved++;
                cout << "Saved" << filename << "." << endl;
                qDebug() << "file saved";
                save_one = true;


            }
        }

        //m_visualizer->removePointCloud("cloud_scan");

        if (stopButton->isDown())
        {
            grabber->stop();
            /*var = 0;
            y[0]=var;
            serial.write(y);
            serial.flush();
            timer.start();
            qDebug()<<timer.elapsed();
            serial.waitForBytesWritten(100);
            Sleep(6000000);*/


        }
        QCoreApplication::processEvents();

    }
    vectorSelectorFlag = 1;
}

void MyVTKWidget::on_stopButton_clicked()
{

    m_visualizer->removePointCloud("cloud_load");
    m_visualizer->removePointCloud("cloud_scan");

    displayWidget_->update();
}

void MyVTKWidget::on_loadKinect_pressed()
{
    //qDebug() << buttonContainer->currentIndex();
    buttonContainer->setCurrentIndex(1);

    //m_visualizer->setCameraPosition( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
    pcl::PointCloud<PointT>::ConstPtr cloud_load;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointT>::ConstPtr& )> pointcloud_function =
            [&cloud_load, &mutex]( const pcl::PointCloud<PointT>::ConstPtr& ptr ){
        boost::mutex::scoped_lock lock( mutex );
        cloud_load = ptr->makeShared();
    };

    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( pointcloud_function );

    grabber->start();

    //Add a cube to the Visualizer
    m_visualizer->addCube(x1_,x2_,y1_,y2_,z1_,z2_,1.0,0.0,0.0,"cube",0);
    m_visualizer->setRepresentationToWireframeForAllActors();
    displayWidget_->update();

    while(!scanButton->isChecked())
    {
        displayWidget_->update();

        //Update the cube
        if(sliderChangedFlag)
        {
            m_visualizer->removeShape("cube");
            m_visualizer->addCube(x1_,x2_,y1_,y2_,z1_,z2_,1.0,0.0,0.0,"cube",0);
            sliderChangedFlag = false;
            displayWidget_->update();
            m_visualizer->setRepresentationToWireframeForAllActors();
        }

        //Run Grabber
        boost::mutex::scoped_try_lock lock( mutex );
        if ( cloud_load && lock.owns_lock() ){
            if( cloud_load->size() != 0 ){
                /* Processing Point Cloud */
                // Update Point Cloud
                if( !m_visualizer->updatePointCloud( cloud_load, "cloud_load" ) )
                {
                    //m_visualizer->removePointCloud("cloud");
                    m_visualizer->addPointCloud( cloud_load, "cloud_load" );
                    m_visualizer->resetCameraViewpoint( "cloud_load" );

                }
            }
        }

        QCoreApplication::processEvents();
        if (scanButton->isDown())
        {

            grabber->stop();
        }
    }

}

void MyVTKWidget::on_backButton_pressed()
{
    buttonContainer->setCurrentIndex(0);

}

void MyVTKWidget::xSlider1ValueChanged(int x1)
{
    x1_ = float(x1)/1000.0;
    xLabel1->setText(QString::number(x1_));
    sliderChangedFlag = true;
}

void MyVTKWidget::xSlider2ValueChanged(int x2)
{
    x2_ = float(x2)/1000.0;
    xLabel2->setText(QString::number(x2_));
    sliderChangedFlag = true;
}

void MyVTKWidget::ySlider1ValueChanged(int y1)
{
    y1_ = float(y1)/1000.0;
    yLabel1->setText(QString::number(y1_));
    sliderChangedFlag = true;
}

void MyVTKWidget::ySlider2ValueChanged(int y2)
{
    y2_ = float(y2)/1000.0;
    yLabel2->setText(QString::number(y2_));
    sliderChangedFlag = true;
}

void MyVTKWidget::zSlider1ValueChanged(int z1)
{
    z1_ = float(z1)/1000.0;
    zLabel1->setText(QString::number(z1_));
    sliderChangedFlag = true;
}

void MyVTKWidget::zSlider2ValueChanged(int z2)
{
    z2_ = float(z2)/1000.0;
    zLabel2->setText(QString::number(z2_));
    sliderChangedFlag = true;
}

void MyVTKWidget::on_registerButton_pressed()
{
    //Change the
    buttonContainer->setCurrentIndex(2);

}

void MyVTKWidget::on_meshingButton_pressed()
{

    buttonContainer->setCurrentIndex(3);

}

void MyVTKWidget::clickedCheckBox()
{
    xSlider1->setEnabled(false);
    xSlider2->setEnabled(false);

    ySlider1->setEnabled(false);
    ySlider2->setEnabled(false);

    zSlider1->setEnabled(false);
    zSlider2->setEnabled(false);
}

void MyVTKWidget::on_doneButton_pressed()
{
    if (icp2Button->isChecked() == true)
    {
        //point clouds for filtering
        PointCloud::Ptr ds_cloud (new PointCloud);
        PointCloud::Ptr pt_cloud (new PointCloud);
        PointCloud::Ptr ol_cloud (new PointCloud);

        //Threshold values for filters
        float LeafSize = 0.002;
        float Threshold = 5.0;

        fileSaved = 1;

        vector<PointCloud::Ptr>::iterator it = cloudVector.begin();
        while(it != cloudVector.end())
        {
            qDebug()<< "filtering";

            //Filter the point cloud using pass through filter
            pt_cloud = myCfilter.PassThrough(*it, z1_, z2_, y1_, y2_, x1_, x2_);

            //Remove outliers
            ol_cloud = myCfilter.OutlierRemoval(pt_cloud, Threshold);


            //Add the filtered cloud to the vector
            filteredCloudVector.push_back(ol_cloud);

            //Save the file
            stringstream stream;
            stream << "../Filtered/FilteredCloud" << fileSaved << ".pcd";
            //qDebug() << fileSaved ;
            filename = stream.str();

            pcl::io::savePCDFileASCII(filename,*ol_cloud);

            //Reset the point clouds for the next iteration
            ds_cloud.reset(new PointCloud);
            pt_cloud.reset(new PointCloud);
            ol_cloud.reset(new PointCloud);


            it++;
            fileSaved++;
        }

        //Call the register function to perform ICP
        registeredCloud = myRegmesh.Register(filteredCloudVector);

        //Add the cloud to the list and vector
        cloudList->addItem("registeredCloud");
        filteredCloudVector.push_back(registeredCloud);

        //Save registered point cloud
        stringstream stream;
        stream << "../Registered/RegisteredCloud" << ".pcd";
        //qDebug() << fileSaved ;
        filename = stream.str();

        pcl::io::savePCDFileASCII(filename,*registeredCloud);

        qDebug()<< "Out of while loop";

        /*m_visualizer->removeAllPointClouds();
        m_visualizer->addPointCloud(filteredCloudVector[0], "cloud");
        displayWidget_->update();*/

        vectorSelectorFlag = 2;
    }
    else if (icp1Button->isChecked() == true)
    {
        //point clouds for filtering
        PointCloud::Ptr ds_cloud (new PointCloud);
        PointCloud::Ptr pt_cloud (new PointCloud);
        PointCloud::Ptr ol_cloud (new PointCloud);

        //Threshold values for filters
        LeafSize = 0.004;
        Threshold = 5.0;

        fileSaved = 1;

        vector<PointCloud::Ptr>::iterator it = cloudVector.begin();
        while(it != cloudVector.end())
        {
            qDebug()<< "filtering";

            //Filter the point cloud using pass through filter
            pt_cloud = myCfilter.PassThrough(*it, z1_, z2_, y1_, y2_, x1_, x2_);

            //Remove outliers
            ol_cloud = myCfilter.OutlierRemoval(pt_cloud, Threshold);



            //Add the filtered cloud to the vector
            filteredCloudVector.push_back(ol_cloud);

            //Save the file
            stringstream stream;
            stream << "../Filtered/FilteredCloud" << fileSaved << ".pcd";
            //qDebug() << fileSaved ;
            filename = stream.str();

            pcl::io::savePCDFileASCII(filename,*ol_cloud);

            //Reset the point clouds for the next iteration
            ds_cloud.reset(new PointCloud);
            pt_cloud.reset(new PointCloud);
            ol_cloud.reset(new PointCloud);


            it++;
            fileSaved++;
        }

        //Call the register function to perform ICP
        registeredCloud = myRegmesh.RegisterNormal(filteredCloudVector);

        //Add the cloud to the list and vector
        cloudList->addItem("registeredCloud");
        filteredCloudVector.push_back(registeredCloud);

        //Save registered point cloud
        stringstream stream;
        stream << "../Registered/RegisteredCloud" << ".pcd";
        //qDebug() << fileSaved ;
        filename = stream.str();

        pcl::io::savePCDFileASCII(filename,*registeredCloud);


        vectorSelectorFlag = 2;
    }
    else if (icp3Button->isChecked() ==  true)
    {
        //point clouds for filtering
        PointCloud::Ptr ds_cloud (new PointCloud);
        PointCloud::Ptr pt_cloud (new PointCloud);
        PointCloud::Ptr ol_cloud (new PointCloud);

        //Threshold values for filters
        LeafSize = 0.004;
        Threshold = 5.0;

        fileSaved = 1;

        vector<PointCloud::Ptr>::iterator it = cloudVector.begin();
        while(it != cloudVector.end())
        {
            qDebug()<< "filtering";

            //Filter the point cloud using pass through filter
            pt_cloud = myCfilter.PassThrough(*it, z1_, z2_, y1_, y2_, x1_, x2_);

            //Remove outliers
            ol_cloud = myCfilter.OutlierRemoval(pt_cloud, Threshold);



            //Add the filtered cloud to the vector
            filteredCloudVector.push_back(ol_cloud);

            //Sav the file
            stringstream stream;
            stream << "../Filtered/FilteredCloud" << fileSaved << ".pcd";
            //qDebug() << fileSaved ;
            filename = stream.str();

            pcl::io::savePCDFileASCII(filename,*ol_cloud);

            //Reset the point clouds for the next iteration
            ds_cloud.reset(new PointCloud);
            pt_cloud.reset(new PointCloud);
            ol_cloud.reset(new PointCloud);


            it++;
            fileSaved++;
        }

        //Call the register function to perform ICP
        registeredCloud = myRegmesh.Register2(filteredCloudVector);

        //Add the cloud to the list and vector
        cloudList->addItem("registeredCloud");
        filteredCloudVector.push_back(registeredCloud);

        //Save registered point cloud
        stringstream stream;
        stream << "../Registered/RegisteredCloud" << ".pcd";
        //qDebug() << fileSaved ;
        filename = stream.str();

        pcl::io::savePCDFileASCII(filename,*registeredCloud);



        vectorSelectorFlag = 2;
    }
}

void MyVTKWidget::on_doneButton2_pressed()
{
    if (meshing1Button->isChecked() == true)
    {
        // Greedy Projection triangulation

        //Call the meshing function from regmesh class
        outMesh = myRegmesh.generateMeshGreedy(filteredCloudVector.back());

        //Get the number of triangles in the mesh
        triangleNumber = myRegmesh.getTriangles();

        cloudList->addItem("Final Mesh Greedy");

        vectorSelectorFlag = 3;

    }
    else if (meshing2Button->isChecked() == true)
    {
        // Poission Triangulation

        //Call the meshing function from regmesh class
        outMesh = myRegmesh.generateMeshPoisson(filteredCloudVector.back());

        cloudList->addItem("Final Mesh Poisson");

        vectorSelectorFlag = 3;

    }
    else if (meshing3Button->isChecked() == true)
    {
        // Grid Projection Triangulation

        //Call the meshing function from regmesh class
        outMesh = myRegmesh.generateMeshGrid(filteredCloudVector.back());

        cloudList->addItem("Final Mesh Grid");

        vectorSelectorFlag = 3;


    }
}
