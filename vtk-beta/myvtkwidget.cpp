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
    mainLayout_ = new QHBoxLayout;
    sideLayout_ = new QVBoxLayout;
    buttonLayout = new QVBoxLayout;
    page1Layout = new QVBoxLayout;
    page2Layout = new QGridLayout;

    qDebug()<< " about to crash";

    //Sliders
    xSlider1 = new QSlider(Qt::Vertical, this);
    xSlider2 = new QSlider(Qt::Vertical, this);
    ySlider1 = new QSlider(Qt::Vertical, this);
    ySlider2 = new QSlider(Qt::Vertical, this);
    zSlider1 = new QSlider(Qt::Vertical, this);
    zSlider2 = new QSlider(Qt::Vertical, this);
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

    //Slider Connections
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

    //Intialize VTK Widget
    //startButton_ = new QPushButton(this);
    displayWidget_ = new QVTKWidget;
    vtkObject::GlobalWarningDisplayOff();

    //Initialize startButton
    readButton_ = new QPushButton("Read", this);
    scanButton = new QPushButton("Scan", this);
    stopButton = new QPushButton("Stop", this);
    loadKinect = new QPushButton("Load Kinect", this);
    backButton = new QPushButton("Back", this);
    registerButton = new QPushButton("Register", this);
    meshButton = new QPushButton("Triangulate", this);

    //Initialize the cloud list widget and connect it to the slot
    cloudList = new QListWidget(this);
    QObject::connect(cloudList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(on_cloudListItem_clicked(QListWidgetItem*)));

    //Connect the pushButton pressed signal to its slot
    QObject::connect(readButton_, SIGNAL(pressed()), this, SLOT(on_readButton_pressed()));
    QObject::connect(scanButton, SIGNAL(clicked()), this, SLOT(on_scanButton_clicked()));
    QObject::connect(stopButton, SIGNAL(clicked()), this, SLOT(on_stopButton_clicked()));
    QObject::connect(loadKinect, SIGNAL(pressed()), this, SLOT(on_loadKinect_pressed()));
    QObject::connect(backButton, SIGNAL(pressed()), this, SLOT(on_backButton_pressed()));
    QObject::connect(registerButton, SIGNAL(pressed()), this, SLOT(on_registerButton_pressed()));
    QObject::connect(meshButton, SIGNAL(pressed()), this, SLOT(on_meshButton_pressed()));

    //Make Scan button checkable
    scanButton->setCheckable(true);
    stopButton->setCheckable(true);

    //initialize point cloud
    cloud.reset(new PointCloud);
    registeredCloud.reset(new PointCloud);

    /*qDebug() << "Calling get count";
    count = fileGrab.getCount();

    for(int i=0; i < count; i++)
    {

        cloudVector.push_back(cloud);
    }*/

    //initialize PCL m_visualizer
    m_visualizer.reset(new pcl::visualization::PCLVisualizer ("3D m_visualizer", false));
    m_visualizer->resetStoppedFlag();
    m_visualizer->setBackgroundColor(0.1,0.1,0.1);
    qDebug() << m_visualizer->wasStopped();
    cout << m_visualizer->wasStopped() << "test" <<endl;
    //renderWindow = m_visualizer->getRenderWindow();
    displayWidget_->SetRenderWindow(m_visualizer->getRenderWindow());
    m_visualizer->setupInteractor(displayWidget_->GetInteractor(), displayWidget_->GetRenderWindow());

//    m_visualizer->setCameraClipDistances(9.77142, 19.2308);
//    m_visualizer->setCameraPosition(10.2063, 1.22232, -7.88023,-0.0891232, 0.995802, -0.0208837);
//    m_visualizer->setPosition(394, 0);
//    m_visualizer->setSize(1132, 965);
//    m_visualizer->setCameraFieldOfView(30);


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
    page1Layout->addWidget(meshButton);

    page2Layout->addWidget(xSlider1, 0,0);
    page2Layout->addWidget(xSlider2, 0,1);
    page2Layout->addWidget(xLabel1, 1,0);
    page2Layout->addWidget(xLabel2, 1,1);

    page2Layout->addWidget(ySlider1, 2,0);
    page2Layout->addWidget(ySlider2, 2,1);
    page2Layout->addWidget(yLabel1, 3,0);
    page2Layout->addWidget(yLabel2, 3,1);

    page2Layout->addWidget(zSlider1, 4,0);
    page2Layout->addWidget(zSlider2, 4,1);
    page2Layout->addWidget(zLabel1, 5,0);
    page2Layout->addWidget(zLabel2, 5,1);

    page2Layout->addWidget(scanButton, 6,0);
    page2Layout->addWidget(stopButton, 6,1);
    page2Layout->addWidget(backButton, 6,2);

    page1->setLayout(page1Layout);
    page2->setLayout(page2Layout);

    buttonContainer->addWidget(page1);
    buttonContainer->addWidget(page2);
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

    fileGrab.initializeFileList();
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
            m_visualizer->removePointCloud("cloud_read");
            //m_visualizer->removeAll
            m_visualizer->addPointCloud(temp[i], "cloud_read");
            //m_visualizer->spinOnce();
            displayWidget_->update();
        }
    }
}

void MyVTKWidget::on_scanButton_clicked()
{
    //Turntable serial setting
    /*serial.setPortName("COM7");
    serial.open(QIODevice::ReadWrite);
    serial.setBaudRate(QSerialPort::Baud4800);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);*/
    //while(!serial.isOpen()) serial.open(QIODevice::ReadWrite);
    m_visualizer->removeShape("cube");
    //m_visualizer->removePointCloud("cloud_load");
    m_visualizer->addCoordinateSystem();
    //m_visualizer->setCameraPosition( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
    qDebug () << "hello";

    pcl::StopWatch clock;

    fileSaved = 1;
    save_one = false;
    pcl::PointCloud<PointT>::Ptr cloud_scan;
    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;

    qDebug()<<"inside scan function";

    boost::function<void( const pcl::PointCloud<PointT>::ConstPtr& )> pointcloud_function =
            [&cloud_scan, &mutex]( const pcl::PointCloud<PointT>::ConstPtr& ptr ){
        boost::mutex::scoped_lock lock( mutex );
        cloud_scan = ptr->makeShared();


    };


    qDebug()<<"cloud initialized";

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( pointcloud_function );

//    // Keyboard Callback Function and saving PCD file - (NEW function added)
//    //int fileSaved = 0;
//    boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function =
//            [&cloud_scan, &mutex, &fileSaved]( const pcl::visualization::KeyboardEvent& event ){
//        // Save Point Cloud to PCD File when Pressed Space Key
//        if( event.getKeyCode() == VK_SPACE && event.keyDown() ){
//            boost::mutex::scoped_try_lock lock( mutex );
//            if(lock.owns_lock()){
//                //pcl::io::savePCDFile("cloud1", *cloud, false);
//                stringstream stream;
//                stream << "InputCloud" << fileSaved << ".pcd";
//                string filename = stream.str();
//                if(pcl::io::savePCDFile(filename, *cloud_scan, false )==0)
//                {
//                    fileSaved++;
//                    cout << "Saved" << filename << "." << endl;
//                }
//                else PCL_ERROR("Problem saving %s.\n",filename.c_str());
//            }
//        }
//    };

//    // Register Callback Function
//    m_visualizer->registerKeyboardCallback( keyboard_function );
      //grabber -> registerCallback(cloud_cb_);


    // Start Grabber
    grabber->start();
    clock.reset();

    cloudVector.clear();

    qDebug () << "started";

   // for(int i=0; i <= 10000; i++){

    //qDebug() << stopButton->isChecked();
    while( !stopButton->isChecked() )
    {
        // Update m_visualizer
        //m_visualizer->spinOnce();
        displayWidget_->update();

        qDebug () << "boom";

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
                //qDebug() << !m_visualizer->updatePointCloud( cloud, "cloud" ) <<endl;
                // Update Point Cloud
                if( !m_visualizer->updatePointCloud( cloud_scan, "cloud_scan" ) ){
                    //qDebug() << "inside if";
                    //m_visualizer->removePointCloud("cloud");
                    m_visualizer->addPointCloud( cloud_scan, "cloud_scan" );
                    m_visualizer->resetCameraViewpoint( "cloud_scan" );

                }
                qDebug()<<"about to save";
                //if(int(clock.getTimeSeconds()) == 1)
                //{

                    stringstream stream;
                    stream << "InputCloud" << fileSaved << ".pcd";
                    //qDebug() << fileSaved ;
                    filename = stream.str();
                    pcl::io::savePCDFileASCII(filename,*cloud_scan);
                    qDebug()<<"converting to qstring";
                    QString qFilename = QString::fromStdString(filename);
                    pcdNames.push_back(qFilename);
                    cloudList->addItem(qFilename);
                    int ret = pcl::io::loadPCDFile (filename, *cloud);
                    if (ret < 0) {
                    PCL_ERROR("Couldn't read file %s\n");
                    //return -1;
                    }
                    cloudVector.push_back(cloud);

                    //reset the cloud to read the next file
                    cloud.reset(new PointCloud);

                    qDebug() << "before saving";
                        //pcl::io::savePCDFileASCII(filename, *cloud_scan);
                        fileSaved++;
                        cout << "Saved" << filename << "." << endl;
                        qDebug() << "file saved";
                        save_one = true;


                        //else PCL_ERROR("Problem saving %s.\n",filename.c_str());
                        clock.reset();
                         QCoreApplication::processEvents();
                //}


            }
        }

        QCoreApplication::processEvents();

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

    }
    vectorSelectorFlag = 1;
}

void MyVTKWidget::on_stopButton_clicked()
{
    m_visualizer->removePointCloud("cloud_scan");
    m_visualizer->removePointCloud("cloud_load");
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
                    //m_visualizer->removePointCloud("cloud_load");
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

    //qDebug()<< "inside here";
    //grabber->stop();

}

void MyVTKWidget::on_backButton_pressed()
{
    buttonContainer->setCurrentIndex((buttonContainer->currentIndex() - 1));

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
    //point clouds for filtering
    PointCloud::Ptr ds_cloud (new PointCloud);
    PointCloud::Ptr pt_cloud (new PointCloud);
    PointCloud::Ptr ol_cloud (new PointCloud);

    //Threshold values for filters
    float LeafSize = 0.004;
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

        //Downsample the cloud
        //ds_cloud = myCfilter.VoxelGridDownSample(ol_cloud, LeafSize);

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

        //filteredCloudVector.push_back(myCfilter.PassThrough(*it, z1_, z2_, y1_, y2_, x1_, x2_));
        //cloudList->addItem();
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

void MyVTKWidget::on_meshButton_pressed()
{
    //Call the meshing function from regmesh class
    outMesh = myRegmesh.generateMesh(filteredCloudVector.back());

    //Get the number of triangles in the mesh
    triangleNumber = myRegmesh.getTriangles();

    cloudList->addItem("Final Mesh");

    vectorSelectorFlag = 3;

}
