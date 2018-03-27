#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    centralWidget = new QWidget(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::setupEverything()
{
    //get the built widget and set it as the central widget
    centralWidget = myVTKWidget.getCentralWidget();
    setCentralWidget(centralWidget);
}
