 #include "centralwidget.h"

CentralWidget::CentralWidget(QWidget *parent) : QWidget(parent)
{
    //Initialize widgets
    centralWidget_ = new QWidget(this);
    sidePanelWidget_ = new QWidget(this);
    //Initialize Layouts
    sidePanelLayout_ = new QVBoxLayout;
    mainLayout_ = new QHBoxLayout;
    //Initialize QVTKWidget
    displayWidget = new QVTKWidget;
    displayWidget = cloudDisplay.getVTKWidget();
    //Intialize pushButton
    pushButton_ = new QPushButton(this);
}

QWidget* CentralWidget::setupCentralWidget(int &x, int &y)
{
    //displayWidget = cloudDisplay.getVTKWidget();
    //Set the size of the rendering window to 2/3rd the screen
    displayWidget->setFixedSize(2*(x/3),y);
    //Set the position for the display window
    displayWidget->setGeometry(0,0,2*(x/3),y);
    //Fix the size and geometry for the sidePanelWidget_
    sidePanelWidget_->setFixedSize(x/3,y);
    sidePanelWidget_->setGeometry(2*(x/3),0,x,y);
    //Get the start button
    pushButton_ = toolsAndOptions.getStartButton();
    //set up the sidePanelWidget_
    sidePanelLayout_->addWidget(pushButton_);
    sidePanelWidget_->setLayout(sidePanelLayout_);
    //Set up the central Widget
    mainLayout_->addWidget(displayWidget);
    mainLayout_->addWidget(sidePanelWidget_);
    centralWidget_->setLayout(mainLayout_);

    return centralWidget_;
}
