#include "toolsandoptions.h"

ToolsAndOptions::ToolsAndOptions(QWidget *parent) : QWidget (parent)
{
    //Initialize startButton
    startButton_ = new QPushButton("Start", this);
    //Connect the pushButton pressed signal
    QObject::connect(startButton_, SIGNAL(pressed()), this, SLOT(on_startButton_pressed()));
}

QPushButton* ToolsAndOptions::getStartButton()
{
    return startButton_;
}

void ToolsAndOptions::on_startButton_pressed()
{
    path = "../bunny1.pcd";
    cloud = cloudReg.getCloud(path);
    disp.displayCloud(cloud);
}
