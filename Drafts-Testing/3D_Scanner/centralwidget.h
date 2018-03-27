#ifndef CENTRALWIDGET_H
#define CENTRALWIDGET_H

#include<QWidget>
#include<QPushButton>
#include<QLabel>
#include<QVTKWidget.h>
#include"screensetup.h"
#include<QSize>
#include<QVBoxLayout>
#include<QHBoxLayout>
#include "toolsandoptions.h"
#include <QWidget>
#include "cloudDisplay.h"

class CentralWidget : public QWidget
{
    Q_OBJECT

private:

    //------widgets---------->
    QWidget *centralWidget_;
    QWidget *sidePanelWidget_;
    QPushButton *pushButton_;

    //Layouts
    QVBoxLayout *sidePanelLayout_;
    QHBoxLayout *mainLayout_;

    //objects
    ToolsAndOptions toolsAndOptions;
    CloudDisplay cloudDisplay;

public:

    //Consrtuctor
    //CentralWidget();
    CentralWidget(QWidget *parent = 0);

    // Rendering Window
    QVTKWidget *displayWidget;

    // Function to setup the central widget
    QWidget* setupCentralWidget(int &x, int &y);

};

#endif // CENTRALWIDGET_H
