#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include "myvtkwidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setupEverything();

private:
    Ui::MainWindow *ui;
    QWidget *centralWidget;
    MyVTKWidget myVTKWidget;
};

#endif // MAINWINDOW_H
