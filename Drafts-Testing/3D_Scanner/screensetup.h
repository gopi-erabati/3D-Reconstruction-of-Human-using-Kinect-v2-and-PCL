#ifndef SCREENSETUP_H
#define SCREENSETUP_H

#include <QSize>
#include <QObject>
#include <QScreen>
#include <QGuiApplication>

class ScreenSetup : public QObject
{
    Q_OBJECT
    //Q_PROPERTY(QSize size READ size)
public:
    QSize size()
    {
        QSize size = qApp->screens()[0]->size();
        return size;
    }
};

#endif // SCREENSETUP_H
