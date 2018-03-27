/****************************************************************************
** Meta object code from reading C++ file 'myvtkwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../myvtkwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'myvtkwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MyVTKWidget_t {
    QByteArrayData data[24];
    char stringdata0[362];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyVTKWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyVTKWidget_t qt_meta_stringdata_MyVTKWidget = {
    {
QT_MOC_LITERAL(0, 0, 11), // "MyVTKWidget"
QT_MOC_LITERAL(1, 12, 21), // "on_readButton_pressed"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 21), // "on_scanButton_clicked"
QT_MOC_LITERAL(4, 57, 21), // "on_stopButton_clicked"
QT_MOC_LITERAL(5, 79, 24), // "on_cloudListItem_clicked"
QT_MOC_LITERAL(6, 104, 16), // "QListWidgetItem*"
QT_MOC_LITERAL(7, 121, 4), // "item"
QT_MOC_LITERAL(8, 126, 21), // "on_loadKinect_pressed"
QT_MOC_LITERAL(9, 148, 21), // "on_backButton_pressed"
QT_MOC_LITERAL(10, 170, 25), // "on_registerButton_pressed"
QT_MOC_LITERAL(11, 196, 21), // "on_meshButton_pressed"
QT_MOC_LITERAL(12, 218, 20), // "xSlider1ValueChanged"
QT_MOC_LITERAL(13, 239, 2), // "x1"
QT_MOC_LITERAL(14, 242, 20), // "xSlider2ValueChanged"
QT_MOC_LITERAL(15, 263, 2), // "x2"
QT_MOC_LITERAL(16, 266, 20), // "ySlider1ValueChanged"
QT_MOC_LITERAL(17, 287, 2), // "y1"
QT_MOC_LITERAL(18, 290, 20), // "ySlider2ValueChanged"
QT_MOC_LITERAL(19, 311, 2), // "y2"
QT_MOC_LITERAL(20, 314, 20), // "zSlider1ValueChanged"
QT_MOC_LITERAL(21, 335, 2), // "z1"
QT_MOC_LITERAL(22, 338, 20), // "zSlider2ValueChanged"
QT_MOC_LITERAL(23, 359, 2) // "z2"

    },
    "MyVTKWidget\0on_readButton_pressed\0\0"
    "on_scanButton_clicked\0on_stopButton_clicked\0"
    "on_cloudListItem_clicked\0QListWidgetItem*\0"
    "item\0on_loadKinect_pressed\0"
    "on_backButton_pressed\0on_registerButton_pressed\0"
    "on_meshButton_pressed\0xSlider1ValueChanged\0"
    "x1\0xSlider2ValueChanged\0x2\0"
    "ySlider1ValueChanged\0y1\0ySlider2ValueChanged\0"
    "y2\0zSlider1ValueChanged\0z1\0"
    "zSlider2ValueChanged\0z2"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyVTKWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   84,    2, 0x0a /* Public */,
       3,    0,   85,    2, 0x0a /* Public */,
       4,    0,   86,    2, 0x0a /* Public */,
       5,    1,   87,    2, 0x0a /* Public */,
       8,    0,   90,    2, 0x0a /* Public */,
       9,    0,   91,    2, 0x0a /* Public */,
      10,    0,   92,    2, 0x0a /* Public */,
      11,    0,   93,    2, 0x0a /* Public */,
      12,    1,   94,    2, 0x0a /* Public */,
      14,    1,   97,    2, 0x0a /* Public */,
      16,    1,  100,    2, 0x0a /* Public */,
      18,    1,  103,    2, 0x0a /* Public */,
      20,    1,  106,    2, 0x0a /* Public */,
      22,    1,  109,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void, QMetaType::Int,   21,
    QMetaType::Void, QMetaType::Int,   23,

       0        // eod
};

void MyVTKWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MyVTKWidget *_t = static_cast<MyVTKWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_readButton_pressed(); break;
        case 1: _t->on_scanButton_clicked(); break;
        case 2: _t->on_stopButton_clicked(); break;
        case 3: _t->on_cloudListItem_clicked((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        case 4: _t->on_loadKinect_pressed(); break;
        case 5: _t->on_backButton_pressed(); break;
        case 6: _t->on_registerButton_pressed(); break;
        case 7: _t->on_meshButton_pressed(); break;
        case 8: _t->xSlider1ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->xSlider2ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->ySlider1ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->ySlider2ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->zSlider1ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->zSlider2ValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject MyVTKWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MyVTKWidget.data,
      qt_meta_data_MyVTKWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MyVTKWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyVTKWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MyVTKWidget.stringdata0))
        return static_cast<void*>(const_cast< MyVTKWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int MyVTKWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
