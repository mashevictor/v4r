/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT2/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[27];
    char stringdata0[521];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 9), // "set_image"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 3), // "idx"
QT_MOC_LITERAL(4, 26, 30), // "on_actionPreferences_triggered"
QT_MOC_LITERAL(5, 57, 23), // "on_actionExit_triggered"
QT_MOC_LITERAL(6, 81, 19), // "on_CamStart_clicked"
QT_MOC_LITERAL(7, 101, 18), // "on_CamStop_clicked"
QT_MOC_LITERAL(8, 120, 23), // "on_TrackerStart_clicked"
QT_MOC_LITERAL(9, 144, 22), // "on_TrackerStop_clicked"
QT_MOC_LITERAL(10, 167, 29), // "on_CreateAndSaveModel_clicked"
QT_MOC_LITERAL(11, 197, 16), // "on_Reset_clicked"
QT_MOC_LITERAL(12, 214, 20), // "on_ResetView_clicked"
QT_MOC_LITERAL(13, 235, 20), // "on_ShowImage_clicked"
QT_MOC_LITERAL(14, 256, 22), // "on_ShowCameras_clicked"
QT_MOC_LITERAL(15, 279, 25), // "on_ShowPointCloud_clicked"
QT_MOC_LITERAL(16, 305, 26), // "on_ShowObjectModel_clicked"
QT_MOC_LITERAL(17, 332, 20), // "on_imForward_clicked"
QT_MOC_LITERAL(18, 353, 21), // "on_imBackward_clicked"
QT_MOC_LITERAL(19, 375, 17), // "finishedModelling"
QT_MOC_LITERAL(20, 393, 24), // "on_ShowDepthMask_clicked"
QT_MOC_LITERAL(21, 418, 17), // "on_setROI_clicked"
QT_MOC_LITERAL(22, 436, 19), // "on_ResetROI_clicked"
QT_MOC_LITERAL(23, 456, 11), // "printStatus"
QT_MOC_LITERAL(24, 468, 11), // "std::string"
QT_MOC_LITERAL(25, 480, 4), // "_txt"
QT_MOC_LITERAL(26, 485, 35) // "vignetting_calibration_file_c..."

    },
    "MainWindow\0set_image\0\0idx\0"
    "on_actionPreferences_triggered\0"
    "on_actionExit_triggered\0on_CamStart_clicked\0"
    "on_CamStop_clicked\0on_TrackerStart_clicked\0"
    "on_TrackerStop_clicked\0"
    "on_CreateAndSaveModel_clicked\0"
    "on_Reset_clicked\0on_ResetView_clicked\0"
    "on_ShowImage_clicked\0on_ShowCameras_clicked\0"
    "on_ShowPointCloud_clicked\0"
    "on_ShowObjectModel_clicked\0"
    "on_imForward_clicked\0on_imBackward_clicked\0"
    "finishedModelling\0on_ShowDepthMask_clicked\0"
    "on_setROI_clicked\0on_ResetROI_clicked\0"
    "printStatus\0std::string\0_txt\0"
    "vignetting_calibration_file_changed"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      22,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  124,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,  127,    2, 0x08 /* Private */,
       5,    0,  128,    2, 0x08 /* Private */,
       6,    0,  129,    2, 0x08 /* Private */,
       7,    0,  130,    2, 0x08 /* Private */,
       8,    0,  131,    2, 0x08 /* Private */,
       9,    0,  132,    2, 0x08 /* Private */,
      10,    0,  133,    2, 0x08 /* Private */,
      11,    0,  134,    2, 0x08 /* Private */,
      12,    0,  135,    2, 0x08 /* Private */,
      13,    0,  136,    2, 0x08 /* Private */,
      14,    0,  137,    2, 0x08 /* Private */,
      15,    0,  138,    2, 0x08 /* Private */,
      16,    0,  139,    2, 0x08 /* Private */,
      17,    0,  140,    2, 0x08 /* Private */,
      18,    0,  141,    2, 0x08 /* Private */,
      19,    0,  142,    2, 0x08 /* Private */,
      20,    0,  143,    2, 0x08 /* Private */,
      21,    0,  144,    2, 0x08 /* Private */,
      22,    0,  145,    2, 0x08 /* Private */,
      23,    1,  146,    2, 0x0a /* Public */,
      26,    0,  149,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->set_image((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->on_actionPreferences_triggered(); break;
        case 2: _t->on_actionExit_triggered(); break;
        case 3: _t->on_CamStart_clicked(); break;
        case 4: _t->on_CamStop_clicked(); break;
        case 5: _t->on_TrackerStart_clicked(); break;
        case 6: _t->on_TrackerStop_clicked(); break;
        case 7: _t->on_CreateAndSaveModel_clicked(); break;
        case 8: _t->on_Reset_clicked(); break;
        case 9: _t->on_ResetView_clicked(); break;
        case 10: _t->on_ShowImage_clicked(); break;
        case 11: _t->on_ShowCameras_clicked(); break;
        case 12: _t->on_ShowPointCloud_clicked(); break;
        case 13: _t->on_ShowObjectModel_clicked(); break;
        case 14: _t->on_imForward_clicked(); break;
        case 15: _t->on_imBackward_clicked(); break;
        case 16: _t->finishedModelling(); break;
        case 17: _t->on_ShowDepthMask_clicked(); break;
        case 18: _t->on_setROI_clicked(); break;
        case 19: _t->on_ResetROI_clicked(); break;
        case 20: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 21: _t->vignetting_calibration_file_changed(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::set_image)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 22)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 22;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::set_image(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
