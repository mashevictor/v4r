/****************************************************************************
** Meta object code from reading C++ file 'glviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/glviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'glviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_GLGraphicsView_t {
    QByteArrayData data[10];
    char stringdata0[109];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GLGraphicsView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GLGraphicsView_t qt_meta_stringdata_GLGraphicsView = {
    {
QT_MOC_LITERAL(0, 0, 14), // "GLGraphicsView"
QT_MOC_LITERAL(1, 15, 11), // "mouse_moved"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 12), // "QMouseEvent*"
QT_MOC_LITERAL(4, 41, 5), // "event"
QT_MOC_LITERAL(5, 47, 13), // "mouse_pressed"
QT_MOC_LITERAL(6, 61, 11), // "key_pressed"
QT_MOC_LITERAL(7, 73, 10), // "QKeyEvent*"
QT_MOC_LITERAL(8, 84, 11), // "wheel_event"
QT_MOC_LITERAL(9, 96, 12) // "QWheelEvent*"

    },
    "GLGraphicsView\0mouse_moved\0\0QMouseEvent*\0"
    "event\0mouse_pressed\0key_pressed\0"
    "QKeyEvent*\0wheel_event\0QWheelEvent*"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GLGraphicsView[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,
       6,    1,   40,    2, 0x06 /* Public */,
       8,    1,   43,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 7,    4,
    QMetaType::Void, 0x80000000 | 9,    4,

       0        // eod
};

void GLGraphicsView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GLGraphicsView *_t = static_cast<GLGraphicsView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->mouse_moved((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 1: _t->mouse_pressed((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 2: _t->key_pressed((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 3: _t->wheel_event((*reinterpret_cast< QWheelEvent*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (GLGraphicsView::*_t)(QMouseEvent * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGraphicsView::mouse_moved)) {
                *result = 0;
            }
        }
        {
            typedef void (GLGraphicsView::*_t)(QMouseEvent * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGraphicsView::mouse_pressed)) {
                *result = 1;
            }
        }
        {
            typedef void (GLGraphicsView::*_t)(QKeyEvent * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGraphicsView::key_pressed)) {
                *result = 2;
            }
        }
        {
            typedef void (GLGraphicsView::*_t)(QWheelEvent * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGraphicsView::wheel_event)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject GLGraphicsView::staticMetaObject = {
    { &QGraphicsView::staticMetaObject, qt_meta_stringdata_GLGraphicsView.data,
      qt_meta_data_GLGraphicsView,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GLGraphicsView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GLGraphicsView::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GLGraphicsView.stringdata0))
        return static_cast<void*>(const_cast< GLGraphicsView*>(this));
    return QGraphicsView::qt_metacast(_clname);
}

int GLGraphicsView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void GLGraphicsView::mouse_moved(QMouseEvent * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GLGraphicsView::mouse_pressed(QMouseEvent * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void GLGraphicsView::key_pressed(QKeyEvent * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void GLGraphicsView::wheel_event(QWheelEvent * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
struct qt_meta_stringdata_GLViewer_t {
    QByteArrayData data[35];
    char stringdata0[542];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GLViewer_t qt_meta_stringdata_GLViewer = {
    {
QT_MOC_LITERAL(0, 0, 8), // "GLViewer"
QT_MOC_LITERAL(1, 9, 13), // "segment_image"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 1), // "x"
QT_MOC_LITERAL(4, 26, 1), // "y"
QT_MOC_LITERAL(5, 28, 10), // "select_roi"
QT_MOC_LITERAL(6, 39, 4), // "draw"
QT_MOC_LITERAL(7, 44, 9), // "new_image"
QT_MOC_LITERAL(8, 54, 38), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(9, 93, 6), // "_cloud"
QT_MOC_LITERAL(10, 100, 19), // "cv::Mat_<cv::Vec3b>"
QT_MOC_LITERAL(11, 120, 6), // "_image"
QT_MOC_LITERAL(12, 127, 18), // "update_model_cloud"
QT_MOC_LITERAL(13, 146, 51), // "boost::shared_ptr<Sensor::Ali..."
QT_MOC_LITERAL(14, 198, 9), // "_oc_cloud"
QT_MOC_LITERAL(15, 208, 21), // "update_cam_trajectory"
QT_MOC_LITERAL(16, 230, 55), // "boost::shared_ptr<std::vector..."
QT_MOC_LITERAL(17, 286, 15), // "_cam_trajectory"
QT_MOC_LITERAL(18, 302, 20), // "update_visualization"
QT_MOC_LITERAL(19, 323, 18), // "update_boundingbox"
QT_MOC_LITERAL(20, 342, 28), // "std::vector<Eigen::Vector3f>"
QT_MOC_LITERAL(21, 371, 5), // "edges"
QT_MOC_LITERAL(22, 377, 15), // "Eigen::Matrix4f"
QT_MOC_LITERAL(23, 393, 4), // "pose"
QT_MOC_LITERAL(24, 398, 18), // "cam_params_changed"
QT_MOC_LITERAL(25, 417, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(26, 437, 11), // "_cam_params"
QT_MOC_LITERAL(27, 449, 11), // "mouse_moved"
QT_MOC_LITERAL(28, 461, 12), // "QMouseEvent*"
QT_MOC_LITERAL(29, 474, 5), // "event"
QT_MOC_LITERAL(30, 480, 13), // "mouse_pressed"
QT_MOC_LITERAL(31, 494, 11), // "key_pressed"
QT_MOC_LITERAL(32, 506, 10), // "QKeyEvent*"
QT_MOC_LITERAL(33, 517, 11), // "wheel_event"
QT_MOC_LITERAL(34, 529, 12) // "QWheelEvent*"

    },
    "GLViewer\0segment_image\0\0x\0y\0select_roi\0"
    "draw\0new_image\0pcl::PointCloud<pcl::PointXYZRGB>::Ptr\0"
    "_cloud\0cv::Mat_<cv::Vec3b>\0_image\0"
    "update_model_cloud\0"
    "boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>\0"
    "_oc_cloud\0update_cam_trajectory\0"
    "boost::shared_ptr<std::vector<Sensor::CameraLocation> >\0"
    "_cam_trajectory\0update_visualization\0"
    "update_boundingbox\0std::vector<Eigen::Vector3f>\0"
    "edges\0Eigen::Matrix4f\0pose\0"
    "cam_params_changed\0RGBDCameraParameter\0"
    "_cam_params\0mouse_moved\0QMouseEvent*\0"
    "event\0mouse_pressed\0key_pressed\0"
    "QKeyEvent*\0wheel_event\0QWheelEvent*"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GLViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   79,    2, 0x06 /* Public */,
       5,    2,   84,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   89,    2, 0x0a /* Public */,
       7,    2,   90,    2, 0x0a /* Public */,
      12,    1,   95,    2, 0x0a /* Public */,
      15,    1,   98,    2, 0x0a /* Public */,
      18,    0,  101,    2, 0x0a /* Public */,
      19,    2,  102,    2, 0x0a /* Public */,
      24,    1,  107,    2, 0x0a /* Public */,
      27,    1,  110,    2, 0x0a /* Public */,
      30,    1,  113,    2, 0x0a /* Public */,
      31,    1,  116,    2, 0x0a /* Public */,
      33,    1,  119,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 10,    9,   11,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 20, 0x80000000 | 22,   21,   23,
    QMetaType::Void, 0x80000000 | 25,   26,
    QMetaType::Void, 0x80000000 | 28,   29,
    QMetaType::Void, 0x80000000 | 28,   29,
    QMetaType::Void, 0x80000000 | 32,   29,
    QMetaType::Void, 0x80000000 | 34,   29,

       0        // eod
};

void GLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GLViewer *_t = static_cast<GLViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->segment_image((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->select_roi((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->draw(); break;
        case 3: _t->new_image((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGB>::Ptr(*)>(_a[1])),(*reinterpret_cast< const cv::Mat_<cv::Vec3b>(*)>(_a[2]))); break;
        case 4: _t->update_model_cloud((*reinterpret_cast< const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>(*)>(_a[1]))); break;
        case 5: _t->update_cam_trajectory((*reinterpret_cast< const boost::shared_ptr<std::vector<Sensor::CameraLocation> >(*)>(_a[1]))); break;
        case 6: _t->update_visualization(); break;
        case 7: _t->update_boundingbox((*reinterpret_cast< const std::vector<Eigen::Vector3f>(*)>(_a[1])),(*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[2]))); break;
        case 8: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 9: _t->mouse_moved((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 10: _t->mouse_pressed((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 11: _t->key_pressed((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 12: _t->wheel_event((*reinterpret_cast< QWheelEvent*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (GLViewer::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLViewer::segment_image)) {
                *result = 0;
            }
        }
        {
            typedef void (GLViewer::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLViewer::select_roi)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject GLViewer::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_GLViewer.data,
      qt_meta_data_GLViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GLViewer.stringdata0))
        return static_cast<void*>(const_cast< GLViewer*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int GLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void GLViewer::segment_image(int _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GLViewer::select_roi(int _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
