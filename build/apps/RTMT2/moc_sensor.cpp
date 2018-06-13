/****************************************************************************
** Meta object code from reading C++ file 'sensor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT2/sensor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sensor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Sensor_t {
    QByteArrayData data[33];
    char stringdata0[513];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Sensor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Sensor_t qt_meta_stringdata_Sensor = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Sensor"
QT_MOC_LITERAL(1, 7, 9), // "new_image"
QT_MOC_LITERAL(2, 17, 0), // ""
QT_MOC_LITERAL(3, 18, 38), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(4, 57, 6), // "_cloud"
QT_MOC_LITERAL(5, 64, 19), // "cv::Mat_<cv::Vec3b>"
QT_MOC_LITERAL(6, 84, 5), // "image"
QT_MOC_LITERAL(7, 90, 8), // "new_pose"
QT_MOC_LITERAL(8, 99, 15), // "Eigen::Matrix4f"
QT_MOC_LITERAL(9, 115, 5), // "_pose"
QT_MOC_LITERAL(10, 121, 18), // "update_model_cloud"
QT_MOC_LITERAL(11, 140, 44), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(12, 185, 21), // "update_cam_trajectory"
QT_MOC_LITERAL(13, 207, 55), // "boost::shared_ptr<std::vector..."
QT_MOC_LITERAL(14, 263, 15), // "_cam_trajectory"
QT_MOC_LITERAL(15, 279, 20), // "update_visualization"
QT_MOC_LITERAL(16, 300, 11), // "printStatus"
QT_MOC_LITERAL(17, 312, 11), // "std::string"
QT_MOC_LITERAL(18, 324, 4), // "_txt"
QT_MOC_LITERAL(19, 329, 18), // "update_boundingbox"
QT_MOC_LITERAL(20, 348, 28), // "std::vector<Eigen::Vector3f>"
QT_MOC_LITERAL(21, 377, 5), // "edges"
QT_MOC_LITERAL(22, 383, 4), // "pose"
QT_MOC_LITERAL(23, 388, 18), // "cam_params_changed"
QT_MOC_LITERAL(24, 407, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(25, 427, 11), // "_cam_params"
QT_MOC_LITERAL(26, 439, 10), // "select_roi"
QT_MOC_LITERAL(27, 450, 1), // "x"
QT_MOC_LITERAL(28, 452, 1), // "y"
QT_MOC_LITERAL(29, 454, 14), // "set_roi_params"
QT_MOC_LITERAL(30, 469, 14), // "_bbox_scale_xy"
QT_MOC_LITERAL(31, 484, 18), // "_bbox_scale_height"
QT_MOC_LITERAL(32, 503, 9) // "_seg_offs"

    },
    "Sensor\0new_image\0\0"
    "pcl::PointCloud<pcl::PointXYZRGB>::Ptr\0"
    "_cloud\0cv::Mat_<cv::Vec3b>\0image\0"
    "new_pose\0Eigen::Matrix4f\0_pose\0"
    "update_model_cloud\0"
    "pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr\0"
    "update_cam_trajectory\0"
    "boost::shared_ptr<std::vector<Sensor::CameraLocation> >\0"
    "_cam_trajectory\0update_visualization\0"
    "printStatus\0std::string\0_txt\0"
    "update_boundingbox\0std::vector<Eigen::Vector3f>\0"
    "edges\0pose\0cam_params_changed\0"
    "RGBDCameraParameter\0_cam_params\0"
    "select_roi\0x\0y\0set_roi_params\0"
    "_bbox_scale_xy\0_bbox_scale_height\0"
    "_seg_offs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Sensor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   64,    2, 0x06 /* Public */,
       7,    1,   69,    2, 0x06 /* Public */,
      10,    1,   72,    2, 0x06 /* Public */,
      12,    1,   75,    2, 0x06 /* Public */,
      15,    0,   78,    2, 0x06 /* Public */,
      16,    1,   79,    2, 0x06 /* Public */,
      19,    2,   82,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      23,    1,   87,    2, 0x0a /* Public */,
      26,    2,   90,    2, 0x0a /* Public */,
      29,    3,   95,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 11,    4,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 20, 0x80000000 | 8,   21,   22,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   27,   28,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   30,   31,   32,

       0        // eod
};

void Sensor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Sensor *_t = static_cast<Sensor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->new_image((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGB>::Ptr(*)>(_a[1])),(*reinterpret_cast< const cv::Mat_<cv::Vec3b>(*)>(_a[2]))); break;
        case 1: _t->new_pose((*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[1]))); break;
        case 2: _t->update_model_cloud((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(*)>(_a[1]))); break;
        case 3: _t->update_cam_trajectory((*reinterpret_cast< const boost::shared_ptr<std::vector<Sensor::CameraLocation> >(*)>(_a[1]))); break;
        case 4: _t->update_visualization(); break;
        case 5: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 6: _t->update_boundingbox((*reinterpret_cast< const std::vector<Eigen::Vector3f>(*)>(_a[1])),(*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[2]))); break;
        case 7: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 8: _t->select_roi((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 9: _t->set_roi_params((*reinterpret_cast< const double(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< const double(*)>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Sensor::*_t)(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & , const cv::Mat_<cv::Vec3b> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::new_image)) {
                *result = 0;
            }
        }
        {
            typedef void (Sensor::*_t)(const Eigen::Matrix4f & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::new_pose)) {
                *result = 1;
            }
        }
        {
            typedef void (Sensor::*_t)(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::update_model_cloud)) {
                *result = 2;
            }
        }
        {
            typedef void (Sensor::*_t)(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::update_cam_trajectory)) {
                *result = 3;
            }
        }
        {
            typedef void (Sensor::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::update_visualization)) {
                *result = 4;
            }
        }
        {
            typedef void (Sensor::*_t)(const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::printStatus)) {
                *result = 5;
            }
        }
        {
            typedef void (Sensor::*_t)(const std::vector<Eigen::Vector3f> & , const Eigen::Matrix4f & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::update_boundingbox)) {
                *result = 6;
            }
        }
    }
}

const QMetaObject Sensor::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Sensor.data,
      qt_meta_data_Sensor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Sensor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Sensor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Sensor.stringdata0))
        return static_cast<void*>(const_cast< Sensor*>(this));
    return QThread::qt_metacast(_clname);
}

int Sensor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void Sensor::new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _t1, const cv::Mat_<cv::Vec3b> & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Sensor::new_pose(const Eigen::Matrix4f & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Sensor::update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Sensor::update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Sensor::update_visualization()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void Sensor::printStatus(const std::string & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Sensor::update_boundingbox(const std::vector<Eigen::Vector3f> & _t1, const Eigen::Matrix4f & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_END_MOC_NAMESPACE
