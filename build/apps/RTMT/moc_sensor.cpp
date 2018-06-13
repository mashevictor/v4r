/****************************************************************************
** Meta object code from reading C++ file 'sensor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/sensor.h"
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
    QByteArrayData data[47];
    char stringdata0[754];
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
QT_MOC_LITERAL(11, 140, 51), // "boost::shared_ptr<Sensor::Ali..."
QT_MOC_LITERAL(12, 192, 9), // "_oc_cloud"
QT_MOC_LITERAL(13, 202, 21), // "update_cam_trajectory"
QT_MOC_LITERAL(14, 224, 55), // "boost::shared_ptr<std::vector..."
QT_MOC_LITERAL(15, 280, 15), // "_cam_trajectory"
QT_MOC_LITERAL(16, 296, 20), // "update_visualization"
QT_MOC_LITERAL(17, 317, 11), // "printStatus"
QT_MOC_LITERAL(18, 329, 11), // "std::string"
QT_MOC_LITERAL(19, 341, 4), // "_txt"
QT_MOC_LITERAL(20, 346, 23), // "finishedOptimizeCameras"
QT_MOC_LITERAL(21, 370, 11), // "num_cameras"
QT_MOC_LITERAL(22, 382, 18), // "update_boundingbox"
QT_MOC_LITERAL(23, 401, 28), // "std::vector<Eigen::Vector3f>"
QT_MOC_LITERAL(24, 430, 5), // "edges"
QT_MOC_LITERAL(25, 436, 4), // "pose"
QT_MOC_LITERAL(26, 441, 7), // "set_roi"
QT_MOC_LITERAL(27, 449, 15), // "Eigen::Vector3f"
QT_MOC_LITERAL(28, 465, 7), // "_bb_min"
QT_MOC_LITERAL(29, 473, 7), // "_bb_max"
QT_MOC_LITERAL(30, 481, 9), // "_roi_pose"
QT_MOC_LITERAL(31, 491, 18), // "cam_params_changed"
QT_MOC_LITERAL(32, 510, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(33, 530, 11), // "_cam_params"
QT_MOC_LITERAL(34, 542, 26), // "cam_tracker_params_changed"
QT_MOC_LITERAL(35, 569, 22), // "CamaraTrackerParameter"
QT_MOC_LITERAL(36, 592, 19), // "_cam_tracker_params"
QT_MOC_LITERAL(37, 612, 35), // "bundle_adjustment_parameter_c..."
QT_MOC_LITERAL(38, 648, 25), // "BundleAdjustmentParameter"
QT_MOC_LITERAL(39, 674, 5), // "param"
QT_MOC_LITERAL(40, 680, 10), // "select_roi"
QT_MOC_LITERAL(41, 691, 1), // "x"
QT_MOC_LITERAL(42, 693, 1), // "y"
QT_MOC_LITERAL(43, 695, 14), // "set_roi_params"
QT_MOC_LITERAL(44, 710, 14), // "_bbox_scale_xy"
QT_MOC_LITERAL(45, 725, 18), // "_bbox_scale_height"
QT_MOC_LITERAL(46, 744, 9) // "_seg_offs"

    },
    "Sensor\0new_image\0\0"
    "pcl::PointCloud<pcl::PointXYZRGB>::Ptr\0"
    "_cloud\0cv::Mat_<cv::Vec3b>\0image\0"
    "new_pose\0Eigen::Matrix4f\0_pose\0"
    "update_model_cloud\0"
    "boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>\0"
    "_oc_cloud\0update_cam_trajectory\0"
    "boost::shared_ptr<std::vector<Sensor::CameraLocation> >\0"
    "_cam_trajectory\0update_visualization\0"
    "printStatus\0std::string\0_txt\0"
    "finishedOptimizeCameras\0num_cameras\0"
    "update_boundingbox\0std::vector<Eigen::Vector3f>\0"
    "edges\0pose\0set_roi\0Eigen::Vector3f\0"
    "_bb_min\0_bb_max\0_roi_pose\0cam_params_changed\0"
    "RGBDCameraParameter\0_cam_params\0"
    "cam_tracker_params_changed\0"
    "CamaraTrackerParameter\0_cam_tracker_params\0"
    "bundle_adjustment_parameter_changed\0"
    "BundleAdjustmentParameter\0param\0"
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
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   84,    2, 0x06 /* Public */,
       7,    1,   89,    2, 0x06 /* Public */,
      10,    1,   92,    2, 0x06 /* Public */,
      13,    1,   95,    2, 0x06 /* Public */,
      16,    0,   98,    2, 0x06 /* Public */,
      17,    1,   99,    2, 0x06 /* Public */,
      20,    1,  102,    2, 0x06 /* Public */,
      22,    2,  105,    2, 0x06 /* Public */,
      26,    3,  110,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      31,    1,  117,    2, 0x0a /* Public */,
      34,    1,  120,    2, 0x0a /* Public */,
      37,    1,  123,    2, 0x0a /* Public */,
      40,    2,  126,    2, 0x0a /* Public */,
      43,    3,  131,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void, QMetaType::Int,   21,
    QMetaType::Void, 0x80000000 | 23, 0x80000000 | 8,   24,   25,
    QMetaType::Void, 0x80000000 | 27, 0x80000000 | 27, 0x80000000 | 8,   28,   29,   30,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 32,   33,
    QMetaType::Void, 0x80000000 | 35,   36,
    QMetaType::Void, 0x80000000 | 38,   39,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   41,   42,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   44,   45,   46,

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
        case 2: _t->update_model_cloud((*reinterpret_cast< const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>(*)>(_a[1]))); break;
        case 3: _t->update_cam_trajectory((*reinterpret_cast< const boost::shared_ptr<std::vector<Sensor::CameraLocation> >(*)>(_a[1]))); break;
        case 4: _t->update_visualization(); break;
        case 5: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 6: _t->finishedOptimizeCameras((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->update_boundingbox((*reinterpret_cast< const std::vector<Eigen::Vector3f>(*)>(_a[1])),(*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[2]))); break;
        case 8: _t->set_roi((*reinterpret_cast< const Eigen::Vector3f(*)>(_a[1])),(*reinterpret_cast< const Eigen::Vector3f(*)>(_a[2])),(*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[3]))); break;
        case 9: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 10: _t->cam_tracker_params_changed((*reinterpret_cast< const CamaraTrackerParameter(*)>(_a[1]))); break;
        case 11: _t->bundle_adjustment_parameter_changed((*reinterpret_cast< const BundleAdjustmentParameter(*)>(_a[1]))); break;
        case 12: _t->select_roi((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 13: _t->set_roi_params((*reinterpret_cast< const double(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< const double(*)>(_a[3]))); break;
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
            typedef void (Sensor::*_t)(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & );
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
            typedef void (Sensor::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::finishedOptimizeCameras)) {
                *result = 6;
            }
        }
        {
            typedef void (Sensor::*_t)(const std::vector<Eigen::Vector3f> & , const Eigen::Matrix4f & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::update_boundingbox)) {
                *result = 7;
            }
        }
        {
            typedef void (Sensor::*_t)(const Eigen::Vector3f & , const Eigen::Vector3f & , const Eigen::Matrix4f & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Sensor::set_roi)) {
                *result = 8;
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
void Sensor::update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & _t1)
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
void Sensor::finishedOptimizeCameras(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Sensor::update_boundingbox(const std::vector<Eigen::Vector3f> & _t1, const Eigen::Matrix4f & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Sensor::set_roi(const Eigen::Vector3f & _t1, const Eigen::Vector3f & _t2, const Eigen::Matrix4f & _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_END_MOC_NAMESPACE
