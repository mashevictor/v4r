/****************************************************************************
** Meta object code from reading C++ file 'ObjectSegmentation.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/ObjectSegmentation.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ObjectSegmentation.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ObjectSegmentation_t {
    QByteArrayData data[41];
    char stringdata0[645];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ObjectSegmentation_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ObjectSegmentation_t qt_meta_stringdata_ObjectSegmentation = {
    {
QT_MOC_LITERAL(0, 0, 18), // "ObjectSegmentation"
QT_MOC_LITERAL(1, 19, 9), // "new_image"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 43), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(4, 74, 6), // "_cloud"
QT_MOC_LITERAL(5, 81, 19), // "cv::Mat_<cv::Vec3b>"
QT_MOC_LITERAL(6, 101, 5), // "image"
QT_MOC_LITERAL(7, 107, 18), // "update_model_cloud"
QT_MOC_LITERAL(8, 126, 51), // "boost::shared_ptr<Sensor::Ali..."
QT_MOC_LITERAL(9, 178, 9), // "_oc_cloud"
QT_MOC_LITERAL(10, 188, 11), // "printStatus"
QT_MOC_LITERAL(11, 200, 11), // "std::string"
QT_MOC_LITERAL(12, 212, 4), // "_txt"
QT_MOC_LITERAL(13, 217, 20), // "update_visualization"
QT_MOC_LITERAL(14, 238, 25), // "set_object_base_transform"
QT_MOC_LITERAL(15, 264, 15), // "Eigen::Matrix4f"
QT_MOC_LITERAL(16, 280, 22), // "_object_base_transform"
QT_MOC_LITERAL(17, 303, 26), // "finishedObjectSegmentation"
QT_MOC_LITERAL(18, 330, 13), // "segment_image"
QT_MOC_LITERAL(19, 344, 1), // "x"
QT_MOC_LITERAL(20, 346, 1), // "y"
QT_MOC_LITERAL(21, 348, 9), // "set_image"
QT_MOC_LITERAL(22, 358, 3), // "idx"
QT_MOC_LITERAL(23, 362, 18), // "cam_params_changed"
QT_MOC_LITERAL(24, 381, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(25, 401, 11), // "_cam_params"
QT_MOC_LITERAL(26, 413, 30), // "segmentation_parameter_changed"
QT_MOC_LITERAL(27, 444, 21), // "SegmentationParameter"
QT_MOC_LITERAL(28, 466, 5), // "param"
QT_MOC_LITERAL(29, 472, 34), // "object_modelling_parameter_ch..."
QT_MOC_LITERAL(30, 507, 15), // "ObjectModelling"
QT_MOC_LITERAL(31, 523, 7), // "set_roi"
QT_MOC_LITERAL(32, 531, 15), // "Eigen::Vector3f"
QT_MOC_LITERAL(33, 547, 7), // "_bb_min"
QT_MOC_LITERAL(34, 555, 7), // "_bb_max"
QT_MOC_LITERAL(35, 563, 9), // "_roi_pose"
QT_MOC_LITERAL(36, 573, 23), // "set_segmentation_params"
QT_MOC_LITERAL(37, 597, 12), // "use_roi_segm"
QT_MOC_LITERAL(38, 610, 4), // "offs"
QT_MOC_LITERAL(39, 615, 13), // "_use_dense_mv"
QT_MOC_LITERAL(40, 629, 15) // "_edge_radius_px"

    },
    "ObjectSegmentation\0new_image\0\0"
    "pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr\0"
    "_cloud\0cv::Mat_<cv::Vec3b>\0image\0"
    "update_model_cloud\0"
    "boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>\0"
    "_oc_cloud\0printStatus\0std::string\0"
    "_txt\0update_visualization\0"
    "set_object_base_transform\0Eigen::Matrix4f\0"
    "_object_base_transform\0"
    "finishedObjectSegmentation\0segment_image\0"
    "x\0y\0set_image\0idx\0cam_params_changed\0"
    "RGBDCameraParameter\0_cam_params\0"
    "segmentation_parameter_changed\0"
    "SegmentationParameter\0param\0"
    "object_modelling_parameter_changed\0"
    "ObjectModelling\0set_roi\0Eigen::Vector3f\0"
    "_bb_min\0_bb_max\0_roi_pose\0"
    "set_segmentation_params\0use_roi_segm\0"
    "offs\0_use_dense_mv\0_edge_radius_px"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ObjectSegmentation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   79,    2, 0x06 /* Public */,
       7,    1,   84,    2, 0x06 /* Public */,
      10,    1,   87,    2, 0x06 /* Public */,
      13,    0,   90,    2, 0x06 /* Public */,
      14,    1,   91,    2, 0x06 /* Public */,
      17,    0,   94,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      18,    2,   95,    2, 0x0a /* Public */,
      21,    1,  100,    2, 0x0a /* Public */,
      23,    1,  103,    2, 0x0a /* Public */,
      26,    1,  106,    2, 0x0a /* Public */,
      29,    1,  109,    2, 0x0a /* Public */,
      31,    3,  112,    2, 0x0a /* Public */,
      36,    4,  119,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   19,   20,
    QMetaType::Void, QMetaType::Int,   22,
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void, 0x80000000 | 27,   28,
    QMetaType::Void, 0x80000000 | 30,   28,
    QMetaType::Void, 0x80000000 | 32, 0x80000000 | 32, 0x80000000 | 15,   33,   34,   35,
    QMetaType::Void, QMetaType::Bool, QMetaType::Double, QMetaType::Bool, QMetaType::Double,   37,   38,   39,   40,

       0        // eod
};

void ObjectSegmentation::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ObjectSegmentation *_t = static_cast<ObjectSegmentation *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->new_image((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr(*)>(_a[1])),(*reinterpret_cast< const cv::Mat_<cv::Vec3b>(*)>(_a[2]))); break;
        case 1: _t->update_model_cloud((*reinterpret_cast< const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>(*)>(_a[1]))); break;
        case 2: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 3: _t->update_visualization(); break;
        case 4: _t->set_object_base_transform((*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[1]))); break;
        case 5: _t->finishedObjectSegmentation(); break;
        case 6: _t->segment_image((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->set_image((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 9: _t->segmentation_parameter_changed((*reinterpret_cast< const SegmentationParameter(*)>(_a[1]))); break;
        case 10: _t->object_modelling_parameter_changed((*reinterpret_cast< const ObjectModelling(*)>(_a[1]))); break;
        case 11: _t->set_roi((*reinterpret_cast< const Eigen::Vector3f(*)>(_a[1])),(*reinterpret_cast< const Eigen::Vector3f(*)>(_a[2])),(*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[3]))); break;
        case 12: _t->set_segmentation_params((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< const double(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ObjectSegmentation::*_t)(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & , const cv::Mat_<cv::Vec3b> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::new_image)) {
                *result = 0;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::update_model_cloud)) {
                *result = 1;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)(const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::printStatus)) {
                *result = 2;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::update_visualization)) {
                *result = 3;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)(const Eigen::Matrix4f & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::set_object_base_transform)) {
                *result = 4;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::finishedObjectSegmentation)) {
                *result = 5;
            }
        }
    }
}

const QMetaObject ObjectSegmentation::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ObjectSegmentation.data,
      qt_meta_data_ObjectSegmentation,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ObjectSegmentation::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ObjectSegmentation::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ObjectSegmentation.stringdata0))
        return static_cast<void*>(const_cast< ObjectSegmentation*>(this));
    return QThread::qt_metacast(_clname);
}

int ObjectSegmentation::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
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
void ObjectSegmentation::new_image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & _t1, const cv::Mat_<cv::Vec3b> & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ObjectSegmentation::update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ObjectSegmentation::printStatus(const std::string & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ObjectSegmentation::update_visualization()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void ObjectSegmentation::set_object_base_transform(const Eigen::Matrix4f & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ObjectSegmentation::finishedObjectSegmentation()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
