/****************************************************************************
** Meta object code from reading C++ file 'ObjectSegmentation.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT2/ObjectSegmentation.h"
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
    QByteArrayData data[18];
    char stringdata0[293];
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
QT_MOC_LITERAL(3, 30, 38), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(4, 69, 6), // "_cloud"
QT_MOC_LITERAL(5, 76, 19), // "cv::Mat_<cv::Vec3b>"
QT_MOC_LITERAL(6, 96, 5), // "image"
QT_MOC_LITERAL(7, 102, 18), // "update_model_cloud"
QT_MOC_LITERAL(8, 121, 44), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(9, 166, 11), // "printStatus"
QT_MOC_LITERAL(10, 178, 11), // "std::string"
QT_MOC_LITERAL(11, 190, 4), // "_txt"
QT_MOC_LITERAL(12, 195, 20), // "update_visualization"
QT_MOC_LITERAL(13, 216, 17), // "finishedModelling"
QT_MOC_LITERAL(14, 234, 14), // "set_roi_params"
QT_MOC_LITERAL(15, 249, 14), // "_bbox_scale_xy"
QT_MOC_LITERAL(16, 264, 18), // "_bbox_scale_height"
QT_MOC_LITERAL(17, 283, 9) // "_seg_offs"

    },
    "ObjectSegmentation\0new_image\0\0"
    "pcl::PointCloud<pcl::PointXYZRGB>::Ptr\0"
    "_cloud\0cv::Mat_<cv::Vec3b>\0image\0"
    "update_model_cloud\0"
    "pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr\0"
    "printStatus\0std::string\0_txt\0"
    "update_visualization\0finishedModelling\0"
    "set_roi_params\0_bbox_scale_xy\0"
    "_bbox_scale_height\0_seg_offs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ObjectSegmentation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x06 /* Public */,
       7,    1,   49,    2, 0x06 /* Public */,
       9,    1,   52,    2, 0x06 /* Public */,
      12,    0,   55,    2, 0x06 /* Public */,
      13,    0,   56,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    3,   57,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 8,    4,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   15,   16,   17,

       0        // eod
};

void ObjectSegmentation::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ObjectSegmentation *_t = static_cast<ObjectSegmentation *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->new_image((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGB>::Ptr(*)>(_a[1])),(*reinterpret_cast< const cv::Mat_<cv::Vec3b>(*)>(_a[2]))); break;
        case 1: _t->update_model_cloud((*reinterpret_cast< const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(*)>(_a[1]))); break;
        case 2: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 3: _t->update_visualization(); break;
        case 4: _t->finishedModelling(); break;
        case 5: _t->set_roi_params((*reinterpret_cast< const double(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< const double(*)>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ObjectSegmentation::*_t)(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & , const cv::Mat_<cv::Vec3b> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::new_image)) {
                *result = 0;
            }
        }
        {
            typedef void (ObjectSegmentation::*_t)(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & );
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
            typedef void (ObjectSegmentation::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ObjectSegmentation::finishedModelling)) {
                *result = 4;
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
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void ObjectSegmentation::new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _t1, const cv::Mat_<cv::Vec3b> & _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ObjectSegmentation::update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & _t1)
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
void ObjectSegmentation::finishedModelling()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
