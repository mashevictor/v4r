/****************************************************************************
** Meta object code from reading C++ file 'BundleAdjustment.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/BundleAdjustment.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'BundleAdjustment.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BundleAdjustment_t {
    QByteArrayData data[17];
    char stringdata0[349];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BundleAdjustment_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BundleAdjustment_t qt_meta_stringdata_BundleAdjustment = {
    {
QT_MOC_LITERAL(0, 0, 16), // "BundleAdjustment"
QT_MOC_LITERAL(1, 17, 11), // "printStatus"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 11), // "std::string"
QT_MOC_LITERAL(4, 42, 4), // "_txt"
QT_MOC_LITERAL(5, 47, 18), // "update_model_cloud"
QT_MOC_LITERAL(6, 66, 51), // "boost::shared_ptr<Sensor::Ali..."
QT_MOC_LITERAL(7, 118, 9), // "_oc_cloud"
QT_MOC_LITERAL(8, 128, 21), // "update_cam_trajectory"
QT_MOC_LITERAL(9, 150, 55), // "boost::shared_ptr<std::vector..."
QT_MOC_LITERAL(10, 206, 15), // "_cam_trajectory"
QT_MOC_LITERAL(11, 222, 20), // "update_visualization"
QT_MOC_LITERAL(12, 243, 23), // "finishedOptimizeCameras"
QT_MOC_LITERAL(13, 267, 11), // "num_cameras"
QT_MOC_LITERAL(14, 279, 26), // "cam_tracker_params_changed"
QT_MOC_LITERAL(15, 306, 22), // "CamaraTrackerParameter"
QT_MOC_LITERAL(16, 329, 19) // "_cam_tracker_params"

    },
    "BundleAdjustment\0printStatus\0\0std::string\0"
    "_txt\0update_model_cloud\0"
    "boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>\0"
    "_oc_cloud\0update_cam_trajectory\0"
    "boost::shared_ptr<std::vector<Sensor::CameraLocation> >\0"
    "_cam_trajectory\0update_visualization\0"
    "finishedOptimizeCameras\0num_cameras\0"
    "cam_tracker_params_changed\0"
    "CamaraTrackerParameter\0_cam_tracker_params"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BundleAdjustment[] = {

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
       1,    1,   44,    2, 0x06 /* Public */,
       5,    1,   47,    2, 0x06 /* Public */,
       8,    1,   50,    2, 0x06 /* Public */,
      11,    0,   53,    2, 0x06 /* Public */,
      12,    1,   54,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    1,   57,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   13,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 15,   16,

       0        // eod
};

void BundleAdjustment::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BundleAdjustment *_t = static_cast<BundleAdjustment *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->update_model_cloud((*reinterpret_cast< const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>(*)>(_a[1]))); break;
        case 2: _t->update_cam_trajectory((*reinterpret_cast< const boost::shared_ptr<std::vector<Sensor::CameraLocation> >(*)>(_a[1]))); break;
        case 3: _t->update_visualization(); break;
        case 4: _t->finishedOptimizeCameras((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->cam_tracker_params_changed((*reinterpret_cast< const CamaraTrackerParameter(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BundleAdjustment::*_t)(const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BundleAdjustment::printStatus)) {
                *result = 0;
            }
        }
        {
            typedef void (BundleAdjustment::*_t)(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BundleAdjustment::update_model_cloud)) {
                *result = 1;
            }
        }
        {
            typedef void (BundleAdjustment::*_t)(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BundleAdjustment::update_cam_trajectory)) {
                *result = 2;
            }
        }
        {
            typedef void (BundleAdjustment::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BundleAdjustment::update_visualization)) {
                *result = 3;
            }
        }
        {
            typedef void (BundleAdjustment::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BundleAdjustment::finishedOptimizeCameras)) {
                *result = 4;
            }
        }
    }
}

const QMetaObject BundleAdjustment::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_BundleAdjustment.data,
      qt_meta_data_BundleAdjustment,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BundleAdjustment::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BundleAdjustment::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BundleAdjustment.stringdata0))
        return static_cast<void*>(const_cast< BundleAdjustment*>(this));
    return QThread::qt_metacast(_clname);
}

int BundleAdjustment::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void BundleAdjustment::printStatus(const std::string & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BundleAdjustment::update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BundleAdjustment::update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BundleAdjustment::update_visualization()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void BundleAdjustment::finishedOptimizeCameras(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
