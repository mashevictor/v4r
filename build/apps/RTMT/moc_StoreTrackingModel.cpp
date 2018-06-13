/****************************************************************************
** Meta object code from reading C++ file 'StoreTrackingModel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/StoreTrackingModel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'StoreTrackingModel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_StoreTrackingModel_t {
    QByteArrayData data[15];
    char stringdata0[223];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_StoreTrackingModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_StoreTrackingModel_t qt_meta_stringdata_StoreTrackingModel = {
    {
QT_MOC_LITERAL(0, 0, 18), // "StoreTrackingModel"
QT_MOC_LITERAL(1, 19, 11), // "printStatus"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 11), // "std::string"
QT_MOC_LITERAL(4, 44, 4), // "_txt"
QT_MOC_LITERAL(5, 49, 26), // "finishedStoreTrackingModel"
QT_MOC_LITERAL(6, 76, 18), // "cam_params_changed"
QT_MOC_LITERAL(7, 95, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(8, 115, 11), // "_cam_params"
QT_MOC_LITERAL(9, 127, 25), // "set_object_base_transform"
QT_MOC_LITERAL(10, 153, 15), // "Eigen::Matrix4f"
QT_MOC_LITERAL(11, 169, 22), // "_object_base_transform"
QT_MOC_LITERAL(12, 192, 12), // "set_cb_param"
QT_MOC_LITERAL(13, 205, 9), // "create_cb"
QT_MOC_LITERAL(14, 215, 7) // "rnn_thr"

    },
    "StoreTrackingModel\0printStatus\0\0"
    "std::string\0_txt\0finishedStoreTrackingModel\0"
    "cam_params_changed\0RGBDCameraParameter\0"
    "_cam_params\0set_object_base_transform\0"
    "Eigen::Matrix4f\0_object_base_transform\0"
    "set_cb_param\0create_cb\0rnn_thr"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_StoreTrackingModel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    0,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   43,    2, 0x0a /* Public */,
       9,    1,   46,    2, 0x0a /* Public */,
      12,    2,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::Bool, QMetaType::Float,   13,   14,

       0        // eod
};

void StoreTrackingModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        StoreTrackingModel *_t = static_cast<StoreTrackingModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->finishedStoreTrackingModel(); break;
        case 2: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 3: _t->set_object_base_transform((*reinterpret_cast< const Eigen::Matrix4f(*)>(_a[1]))); break;
        case 4: _t->set_cb_param((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (StoreTrackingModel::*_t)(const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&StoreTrackingModel::printStatus)) {
                *result = 0;
            }
        }
        {
            typedef void (StoreTrackingModel::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&StoreTrackingModel::finishedStoreTrackingModel)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject StoreTrackingModel::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_StoreTrackingModel.data,
      qt_meta_data_StoreTrackingModel,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *StoreTrackingModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *StoreTrackingModel::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_StoreTrackingModel.stringdata0))
        return static_cast<void*>(const_cast< StoreTrackingModel*>(this));
    return QThread::qt_metacast(_clname);
}

int StoreTrackingModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void StoreTrackingModel::printStatus(const std::string & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void StoreTrackingModel::finishedStoreTrackingModel()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
