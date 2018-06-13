/****************************************************************************
** Meta object code from reading C++ file 'params.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/params.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'params.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Params_t {
    QByteArrayData data[30];
    char stringdata0[524];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Params_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Params_t qt_meta_stringdata_Params = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Params"
QT_MOC_LITERAL(1, 7, 18), // "cam_params_changed"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 19), // "RGBDCameraParameter"
QT_MOC_LITERAL(4, 47, 3), // "cam"
QT_MOC_LITERAL(5, 51, 26), // "cam_tracker_params_changed"
QT_MOC_LITERAL(6, 78, 22), // "CamaraTrackerParameter"
QT_MOC_LITERAL(7, 101, 5), // "param"
QT_MOC_LITERAL(8, 107, 17), // "rgbd_path_changed"
QT_MOC_LITERAL(9, 125, 35), // "bundle_adjustment_parameter_c..."
QT_MOC_LITERAL(10, 161, 25), // "BundleAdjustmentParameter"
QT_MOC_LITERAL(11, 187, 30), // "segmentation_parameter_changed"
QT_MOC_LITERAL(12, 218, 21), // "SegmentationParameter"
QT_MOC_LITERAL(13, 240, 34), // "object_modelling_parameter_ch..."
QT_MOC_LITERAL(14, 275, 15), // "ObjectModelling"
QT_MOC_LITERAL(15, 291, 14), // "set_roi_params"
QT_MOC_LITERAL(16, 306, 14), // "_bbox_scale_xy"
QT_MOC_LITERAL(17, 321, 18), // "_bbox_scale_height"
QT_MOC_LITERAL(18, 340, 9), // "_seg_offs"
QT_MOC_LITERAL(19, 350, 23), // "set_segmentation_params"
QT_MOC_LITERAL(20, 374, 12), // "use_roi_segm"
QT_MOC_LITERAL(21, 387, 4), // "offs"
QT_MOC_LITERAL(22, 392, 13), // "_use_dense_mv"
QT_MOC_LITERAL(23, 406, 15), // "_edge_radius_px"
QT_MOC_LITERAL(24, 422, 12), // "set_cb_param"
QT_MOC_LITERAL(25, 435, 9), // "create_cb"
QT_MOC_LITERAL(26, 445, 7), // "rnn_thr"
QT_MOC_LITERAL(27, 453, 19), // "on_okButton_clicked"
QT_MOC_LITERAL(28, 473, 27), // "on_pushFindRGBDPath_pressed"
QT_MOC_LITERAL(29, 501, 22) // "on_applyButton_clicked"

    },
    "Params\0cam_params_changed\0\0"
    "RGBDCameraParameter\0cam\0"
    "cam_tracker_params_changed\0"
    "CamaraTrackerParameter\0param\0"
    "rgbd_path_changed\0bundle_adjustment_parameter_changed\0"
    "BundleAdjustmentParameter\0"
    "segmentation_parameter_changed\0"
    "SegmentationParameter\0"
    "object_modelling_parameter_changed\0"
    "ObjectModelling\0set_roi_params\0"
    "_bbox_scale_xy\0_bbox_scale_height\0"
    "_seg_offs\0set_segmentation_params\0"
    "use_roi_segm\0offs\0_use_dense_mv\0"
    "_edge_radius_px\0set_cb_param\0create_cb\0"
    "rnn_thr\0on_okButton_clicked\0"
    "on_pushFindRGBDPath_pressed\0"
    "on_applyButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Params[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   74,    2, 0x06 /* Public */,
       5,    1,   77,    2, 0x06 /* Public */,
       8,    0,   80,    2, 0x06 /* Public */,
       9,    1,   81,    2, 0x06 /* Public */,
      11,    1,   84,    2, 0x06 /* Public */,
      13,    1,   87,    2, 0x06 /* Public */,
      15,    3,   90,    2, 0x06 /* Public */,
      19,    4,   97,    2, 0x06 /* Public */,
      24,    2,  106,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      27,    0,  111,    2, 0x08 /* Private */,
      28,    0,  112,    2, 0x08 /* Private */,
      29,    0,  113,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,    7,
    QMetaType::Void, 0x80000000 | 12,    7,
    QMetaType::Void, 0x80000000 | 14,    7,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   16,   17,   18,
    QMetaType::Void, QMetaType::Bool, QMetaType::Double, QMetaType::Bool, QMetaType::Double,   20,   21,   22,   23,
    QMetaType::Void, QMetaType::Bool, QMetaType::Float,   25,   26,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Params::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Params *_t = static_cast<Params *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->cam_params_changed((*reinterpret_cast< const RGBDCameraParameter(*)>(_a[1]))); break;
        case 1: _t->cam_tracker_params_changed((*reinterpret_cast< const CamaraTrackerParameter(*)>(_a[1]))); break;
        case 2: _t->rgbd_path_changed(); break;
        case 3: _t->bundle_adjustment_parameter_changed((*reinterpret_cast< const BundleAdjustmentParameter(*)>(_a[1]))); break;
        case 4: _t->segmentation_parameter_changed((*reinterpret_cast< const SegmentationParameter(*)>(_a[1]))); break;
        case 5: _t->object_modelling_parameter_changed((*reinterpret_cast< const ObjectModelling(*)>(_a[1]))); break;
        case 6: _t->set_roi_params((*reinterpret_cast< const double(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< const double(*)>(_a[3]))); break;
        case 7: _t->set_segmentation_params((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< const double(*)>(_a[4]))); break;
        case 8: _t->set_cb_param((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 9: _t->on_okButton_clicked(); break;
        case 10: _t->on_pushFindRGBDPath_pressed(); break;
        case 11: _t->on_applyButton_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Params::*_t)(const RGBDCameraParameter & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::cam_params_changed)) {
                *result = 0;
            }
        }
        {
            typedef void (Params::*_t)(const CamaraTrackerParameter & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::cam_tracker_params_changed)) {
                *result = 1;
            }
        }
        {
            typedef void (Params::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::rgbd_path_changed)) {
                *result = 2;
            }
        }
        {
            typedef void (Params::*_t)(const BundleAdjustmentParameter & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::bundle_adjustment_parameter_changed)) {
                *result = 3;
            }
        }
        {
            typedef void (Params::*_t)(const SegmentationParameter & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::segmentation_parameter_changed)) {
                *result = 4;
            }
        }
        {
            typedef void (Params::*_t)(const ObjectModelling & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::object_modelling_parameter_changed)) {
                *result = 5;
            }
        }
        {
            typedef void (Params::*_t)(const double & , const double & , const double & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::set_roi_params)) {
                *result = 6;
            }
        }
        {
            typedef void (Params::*_t)(bool , const double & , bool , const double & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::set_segmentation_params)) {
                *result = 7;
            }
        }
        {
            typedef void (Params::*_t)(bool , float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::set_cb_param)) {
                *result = 8;
            }
        }
    }
}

const QMetaObject Params::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_Params.data,
      qt_meta_data_Params,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Params::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Params::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Params.stringdata0))
        return static_cast<void*>(const_cast< Params*>(this));
    return QDialog::qt_metacast(_clname);
}

int Params::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void Params::cam_params_changed(const RGBDCameraParameter & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Params::cam_tracker_params_changed(const CamaraTrackerParameter & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Params::rgbd_path_changed()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void Params::bundle_adjustment_parameter_changed(const BundleAdjustmentParameter & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Params::segmentation_parameter_changed(const SegmentationParameter & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Params::object_modelling_parameter_changed(const ObjectModelling & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Params::set_roi_params(const double & _t1, const double & _t2, const double & _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Params::set_segmentation_params(bool _t1, const double & _t2, bool _t3, const double & _t4)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Params::set_cb_param(bool _t1, float _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_END_MOC_NAMESPACE
