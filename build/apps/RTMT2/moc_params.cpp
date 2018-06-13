/****************************************************************************
** Meta object code from reading C++ file 'params.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT2/params.h"
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
    QByteArrayData data[16];
    char stringdata0[289];
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
QT_MOC_LITERAL(5, 51, 17), // "rgbd_path_changed"
QT_MOC_LITERAL(6, 69, 14), // "set_roi_params"
QT_MOC_LITERAL(7, 84, 14), // "_bbox_scale_xy"
QT_MOC_LITERAL(8, 99, 18), // "_bbox_scale_height"
QT_MOC_LITERAL(9, 118, 9), // "_seg_offs"
QT_MOC_LITERAL(10, 128, 35), // "vignetting_calibration_file_c..."
QT_MOC_LITERAL(11, 164, 19), // "on_okButton_clicked"
QT_MOC_LITERAL(12, 184, 27), // "on_pushFindRGBDPath_pressed"
QT_MOC_LITERAL(13, 212, 26), // "on_pushFindVGNfile_pressed"
QT_MOC_LITERAL(14, 239, 26), // "on_pushFindCRFfile_pressed"
QT_MOC_LITERAL(15, 266, 22) // "on_applyButton_clicked"

    },
    "Params\0cam_params_changed\0\0"
    "RGBDCameraParameter\0cam\0rgbd_path_changed\0"
    "set_roi_params\0_bbox_scale_xy\0"
    "_bbox_scale_height\0_seg_offs\0"
    "vignetting_calibration_file_changed\0"
    "on_okButton_clicked\0on_pushFindRGBDPath_pressed\0"
    "on_pushFindVGNfile_pressed\0"
    "on_pushFindCRFfile_pressed\0"
    "on_applyButton_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Params[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x06 /* Public */,
       5,    0,   62,    2, 0x06 /* Public */,
       6,    3,   63,    2, 0x06 /* Public */,
      10,    0,   70,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    0,   71,    2, 0x08 /* Private */,
      12,    0,   72,    2, 0x08 /* Private */,
      13,    0,   73,    2, 0x08 /* Private */,
      14,    0,   74,    2, 0x08 /* Private */,
      15,    0,   75,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    7,    8,    9,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
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
        case 1: _t->rgbd_path_changed(); break;
        case 2: _t->set_roi_params((*reinterpret_cast< const double(*)>(_a[1])),(*reinterpret_cast< const double(*)>(_a[2])),(*reinterpret_cast< const double(*)>(_a[3]))); break;
        case 3: _t->vignetting_calibration_file_changed(); break;
        case 4: _t->on_okButton_clicked(); break;
        case 5: _t->on_pushFindRGBDPath_pressed(); break;
        case 6: _t->on_pushFindVGNfile_pressed(); break;
        case 7: _t->on_pushFindCRFfile_pressed(); break;
        case 8: _t->on_applyButton_clicked(); break;
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
            typedef void (Params::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::rgbd_path_changed)) {
                *result = 1;
            }
        }
        {
            typedef void (Params::*_t)(const double & , const double & , const double & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::set_roi_params)) {
                *result = 2;
            }
        }
        {
            typedef void (Params::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Params::vignetting_calibration_file_changed)) {
                *result = 3;
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
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
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
void Params::rgbd_path_changed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void Params::set_roi_params(const double & _t1, const double & _t2, const double & _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Params::vignetting_calibration_file_changed()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
