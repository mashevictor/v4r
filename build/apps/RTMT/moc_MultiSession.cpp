/****************************************************************************
** Meta object code from reading C++ file 'MultiSession.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../apps/RTMT/MultiSession.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MultiSession.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MultiSession_t {
    QByteArrayData data[14];
    char stringdata0[223];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MultiSession_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MultiSession_t qt_meta_stringdata_MultiSession = {
    {
QT_MOC_LITERAL(0, 0, 12), // "MultiSession"
QT_MOC_LITERAL(1, 13, 11), // "printStatus"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 11), // "std::string"
QT_MOC_LITERAL(4, 38, 4), // "_txt"
QT_MOC_LITERAL(5, 43, 17), // "finishedAlignment"
QT_MOC_LITERAL(6, 61, 2), // "ok"
QT_MOC_LITERAL(7, 64, 18), // "update_model_cloud"
QT_MOC_LITERAL(8, 83, 51), // "boost::shared_ptr<Sensor::Ali..."
QT_MOC_LITERAL(9, 135, 9), // "_oc_cloud"
QT_MOC_LITERAL(10, 145, 20), // "update_visualization"
QT_MOC_LITERAL(11, 166, 34), // "object_modelling_parameter_ch..."
QT_MOC_LITERAL(12, 201, 15), // "ObjectModelling"
QT_MOC_LITERAL(13, 217, 5) // "param"

    },
    "MultiSession\0printStatus\0\0std::string\0"
    "_txt\0finishedAlignment\0ok\0update_model_cloud\0"
    "boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>\0"
    "_oc_cloud\0update_visualization\0"
    "object_modelling_parameter_changed\0"
    "ObjectModelling\0param"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MultiSession[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,
       7,    1,   45,    2, 0x06 /* Public */,
      10,    0,   48,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 12,   13,

       0        // eod
};

void MultiSession::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MultiSession *_t = static_cast<MultiSession *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->printStatus((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->finishedAlignment((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->update_model_cloud((*reinterpret_cast< const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector>(*)>(_a[1]))); break;
        case 3: _t->update_visualization(); break;
        case 4: _t->object_modelling_parameter_changed((*reinterpret_cast< const ObjectModelling(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MultiSession::*_t)(const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MultiSession::printStatus)) {
                *result = 0;
            }
        }
        {
            typedef void (MultiSession::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MultiSession::finishedAlignment)) {
                *result = 1;
            }
        }
        {
            typedef void (MultiSession::*_t)(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MultiSession::update_model_cloud)) {
                *result = 2;
            }
        }
        {
            typedef void (MultiSession::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MultiSession::update_visualization)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject MultiSession::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_MultiSession.data,
      qt_meta_data_MultiSession,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MultiSession::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MultiSession::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MultiSession.stringdata0))
        return static_cast<void*>(const_cast< MultiSession*>(this));
    return QThread::qt_metacast(_clname);
}

int MultiSession::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void MultiSession::printStatus(const std::string & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MultiSession::finishedAlignment(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MultiSession::update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MultiSession::update_visualization()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
