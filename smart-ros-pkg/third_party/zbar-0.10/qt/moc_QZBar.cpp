/****************************************************************************
** Meta object code from reading C++ file 'QZBar.h'
**
** Created: Tue Mar 4 21:07:59 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../include/zbar/QZBar.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QZBar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_zbar__QZBar[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       3,   54, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      25,   13,   12,   12, 0x05,
      53,   43,   12,   12, 0x05,
      79,   74,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
     112,  100,   12,   12, 0x0a,
     149,  136,   12,   12, 0x0a,
     171,   12,   12,   12, 0x2a,
     195,  189,   12,   12, 0x0a,
     213,   12,   12,   12, 0x09,

 // properties: name, type, flags
     100,  226, 0x0a094103,
     136,  234, 0x01094103,
      13,  234, 0x01094001,

       0        // eod
};

static const char qt_meta_stringdata_zbar__QZBar[] = {
    "zbar::QZBar\0\0videoOpened\0videoOpened(bool)\0"
    "type,data\0decoded(int,QString)\0text\0"
    "decodedText(QString)\0videoDevice\0"
    "setVideoDevice(QString)\0videoEnabled\0"
    "setVideoEnabled(bool)\0setVideoEnabled()\0"
    "image\0scanImage(QImage)\0sizeChange()\0"
    "QString\0bool\0"
};

void zbar::QZBar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QZBar *_t = static_cast<QZBar *>(_o);
        switch (_id) {
        case 0: _t->videoOpened((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->decoded((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 2: _t->decodedText((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->setVideoDevice((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->setVideoEnabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->setVideoEnabled(); break;
        case 6: _t->scanImage((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 7: _t->sizeChange(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData zbar::QZBar::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject zbar::QZBar::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_zbar__QZBar,
      qt_meta_data_zbar__QZBar, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &zbar::QZBar::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *zbar::QZBar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *zbar::QZBar::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_zbar__QZBar))
        return static_cast<void*>(const_cast< QZBar*>(this));
    return QWidget::qt_metacast(_clname);
}

int zbar::QZBar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
#ifndef QT_NO_PROPERTIES
      else if (_c == QMetaObject::ReadProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = videoDevice(); break;
        case 1: *reinterpret_cast< bool*>(_v) = isVideoEnabled(); break;
        case 2: *reinterpret_cast< bool*>(_v) = isVideoOpened(); break;
        }
        _id -= 3;
    } else if (_c == QMetaObject::WriteProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: setVideoDevice(*reinterpret_cast< QString*>(_v)); break;
        case 1: setVideoEnabled(*reinterpret_cast< bool*>(_v)); break;
        }
        _id -= 3;
    } else if (_c == QMetaObject::ResetProperty) {
        _id -= 3;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 3;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 3;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 3;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 3;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 3;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void zbar::QZBar::videoOpened(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void zbar::QZBar::decoded(int _t1, const QString & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void zbar::QZBar::decodedText(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE