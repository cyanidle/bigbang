#ifndef SETTINGSSERIALIZERBASE_H
#define SETTINGSSERIALIZERBASE_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <QVariant>
#include <QMetaProperty>
#include <QMetaObject>
#include <type_traits>

#define _SERIALIZER_PORTABLE 1
#if !_SERIALIZER_PORTABLE
#include "settingsparsingglobal.h"
#include "settingsparsinglogging.h"
#endif
#define _ENABLE_WRITTEN_WARNINGS !_SERIALIZER_PORTABLE

#define _SETTINGS_GET_MAP(name) _settings_impl_get_map_##name
#define _SETTINGS_SET(name) _settings_impl_set_variant_##name
#define _SETTINGS_GET(name) _settings_impl_get_variant_##name
#define _HAS_DEFAULT(name) name##_hasDefault
#define _DEFAULT_GETTER(name) name##_Default

#define _GET_MAP_PROP_SUFFIX "_getMapImplProp"
#define _HAS_DEFAULT_SUFFIX "_hasDefaultProp"

#define _HAS_DEFAULT_PROP(name) name##_hasDefaultProp
#define _GET_MAP_PROP(name) name##_getMapImplProp


#define _SYNTAX_GUARD \
public:

#define _ROOT_PREFIX "~"
#define COMMA ,
#define DEFAULT
#define CUSTOM_NO_INIT initMock
#define CUSTOM_NO_READ readMock
#define MAP_MARKER "<map>"
#define LIST_MARKER "<list>"

#define SERIAL_FIELD(type, name, defaults...)                                                       \
    public:                                                                                         \
        type name = type(defaults);                                                                 \
        static type _DEFAULT_GETTER(name)() {return type(defaults);}                                \
        constexpr static bool _HAS_DEFAULT(name)(){return !QLatin1String(#defaults).isEmpty();}     \
    private:                                                                                        \
        Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                \
        Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                        \
        Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                    \
        QVariantMap _SETTINGS_GET_MAP(name) () const {                                              \
            _check_if_has_is_serializable();                                                        \
            return {{#name, #type}};                                                                \
        }                                                                                           \
        QVariant _SETTINGS_GET(name) () const {                                                     \
            return QVariant(name);                                                                  \
        }                                                                                           \
        void _SETTINGS_SET(name) (const QVariant & newVal) {                                        \
            if (newVal.canConvert<type>()) {                                                        \
                name = newVal.value<type>();                                                        \
                updateWasInit(#name);                                                               \
            }                                                                                       \
        }                                                                                           \
_SYNTAX_GUARD

#define SERIAL_CONTAINER(container, type, name, defaults...)                                        \
    public:                                                                                         \
        container<type> name = container<type>(defaults);                                           \
        static container<type> _DEFAULT_GETTER(name)() {return container<type>(defaults);}          \
        constexpr static bool _HAS_DEFAULT(name)() {return !QLatin1String(#defaults).isEmpty();}    \
    private:                                                                                        \
        Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                \
        Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                        \
        Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                    \
        QVariantMap _SETTINGS_GET_MAP(name) () const {                                              \
            _check_if_has_is_serializable();                                                        \
            return {{#name, QVariantMap{{LIST_MARKER, #type}}}};                                    \
        }                                                                                           \
        QVariant _SETTINGS_GET(name) () const {                                                     \
            auto result = QVariantList();                                                           \
            for (int i = 0; i < name.size(); ++i) {                                                 \
                result.append(QVariant(name.at(i)));                                                \
            }                                                                                       \
            return QVariant(result);                                                                \
        }                                                                                           \
        void _SETTINGS_SET(name) (const QVariant & newVal) {                                        \
            if (newVal.canConvert<QVariantList>()){                                                 \
                name.clear();                                                                       \
                bool wasInit = true;                                                                \
                auto source = newVal.value<QVariantList>();                                         \
                for (int i = 0; i < source.size(); ++i) {                                           \
                    if (source.at(i).canConvert<type>()) {                                          \
                        name.append(source.at(i).value<type>());                                    \
                    } else {                                                                        \
                        wasInit = false;                                                            \
                    }                                                                               \
                }                                                                                   \
                if (wasInit && source.size() > 0) {                                                 \
                    updateWasInit(#name);                                                           \
                }                                                                                   \
            }                                                                                       \
        }                                                                                           \
_SYNTAX_GUARD

#define SERIAL_NEST(obj, name, defaults...)                                                         \
    public:                                                                                         \
        obj name = obj(defaults);                                                                   \
        static obj _DEFAULT_GETTER(name)() {return obj(defaults);}                                  \
        constexpr static bool _HAS_DEFAULT(name)()                                           \
              {return !QLatin1String(#defaults).isEmpty();}                                         \
    private:                                                                                        \
        Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                \
        Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                        \
        Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                    \
        QVariantMap _SETTINGS_GET_MAP(name) () const {                                              \
            _check_if_has_is_serializable();                                                        \
            return {{#name, this->name.structure()}};                                               \
        }                                                                                           \
        QVariant _SETTINGS_GET(name) () const {                                                     \
            return name.serialize();                                                                \
        }                                                                                           \
        void _SETTINGS_SET(name) (const QVariant & newVal) {                                        \
            if (name.deserialize(newVal.value<QVariantMap>(),                                       \
                    isPedantic(), prefix()+'/'+#name)) {                                            \
                updateWasInit(#name);                                                               \
            }                                                                                       \
        }                                                                                           \
_SYNTAX_GUARD

#define SERIAL_CONTAINER_NEST(container, obj, name, defaults...)                                    \
    public:                                                                                         \
        container<obj> name = container<obj>(defaults);                                             \
        static container<obj> _DEFAULT_GETTER(name)() {return container<obj>(defaults);}            \
        constexpr static bool _HAS_DEFAULT(name)()                                           \
             {return !QLatin1String(#defaults).isEmpty();}                                          \
    private:                                                                                        \
        Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                \
        Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                        \
        Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                    \
        QVariantMap _SETTINGS_GET_MAP(name) () const {                                              \
            _check_if_has_is_serializable();                                                        \
            return {{#name, QVariantMap{{LIST_MARKER, obj{}.structure()}}}};                        \
        }                                                                                           \
        QVariant _SETTINGS_GET(name) () const {                                                     \
            auto result = QVariantList();                                                           \
            for (int i = 0; i < name.size(); ++i) {                                                 \
                result.append(name.at(i).serialize());                                              \
            }                                                                                       \
            return result;                                                                          \
        }                                                                                           \
        void _SETTINGS_SET(name) (const QVariant & newVal) {                                        \
            if (newVal.canConvert<QVariantList>()){                                                 \
                name.clear();                                                                       \
                auto source = newVal.value<QVariantList>();                                         \
                bool wasInit = true;                                                                \
                for (int i = 0; i < source.size(); ++i) {                                           \
                    auto current = obj();                                                           \
                    if (!current.deserialize(source.at(i).value<QVariantMap>(), isPedantic(),       \
                                prefix()+'/'+#name+'/'+QString::number(i))) {                       \
                        wasInit = false;                                                            \
                    }                                                                               \
                    name.append(current);                                                           \
                }                                                                                   \
                if (wasInit && source.size() > 0) {                                                 \
                    updateWasInit(#name);                                                           \
                }                                                                                   \
            }                                                                                       \
        }                                                                                           \
_SYNTAX_GUARD

//! QVariant getter(), which serializes field;
//! setter(QVariant) returns true/false (setting value successful)
#define SERIAL_CUSTOM(type, name, setter, getter, defaults...)                                      \
    public:                                                                                         \
        type name = type{defaults};                                                                 \
        static type _DEFAULT_GETTER(name)() {return type{defaults};}                                \
        constexpr static bool _HAS_DEFAULT(name)() {return !QLatin1String(#defaults).isEmpty();}    \
    private:                                                                                        \
        Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                \
        Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                        \
        Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                    \
        QVariantMap _SETTINGS_GET_MAP(name) () const {                                              \
            _check_if_has_is_serializable();                                                        \
            return {{#name, #type}};                                                                \
        }                                                                                           \
        QVariant _SETTINGS_GET(name) () const {                                                     \
            return QVariant(getter());                                                              \
        }                                                                                           \
        void _SETTINGS_SET(name) (const QVariant & newVal) {                                        \
            if (setter(newVal)) {                                                                   \
                updateWasInit(#name);                                                               \
            }                                                                                       \
        }                                                                                           \
_SYNTAX_GUARD

#define SERIAL_MAP(type, name, defaults...)                                                         \
public:                                                                                             \
    QMap<QString, type> name = QMap<QString, type>(defaults);                                       \
    static QMap<QString, type> _DEFAULT_GETTER(name)() {return QMap<QString, type>(defaults);}      \
    constexpr static bool _HAS_DEFAULT(name)() {return !QLatin1String(#defaults).isEmpty();}        \
private:                                                                                            \
    Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                    \
    Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                            \
    Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                        \
    QVariantMap _SETTINGS_GET_MAP(name) () const {                                                  \
        _check_if_has_is_serializable();                                                            \
        return {{#name, QVariantMap{{MAP_MARKER, #type}}}};                                         \
    }                                                                                               \
    QVariant _SETTINGS_GET(name) () const {                                                         \
        auto result = QVariantMap();                                                                \
        for (auto iter = name.constBegin(); iter != name.constEnd(); ++iter) {                      \
                result.insert(iter.key(), iter.value());                                            \
        }                                                                                           \
        return result;                                                                              \
    }                                                                                               \
    void _SETTINGS_SET(name) (const QVariant & newVal) {                                            \
        if (newVal.canConvert<QVariantMap>()){                                                      \
            name.clear();                                                                           \
            bool wasInit = true;                                                                    \
            auto currentMap = newVal.toMap();                                                       \
            for (auto iter = currentMap.constBegin(); iter != currentMap.constEnd(); ++iter) {      \
                if (!iter.value().canConvert<type>()) {                                             \
                    wasInit = false;                                                                \
                } else {                                                                            \
                    name.insert(iter.key(), iter.value().value<type>());                            \
                }                                                                                   \
            }                                                                                       \
            if (wasInit) {                                                                          \
                updateWasInit(#name);                                                               \
            }                                                                                       \
        }                                                                                           \
    }                                                                                               \
_SYNTAX_GUARD

#define SERIAL_MAP_NEST(obj, name, defaults...)                                                     \
public:                                                                                             \
    QMap<QString, obj> name = QMap<QString, obj>(defaults);                                         \
    static QMap<QString, obj> _DEFAULT_GETTER(name)() {return QMap<QString, obj>(defaults);}        \
    constexpr static bool _HAS_DEFAULT(name)()                                                      \
        {return !QLatin1String(#defaults).isEmpty();}                                               \
private:                                                                                            \
    Q_PROPERTY(QVariant name READ _SETTINGS_GET(name) WRITE _SETTINGS_SET(name))                    \
    Q_PROPERTY(QVariant _HAS_DEFAULT_PROP(name) READ _HAS_DEFAULT(name))                            \
    Q_PROPERTY(QVariantMap _GET_MAP_PROP(name) READ _SETTINGS_GET_MAP(name))                        \
    QVariantMap _SETTINGS_GET_MAP(name) () const {                                                  \
        _check_if_has_is_serializable();                                                            \
        return {{#name, QVariantMap{{MAP_MARKER, obj{}.structure()}}}};                             \
    }                                                                                               \
    QVariant _SETTINGS_GET(name) () const {                                                         \
        auto result = QVariantMap();                                                                \
        for (auto iter = name.constBegin(); iter != name.constEnd(); ++iter) {                      \
            result.insert(iter.key(),   iter.value().serialize());                                  \
        }                                                                                           \
        return result;                                                                              \
    }                                                                                               \
    void _SETTINGS_SET(name) (const QVariant & newVal) {                                            \
        if (newVal.canConvert<QVariantMap>()){                                                      \
            name.clear();                                                                           \
            bool wasInit = true;                                                                    \
            auto currentMap = newVal.toMap();                                                       \
            for (auto iter = currentMap.constBegin(); iter != currentMap.constEnd(); ++iter) {      \
                obj current;                                                                        \
                if (!current.deserialize(iter.value().toMap()), isPedantic(),                       \
                                 prefix()+'/'+iter.key()+'/'+#name) {                               \
                    wasInit = false;                                                                \
                }                                                                                   \
                name.insert(iter.key(), current);                                                   \
            }                                                                                       \
            if (wasInit) {                                                                          \
                updateWasInit(#name);                                                               \
            }                                                                                       \
        }                                                                                           \
    }                                                                                               \
_SYNTAX_GUARD

#define SERIAL_POST_INIT(func, defaults...)                                                         \
private:                                                                                            \
    void _settings_impl_PostInit() override {                                                       \
        func(defaults);                                                                             \
    }                                                                                               \
_SYNTAX_GUARD

#define SERIAL_ON_FAIL(func, defaults...)                                                           \
private:                                                                                            \
    void _settings_impl_OnFail() override {                                                         \
        func(defaults);                                                                             \
}                                                                                                   \
_SYNTAX_GUARD

#define SERIAL_PRE_INIT(func, defaults...)                                                          \
private:                                                                                            \
    void _settings_impl_OnInit() override {                                                         \
        func(defaults);                                                                             \
}                                                                                                   \
    _SYNTAX_GUARD

#define SERIAL_ON_SUCCESS(func, defaults...)                                                        \
private:                                                                                            \
    void _settings_impl_OnSuccess() override {                                                      \
        func(defaults);                                                                             \
}                                                                                                   \
_SYNTAX_GUARD

#define IS_SERIALIZABLE                                                                             \
    private:                                                                                        \
        static void _check_if_has_is_serializable() noexcept {}                                     \
        virtual const QMetaObject * metaObject() const noexcept override  {                         \
            return &(this->staticMetaObject);                                                       \
    }                                                                                               \
_SYNTAX_GUARD

namespace Serializer {

    struct SerializerBase;

    template<class T>
    T fromQMap(const QVariantMap &src, bool *ok = nullptr)
    {
        static_assert(std::is_base_of<SerializerBase, T>(),
                      "Attempt to deserialize class not derived from SerializerBase!");
        T result;
        bool status = result.deserialize(src);
        if (ok) {
            *ok = status;
        }
        return result;
    }

    template<class T>
    QList<T> fromQList(const QVariantList &src, bool *ok = nullptr)
    {
        static_assert(std::is_base_of<SerializerBase, T>(),
                      "Attempt to deserialize class not derived from SerializerBase!");
        QList<T> result;
        bool status = true;
        for (auto &obj : src) {
            T current;
            if (!current.deserialize(obj.toMap())) {
                status = false;
            }
            result.append(current);
        }
        if (ok) {
            *ok = status;
        }
        return result;
    }

    template<typename T>
    QMap<QString, T> convertQMap(const QVariantMap &src)
    {
        QMap<QString, T> result;
        for (auto iter = src.constBegin(); iter != src.constEnd(); ++iter) {
            if (iter.value().canConvert<T>()) {
                result.insert(iter.key(), iter.value().value<T>());
            }
        }
        return result;
    }
    template<typename T>
    QList<T> convertQList(const QVariantList &src)
    {
        QList<T> result;
        for (auto iter = src.constBegin(); iter != src.constEnd(); ++iter) {
            if (iter->canConvert<T>()) {
                result.append(iter->value<T>());
            }
        }
        return result;
    }
}

/// \warning quint16 is bugged --> "can be" converted to from qstring
struct Serializer::SerializerBase
{
    template<class T>
    friend T fromQMap(const QVariantMap &src, bool *ok);
    template<class T>
    friend QList<T> fromQList(const QVariantList &src, bool *ok);
public:
    virtual QVariantMap structure() const {
        QVariantMap result;
        int propCount = metaObject()->propertyCount();
        for(int i = 0; i < propCount; i++) {
            auto prop = metaObject()->property(i);
            auto name = QString(prop.name());
            if (name.endsWith(_GET_MAP_PROP_SUFFIX)) {
                #if !_SERIALIZER_PORTABLE
                    result.insert(prop.readOnGadget(this).toMap());
                #else
                    auto subresult = prop.readOnGadget(this).toMap();
                    for (auto iter = subresult.constBegin(); iter != subresult.constEnd(); ++iter) {
                        result.insert(iter.key(), iter.value());
                    }
                #endif
            }
        }
        return result;
    }
    virtual bool deserialize(const QVariantMap &src, bool pedantic = true, const QString &prefix = _ROOT_PREFIX) {
        m_prefix = prefix;
        m_pedantic = pedantic;
        _settings_impl_OnInit();
        int propCount = metaObject()->propertyCount();
        for(int i = 0; i < propCount; i++) {
            auto prop = metaObject()->property(i);
            for(auto keyVal = src.constBegin(); keyVal!= src.constEnd(); ++keyVal) {
                if (keyVal.key() == prop.name()) {
                    // has a property with the same name but with suffix --> is made by macro --> should overwrite from src
                    if (metaObject()->indexOfProperty((keyVal.key() + _HAS_DEFAULT_SUFFIX).toStdString().c_str()) != -1) {
                        prop.writeOnGadget(this, keyVal.value());
                        break;
                    }
                }
            }
        }
        QStringList properties;
        for(int i = 0; i < propCount; i++) {
            properties.append(metaObject()->property(i).name());
        }
        _settings_impl_PostInit();
        if (wasFullyDeserialized(pedantic)) {
            _settings_impl_OnSuccess();
            return true;
        } else {
            _settings_impl_OnFail();
            return false;
        }
    }
    virtual QVariantMap serialize() const {
        auto result = QVariantMap();
        int propCount = metaObject()->propertyCount();
        for(int i = 0; i < propCount; i++) {
            auto property = metaObject()->property(i);
            auto name = QString(property.name());
            if (name.endsWith(_HAS_DEFAULT_SUFFIX) ||
                name.endsWith(_GET_MAP_PROP_SUFFIX)) {
                continue;
            }
            result.insert(property.name(), property.readOnGadget(this));
        }
        return result;
    }
    bool wasFullyDeserialized(bool pedantic = false) const {
        bool ok = true;
        for (int i = 0; i < metaObject()->propertyCount(); ++i) {
            QString currentName = metaObject()->property(i).name();
            if (!m_wasInit.contains(currentName) &&
                !currentName.endsWith(_HAS_DEFAULT_SUFFIX) &&
                !currentName.endsWith(_GET_MAP_PROP_SUFFIX) ) {
                if (_checkHasDefault(currentName)) {
                    continue;
                }
                if (pedantic) {
#if _ENABLE_WRITTEN_WARNINGS
                    settingsParsingWarn() << metaObject()->className() << ": Field (" <<
                        metaObject()->property(i).name() << ") could not be deserialized (or marked as failed!)";
#endif
                    throw std::runtime_error("Deserialization Error!");
                }
                ok = false;
            }
        }
        return ok;
    }
    const QString &prefix() const {return m_prefix;}
    bool isPedantic() const {return m_pedantic;}
    void setPrefix(const QString &newPref) {m_prefix = newPref;}
    const QStringList &deserializedFields() const {return m_wasInit;}
    virtual ~SerializerBase() = default;
protected:
    bool _checkHasDefault(const QString &name) const {
        auto propIndex = metaObject()->indexOfProperty((name + _HAS_DEFAULT_SUFFIX).toStdString().c_str());
        return (propIndex > -1) ? metaObject()->property(propIndex).readOnGadget(this).toBool() : false;
    }
    void updateWasInit(const QString &name, bool remove = false){
        if (!m_wasInit.contains(name)) {
            m_wasInit.append(name);
        } else if (remove) {
            m_wasInit.removeOne(name);
        }
    }
    QVariant readMock() const {return QVariant();}
    bool initMock(const QVariant &src) {Q_UNUSED(src); return true;}
    SerializerBase() : m_wasInit(), m_prefix(_ROOT_PREFIX), m_pedantic(true) {};
private:
    QStringList m_wasInit;
    QString m_prefix;
    bool m_pedantic;
    virtual void _settings_impl_OnInit(){};
    virtual void _settings_impl_PostInit(){};
    virtual void _settings_impl_OnFail(){};
    virtual void _settings_impl_OnSuccess(){};
    virtual const QMetaObject * metaObject() const noexcept = 0;
};

#endif // SETTINGSSERIALIZERBASE_H
