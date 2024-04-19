#include "rosparams.h"
#include <ros/ros.h>

bool RosParams::deserialize(const QVariantMap &src, bool pedantic, const QString &prefix) 
{
    return SerializerBase::deserialize(src, pedantic, prefix);
}

QVariantMap RosParams::serialize() const
{
    return SerializerBase::serialize();
}

void RosParams::update(const QString &name, const QString& prefix, bool pedantic)
{
    auto defaults = JsonDict{serialize()};
    const JsonDict map = structure();
    if (!name.isEmpty()) {
        auto wantedKey = name.split("/"); 
        auto wantedType = JsonDict{map[wantedKey].toMap()};
        JsonDict toUse; 
        if (wantedType.firstKey().contains(LIST_MARKER)) {
            toUse[wantedKey] = receiveParamsList(name.toStdString(), wantedType.first().toString(), defaults[wantedKey].toList());
        } else if (wantedType.firstKey().contains(MAP_MARKER)) {
            toUse[wantedKey] = receiveParamsMap(name.toStdString(), wantedType.first().toString(), defaults[wantedKey].toMap());
        } else {
            toUse[wantedKey] = receiveParam(name.toStdString(), wantedType.first().toString(), defaults[wantedKey]);
        }
        deserialize(toUse, false, prefix);
        return;
    }
    for (auto &iter : map) {
        ROS_DEBUG_STREAM(iter.key().join("/").toStdString() << ": " << iter.value().toString().toStdString());
    }
    auto toUse = fetchFromStruct(map, defaults, prefix);
    deserialize(toUse, pedantic, prefix);
}

JsonDict RosParams::fetchFromStruct(const JsonDict &objStruct, const JsonDict &defaults, const QString& prefix) 
{
    JsonDict result;
    for (auto iter = objStruct.begin(); iter != objStruct.end(); ++iter) {
        auto fullKey = iter.key();
        QVariant wanted;
        if (fullKey.contains(LIST_MARKER)) {
            if (iter.field()!= LIST_MARKER) {
                throw std::runtime_error("Cannot get non-trivial types inside containers!");
            } else {
                fullKey = iter.domain();
                wanted = receiveParamsList((prefix + iter.domain().join("/")).toStdString(),
                 iter.value().toString(), defaults[fullKey].toList());
            }
        } else if (fullKey.contains(MAP_MARKER)) {
            if (iter.field()!= MAP_MARKER) {
                throw std::runtime_error("Cannot get non-trivial types inside containers!");
            } else {
                fullKey = iter.domain();
                wanted = receiveParamsMap((prefix + iter.domain().join("/")).toStdString(),
                 iter.value().toString(), defaults[fullKey].toMap());
            }
        } else {
            wanted = receiveParam((prefix + fullKey.join("/")).toStdString(),
                 iter.value().toString(), defaults[fullKey]);
        }
        if (wanted.isValid()) {
            result[fullKey] = wanted;
        }
    }
    return result;
}

#define GET_IMPL(typeImpl, suff...)                                 \
    typeImpl r;                                                     \
    if (!ros::param::get(name, r)) {                                \
        r = defaults.value<typeImpl>(); \
        ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name << "(" << r << ")");     \
        ros::param::set(name, r);          \
    }                                                               \
    return r  suff;

#define GET_IMPL_TYPED(typeImpl)                                    \
    else if (type == #typeImpl) {                                   \
        GET_IMPL(typeImpl)                                          \
    } 

#define GET_IMPL_MAP(typeImpl, suff...)                             \
    std::map<std::string, typeImpl> r;                              \
    if (!ros::param::get(name, r)) {                                \
        ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name);     \
        for (auto iter{defaults.begin()};iter!=defaults.end();++iter) \
            {r[iter.key().toStdString()] = iter.value().value<typeImpl>();} \
    }                                                               \
    QVariantMap subresult;                                          \
    for (auto iter = r.cbegin(); iter != r.cend(); ++iter) {        \
        subresult.insert(iter->first.c_str(), iter->second  suff);  \
    }                                                               \
    return subresult;                                           

#define GET_IMPL_MAP_TYPED(typeImpl)                                \
    else if (type == #typeImpl) {                                   \
        GET_IMPL_MAP(typeImpl)                                      \
    } 

#define GET_IMPL_LIST(typeImpl, suff...)                            \
    std::vector<typeImpl> r;                                        \
    if (!ros::param::get(name, r)) {                                \
        ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name);     \
        for (auto &val : defaults) {r.push_back(val.value<typeImpl>());} \
        ros::param::set(name, r);                                   \
    }                                                               \
    QVariantList subresult;                                         \
    for (auto &&subitem : r) {                                      \
        subresult.append(QVariant(subitem  suff));                  \
    }                                                               \
    return subresult;                                           

#define GET_IMPL_LIST_TYPED(typeImpl)                               \
    else if (type == #typeImpl) {                                   \
        GET_IMPL_LIST(typeImpl)                                     \
    } 

QVariant RosParams::receiveParam(const std::string &name, const QString &type, const QVariant& defaults)
{
    if (type == "QString") {
        std::string r;
        if (!ros::param::get(name, r)) {
            r = defaults.toString().toStdString();
            ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name << "(" << r << ")");
            ros::param::set(name, r);
        }
        return QString(r.c_str());
    }
    GET_IMPL_TYPED(float)
    GET_IMPL_TYPED(double)
    GET_IMPL_TYPED(int)
    GET_IMPL_TYPED(bool)
    else 
    {
        ROS_WARN_STREAM("Unknown type: " << type.toStdString() << ": for" << name);
        abort();
    }
}

QVariant RosParams::receiveParamsMap(const std::string &name, const QString &type, const QVariantMap& defaults)
{
    if (type == "QString") {
        std::map<std::string, std::string> r;
        if (!ros::param::get(name, r)) {
            ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name);
            for (auto iter{defaults.begin()};iter!=defaults.end();++iter) {
                r[iter.key().toStdString()] = iter.value().toString().toStdString();
            }
            ros::param::set(name, r);
        }
        QVariantMap subresult;
        for (auto iter = r.cbegin(); iter != r.cend(); ++iter) {
            subresult.insert(iter->first.c_str(), iter->second.c_str());
        }
        return subresult;
    }
    GET_IMPL_MAP_TYPED(float)
    GET_IMPL_MAP_TYPED(double)
    GET_IMPL_MAP_TYPED(int)
    GET_IMPL_MAP_TYPED(bool)
    else 
    {
        ROS_WARN_STREAM("Unknown type for map: " << type.toStdString());
        abort();
    }
}

QVariant RosParams::receiveParamsList(const std::string &name, const QString &type, const QVariantList& defaults)
{
    if (type.toStdString() == "QString") {
        std::vector<std::string> r;
        if (!ros::param::get(name, r)) {
            ROS_WARN_STREAM(ros::this_node::getName() << ": Setting default value for: " << name);
            for (auto &val : defaults) {r.push_back(val.toString().toStdString());}
            ros::param::set(name, r);
        }
        QVariantList subresult;
        for (auto &&subitem : r) {
            subresult.append(QString(subitem.c_str()));
        }
        return subresult;
    }
    GET_IMPL_LIST_TYPED(float)
    GET_IMPL_LIST_TYPED(double)
    GET_IMPL_LIST_TYPED(int)
    GET_IMPL_LIST_TYPED(bool)
    else 
    {
        ROS_WARN_STREAM("Unknown type for list: " << type.toStdString());
        abort();
    }
}
