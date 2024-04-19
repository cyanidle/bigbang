#ifndef ROSPARAMS_H
#define ROSPARAMS_H

#include "serializer.hpp"
#include "jsondict/jsondict.hpp"

struct  RosParams : public Serializer::SerializerBase
{
    Q_GADGET
public:
    void update(const QString &name = "", const QString& prefix = "~", bool pedantic = true);
    virtual QVariantMap serialize() const override;
    virtual bool deserialize(const QVariantMap &src, bool pedantic = false, const QString &prefix = "~") override;
private:
    static JsonDict fetchFromStruct(const JsonDict &objStruct, const JsonDict &defaults, const QString& prefix);
    static QVariant receiveParam(const std::string &name, const QString &type, const QVariant& defaults);
    static QVariant receiveParamsList(const std::string &name, const QString &type, const QVariantList& defaults);
    static QVariant receiveParamsMap(const std::string &name, const QString &type, const QVariantMap& defaults);
};

#endif