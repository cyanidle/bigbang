#pragma once

#include <QCoreApplication>
#include <QObject>
#include <QTimer>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "describe/describe.hpp"

inline ros::NodeHandle* nh() {
    static ros::NodeHandle _nh;
    return &_nh;
}

template<typename Node>
int node_main(int argc, char* argv[], const char* node_name) {
    QCoreApplication app(argc, argv);
    ros::init(argc, argv, node_name);
    return Node{}, app.exec();
}

#define NODE_MAIN(node, node_name) \
int main(int argc, char* argv[]) { \
    return node_main<node>(argc, argv, node_name); \
}


class NodeBase : public QObject
{
    Q_OBJECT
public:
    NodeBase();
    void shutdown();
};

template<typename T>
void DeserializeFromRos(T& out, std::string key = "~") {
    if constexpr (describe::is_described_v<T>) {
        describe::Get<T>().for_each_field([&](auto f){
            DeserializeFromRos(f.get(out), key+'/'+std::string{f.name});
        });
    } else {
        ros::param::get(key, out);
    }
}

inline float thetaFromMsg(const geometry_msgs::Quaternion &quat)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

inline geometry_msgs::Quaternion msgFromTheta(float theta)
{
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    geometry_msgs::Quaternion result;
    tf::quaternionTFToMsg(q, result);
    return result;
}
